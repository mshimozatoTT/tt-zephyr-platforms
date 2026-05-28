#!/usr/bin/env python3
"""
Capture CSM clock_pattern + power_pattern from CMFW, write a CSV, and plot it.

Runs until Ctrl-C / SIGTERM (or until --duration-ms expires firmware-side),
then cleanly stops the capture, reads CSM, writes a CSV, and saves a PNG
plot of AICLK + input_power vs time.

CMFW infrastructure used (see lib/tenstorrent/bh_arc/aiclk_ppm.c,
power_pattern.c, capture_buffer.c and include/tenstorrent/smc_msg.h):

  TT_SMC_MSG_CHARACTERISATION (0xC6) submessages:
    0x2  TT_SUB_MSG_START_CLOCK_COUNTER     payload: delay_ms, on_go_busy, dur_ms
    0x3  TT_SUB_MSG_STOP_CLOCK_COUNTER
    0x4  TT_SUB_MSG_GET_CLOCK_PATTERN_INFO  -> VMA, capacity, evt_bytes, divisor,
                                              magic 0x02636c70 (v2 packs the
                                              dominant arb_max throttler ID
                                              into bits 12..15 of mhz),
                                              data[6] = (avg_mhz<<16)|fill_idx,
                                              data[7] = ring_wrapped
    0x5  TT_SUB_MSG_START_POWER_COUNTER     payload: delay_ms, on_go_busy, dur_ms
    0x6  TT_SUB_MSG_STOP_POWER_COUNTER
    0x7  TT_SUB_MSG_GET_POWER_PATTERN_INFO  -> VMA, capacity, 2,
                                              divisor, magic 0x01727770,
                                              fill_idx, ring_wrapped

power_pattern[] stores GetInputPower() (board input / PSYS) in centiwatts.

Each clock event is annotated with the @c aiclk_arb_max throttler that was
binding AICLK at the moment of the transition; the host script unpacks this
into the @c throttler column of the CSV so you can correlate AICLK floors
and ceilings to a specific throttler (e.g. doppler_critical vs doppler_slow).

CSV layout (wide; blank cells mean "no sample at this t"):

    t_ms, aiclk_mhz, throttler, input_power_w
"""
import argparse
import atexit
import csv
import os
import signal
import struct
import sys
import time

import pyluwen

TT_SMC_MSG_CHARACTERISATION = 0xC6
TT_SMC_MSG_THROTTLER_ASYMMETRIC_EN = 0x3A

SUB_START_CLOCK = 0x2
SUB_STOP_CLOCK = 0x3
SUB_GET_CLOCK_INFO = 0x4
SUB_START_POWER = 0x5
SUB_STOP_POWER = 0x6
SUB_GET_POWER_INFO = 0x7

CLOCK_PATTERN_INFO_MAGIC = 0x02636C70
POWER_PATTERN_INFO_MAGIC = 0x01727770

CLOCK_EVENT_BYTES = 6
POWER_SAMPLE_BYTES = 2

# Mirror of enum aiclk_arb_max in lib/tenstorrent/bh_arc/aiclk_ppm.h.
# Index = throttler ID packed into bits 12..15 of clock_pattern_event.mhz.
# Index >= len(ARB_MAX_NAMES) is reported by CMFW when no dominant arbiter
# could be identified at sample time; we render that as "unknown".
ARB_MAX_NAMES = [
    "fmax",             # 0  aiclk_arb_max_fmax
    "tdp",              # 1  aiclk_arb_max_tdp
    "fast_tdc",         # 2  aiclk_arb_max_fast_tdc
    "tdc",              # 3  aiclk_arb_max_tdc
    "thm",              # 4  aiclk_arb_max_thm
    "board_power",      # 5  aiclk_arb_max_board_power
    "voltage",          # 6  aiclk_arb_max_voltage
    "gddr_thm",         # 7  aiclk_arb_max_gddr_thm
    "doppler_slow",     # 8  aiclk_arb_max_doppler_slow
    "doppler_critical", # 9  aiclk_arb_max_doppler_critical
    "host_fmax",        # 10 aiclk_arb_max_host_fmax
]


def arb_name(arb_id):
    if 0 <= arb_id < len(ARB_MAX_NAMES):
        return ARB_MAX_NAMES[arb_id]
    return f"unknown({arb_id})"


def _send(chip, sub_id, w1=0, w2=0, w3=0):
    header = TT_SMC_MSG_CHARACTERISATION | (sub_id & 0xFF) << 8
    return chip.as_bh().arc_msg_buf([header, w1, w2, w3, 0, 0, 0, 0])


def set_asymmetric_pd(chip, enable):
    """Toggle the on-chip asymmetric PD law via TT_SMC_MSG_THROTTLER_ASYMMETRIC_EN."""
    header = TT_SMC_MSG_THROTTLER_ASYMMETRIC_EN | ((1 if enable else 0) & 0xFF) << 8
    r = chip.as_bh().arc_msg_buf([header, 0, 0, 0, 0, 0, 0, 0])
    if r[0] != 0:
        raise RuntimeError(
            f"TT_SMC_MSG_THROTTLER_ASYMMETRIC_EN(enable={int(bool(enable))}) "
            f"failed: rc={r[0]} (is CMFW out of date?)")


def _axi_read_bytes(chip, addr, nbytes):
    """Bulk read CSM as little-endian 32-bit words, returning raw bytes."""
    start = addr & ~0x3
    end = (addr + nbytes + 3) & ~0x3
    buf = bytearray()
    for a in range(start, end, 4):
        buf += struct.pack("<I", chip.axi_read32(a))
    off = addr - start
    return bytes(buf[off:off + nbytes])


def start_capture(chip, delay_ms=0, capture_duration_ms=0, on_go_busy=0):
    # Stop first, in case a prior run left them armed.
    _send(chip, SUB_STOP_CLOCK)
    _send(chip, SUB_STOP_POWER)

    r = _send(chip, SUB_START_CLOCK, delay_ms, on_go_busy, capture_duration_ms)
    if r[0] != 0:
        raise RuntimeError(f"START_CLOCK_COUNTER failed: rc={r[0]}")
    r = _send(chip, SUB_START_POWER, delay_ms, on_go_busy, capture_duration_ms)
    if r[0] != 0:
        raise RuntimeError(f"START_POWER_COUNTER failed: rc={r[0]}")


def stop_capture(chip):
    _send(chip, SUB_STOP_CLOCK)
    _send(chip, SUB_STOP_POWER)


def get_clock_info(chip):
    r = _send(chip, SUB_GET_CLOCK_INFO)
    if r[0] != 0:
        raise RuntimeError(f"GET_CLOCK_PATTERN_INFO failed: rc={r[0]}")
    if r[5] != CLOCK_PATTERN_INFO_MAGIC:
        raise RuntimeError(
            f"Clock pattern magic mismatch: got 0x{r[5]:08x}, expected "
            f"0x{CLOCK_PATTERN_INFO_MAGIC:08x} - is CMFW out of date?")
    return {
        "vma": r[1],
        "capacity": r[2],
        "evt_bytes": r[3],
        "divisor": max(1, r[4]),
        "fill_idx": r[6] & 0xFFFF,
        "avg_mhz": (r[6] >> 16) & 0xFFFF,
        "wrapped": bool(r[7]),
    }


def get_power_info(chip):
    r = _send(chip, SUB_GET_POWER_INFO)
    if r[0] != 0:
        raise RuntimeError(f"GET_POWER_PATTERN_INFO failed: rc={r[0]}")
    if r[5] != POWER_PATTERN_INFO_MAGIC:
        raise RuntimeError(
            f"Power pattern magic mismatch: got 0x{r[5]:08x}, expected "
            f"0x{POWER_PATTERN_INFO_MAGIC:08x} - is CMFW out of date?")
    return {
        "vma": r[1],
        "capacity": r[2],
        "sample_bytes": r[3],
        "divisor": max(1, r[4]),
        "fill_idx": r[6],
        "wrapped": bool(r[7]),
    }


def read_clock_events(chip, info):
    """Return (seq, mhz, arb_id) tuples in chronological order.

    @c arb_id is the dominant @c aiclk_arb_max throttler at the moment of the
    transition (unpacked from bits 12..15 of the stored @c mhz field; the low
    12 bits hold the applied MHz). See ARB_MAX_NAMES for the mapping.
    """
    cap = info["capacity"]
    n = cap if info["wrapped"] else info["fill_idx"]
    if n == 0:
        return []
    raw = _axi_read_bytes(chip, info["vma"], cap * CLOCK_EVENT_BYTES)
    events = []
    for i in range(cap):
        seq, packed = struct.unpack_from("<IH", raw, i * CLOCK_EVENT_BYTES)
        mhz = packed & 0x0FFF
        arb_id = (packed >> 12) & 0x0F
        events.append((seq, mhz, arb_id))
    if info["wrapped"]:
        # Ring: oldest at fill_idx, newest at fill_idx-1.
        head = info["fill_idx"] % cap
        events = events[head:] + events[:head]
    else:
        events = events[:n]
    return events


def read_power_samples(chip, info):
    """Return uint16 centiwatt samples in chronological order."""
    cap = info["capacity"]
    n = cap if info["wrapped"] else info["fill_idx"]
    if n == 0:
        return []
    raw = _axi_read_bytes(chip, info["vma"], cap * POWER_SAMPLE_BYTES)
    samples = list(struct.unpack(f"<{cap}H", raw))
    if info["wrapped"]:
        head = info["fill_idx"] % cap
        samples = samples[head:] + samples[:head]
    else:
        samples = samples[:n]
    return samples


def build_rows(clock_events, clock_divisor, power_samples, power_divisor):
    """Merge the two signals into a sparse wide table keyed by t_ms.

    Each row is (t_ms, aiclk_mhz, throttler, input_power_w). The throttler
    field is the human-readable name of the dominant arb_max at the moment
    of the AICLK transition, or None on rows that only carry a power sample.
    """
    # t_ms -> [aiclk_mhz, throttler_name, input_power_w]
    rows = {}

    # AICLK: sparse, seq is in units of clock-sample periods (1 ms * divisor).
    for seq, mhz, arb_id in clock_events:
        t_ms = seq * clock_divisor
        slot = rows.setdefault(t_ms, [None, None, None])
        slot[0] = mhz
        slot[1] = arb_name(arb_id)

    # input_power: dense, one sample every (1 ms * power_divisor).
    for i, cw in enumerate(power_samples):
        t_ms = i * power_divisor
        rows.setdefault(t_ms, [None, None, None])[2] = cw / 100.0

    return [(t,) + tuple(v) for t, v in sorted(rows.items())]


def write_csv(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_ms", "aiclk_mhz", "throttler", "input_power_w"])
        for r in rows:
            w.writerow(["" if x is None else x for x in r])


def rolling_mean(values, window):
    """Centered N-sample rolling mean. O(n), no numpy needed."""
    if window <= 1 or len(values) < 2:
        return list(values)
    half = window // 2
    n = len(values)
    pref = [0.0] * (n + 1)
    for i, v in enumerate(values):
        pref[i + 1] = pref[i] + v
    out = [0.0] * n
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        out[i] = (pref[hi] - pref[lo]) / (hi - lo)
    return out


def _preload_matplotlib():
    """Import matplotlib eagerly so the first-time import (which can take ~1 s
    and traverses dozens of submodules) doesn't sit inside the atexit cleanup
    where a second Ctrl-C would tear it down mid-flight. Returns the pyplot
    module on success, or None if matplotlib isn't installed."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        return plt
    except ImportError:
        return None


# Stable colour per arb_max so the same throttler reads the same colour across
# captures. doppler_critical is forced red because that's the bang-bang
# protection event we most want to flag visually.
_ARB_COLORS = {
    "fmax":             "#1f77b4",  # blue
    "tdp":              "#ff7f0e",  # orange
    "fast_tdc":         "#8c564b",  # brown
    "tdc":              "#e377c2",  # pink
    "thm":              "#bcbd22",  # olive
    "board_power":      "#2ca02c",  # green
    "voltage":          "#17becf",  # cyan
    "gddr_thm":         "#9467bd",  # purple
    "doppler_slow":     "#7f7f7f",  # grey
    "doppler_critical": "#d62728",  # red
    "host_fmax":        "#000000",  # black
}


def _color_for_arb(name):
    return _ARB_COLORS.get(name, "#999999")


def plot_rows(rows, png_path, smooth, title, plt=None):
    """Render AICLK (step) + input_power (line, optionally smoothed) to PNG.

    AICLK transitions are plotted as a faint step line plus markers coloured
    by the dominant @c aiclk_arb_max throttler at the moment of the
    transition (see ARB_MAX_NAMES). This makes it visually obvious which
    throttler is causing each floor / ceiling event.
    """
    if plt is None:
        plt = _preload_matplotlib()
        if plt is None:
            raise RuntimeError("matplotlib is not installed; use --no-plot")

    t_clk, clk, clk_arb = [], [], []
    t_in, inp = [], []
    for t, mhz, throttler, w in rows:
        if mhz is not None:
            t_clk.append(t)
            clk.append(float(mhz))
            clk_arb.append(throttler or "unknown")
        if w is not None:
            t_in.append(t)
            inp.append(float(w))

    if smooth > 1 and inp:
        inp = rolling_mean(inp, smooth)

    if not t_clk and not t_in:
        print("[log_capture] no samples to plot", file=sys.stderr)
        return

    fig, ax_clk = plt.subplots(figsize=(12, 5))
    ax_pow = ax_clk.twinx()

    if t_clk:
        # Faint backbone showing the step trace, then coloured markers per
        # throttler on top so the eye can group floor / ceiling events by
        # cause.
        ax_clk.step(t_clk, clk, where="post", color="#1f77b4",
                    linewidth=0.8, alpha=0.35, label="AICLK (MHz)")
        for throttler in sorted(set(clk_arb)):
            xs = [t for t, a in zip(t_clk, clk_arb) if a == throttler]
            ys = [y for y, a in zip(clk, clk_arb) if a == throttler]
            ax_clk.scatter(xs, ys, s=18,
                           color=_color_for_arb(throttler),
                           edgecolors="none",
                           label=f"→ {throttler} (n={len(xs)})",
                           zorder=5)
    ax_clk.set_xlabel("time (ms)")
    ax_clk.set_ylabel("AICLK (MHz)", color="#1f77b4")
    ax_clk.tick_params(axis="y", labelcolor="#1f77b4")
    ax_clk.grid(True, axis="x", alpha=0.3)

    pow_line = None
    if t_in:
        power_label = "input_power (W)"
        if smooth > 1:
            power_label = f"input_power (W, rolling-mean N={smooth})"
        pow_line, = ax_pow.plot(t_in, inp, color="#2ca02c", linewidth=1.0,
                                alpha=0.85, label=power_label)
    ax_pow.set_ylabel("input power (W)", color="#2ca02c")
    ax_pow.tick_params(axis="y", labelcolor="#2ca02c")

    # Combine handles so the throttler legend and the power line share one box.
    handles, labels = ax_clk.get_legend_handles_labels()
    if pow_line is not None:
        handles.append(pow_line)
        labels.append(pow_line.get_label())
    ax_clk.legend(handles, labels, loc="upper right", fontsize="small",
                  framealpha=0.9)
    ax_clk.set_title(title)
    fig.tight_layout()
    fig.savefig(png_path, dpi=150)
    plt.close(fig)
    return True


def summarize_throttlers(events):
    """Return a list of (throttler_name, count) sorted by count desc."""
    counts = {}
    for _seq, _mhz, arb_id in events:
        name = arb_name(arb_id)
        counts[name] = counts.get(name, 0) + 1
    return sorted(counts.items(), key=lambda kv: -kv[1])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--asic-id", type=int, default=0)
    ap.add_argument("--output", "-o", default="capture.csv",
                    help="Output CSV path (default: capture.csv). The PNG is "
                         "saved alongside with the same basename unless "
                         "--plot-path is given.")
    ap.add_argument("--plot-path", default=None,
                    help="PNG output path (default: <output>.png)")
    ap.add_argument("--smooth", type=int, default=50,
                    help="Apply an N-sample centered rolling mean to "
                         "input_power before plotting (1 = no smoothing). "
                         "AICLK is never smoothed.")
    ap.add_argument("--no-plot", action="store_true",
                    help="Write only the CSV; skip plotting.")
    ap.add_argument("--duration-ms", type=int, default=0,
                    help="Optional firmware-side auto-stop after N ms "
                         "(0 = run until Ctrl-C; firmware caps at 300000)")
    ap.add_argument("--asymmetric-pid", action=argparse.BooleanOptionalAction,
                    default=True,
                    help="Enable the on-chip asymmetric PD law for the run "
                         "(default: enabled). Use --no-asymmetric-pid to run "
                         "the legacy linear law for comparison. Sent via "
                         "TT_SMC_MSG_THROTTLER_ASYMMETRIC_EN before capture "
                         "arms; the chip retains this setting until changed.")
    ap.add_argument("--title", default=None,
                    help="Override plot title (defaults to CSV path)")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    # Preload matplotlib up-front (not inside the atexit cleanup) so that a
    # second Ctrl-C during shutdown can't interrupt the import and leave us
    # without a PNG. Skipped when plotting was disabled anyway.
    plt_module = None if args.no_plot else _preload_matplotlib()
    if not args.no_plot and plt_module is None and not args.quiet:
        print("[log_capture] matplotlib not installed; CSV will be written "
              "but PNG will be skipped", file=sys.stderr)

    chip = pyluwen.detect_chips()[args.asic_id]

    set_asymmetric_pd(chip, args.asymmetric_pid)
    if not args.quiet:
        print(f"[log_capture] asymmetric PD law: "
              f"{'ENABLED' if args.asymmetric_pid else 'DISABLED (legacy linear)'}",
              file=sys.stderr)

    start_capture(chip, capture_duration_ms=args.duration_ms)
    started = True
    if not args.quiet:
        print(f"[log_capture] capture armed on asic {args.asic_id}; "
              f"Ctrl-C to stop. Output: {args.output}", file=sys.stderr)

    def _cleanup():
        nonlocal started
        if not started:
            return
        started = False
        # Ignore further SIGINT/SIGTERM during cleanup so a second Ctrl-C
        # can't tear down the CSV write or the plot mid-flight.
        try:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
        except (ValueError, OSError):
            pass
        try:
            stop_capture(chip)
        except Exception as e:
            print(f"[log_capture] stop failed: {e}", file=sys.stderr)
        try:
            ci = get_clock_info(chip)
            pi = get_power_info(chip)
            events = read_clock_events(chip, ci)
            samples = read_power_samples(chip, pi)
        except Exception as e:
            print(f"[log_capture] readback failed: {e}", file=sys.stderr)
            return
        rows = build_rows(events, ci["divisor"], samples, pi["divisor"])
        write_csv(args.output, rows)
        if not args.quiet:
            print(f"[log_capture] wrote {len(rows)} rows to {args.output} "
                  f"(clock events={len(events)}, power samples={len(samples)})",
                  file=sys.stderr)
            # Per-throttler attribution: which arb_max caused each AICLK
            # transition? Critical for diagnosing "is doppler_critical
            # bang-banging?" without staring at the plot.
            dist = summarize_throttlers(events)
            if dist:
                pretty = ", ".join(f"{n}={c}" for n, c in dist)
                print(f"[log_capture] throttler attribution (per AICLK event): "
                      f"{pretty}", file=sys.stderr)
        if args.no_plot or plt_module is None:
            return
        png_path = args.plot_path or (os.path.splitext(args.output)[0] + ".png")
        title = args.title or args.output
        try:
            plot_rows(rows, png_path, smooth=args.smooth, title=title,
                      plt=plt_module)
            if not args.quiet:
                print(f"[log_capture] wrote {png_path}", file=sys.stderr)
        except Exception as e:
            print(f"[log_capture] plot step failed: {e}", file=sys.stderr)

    atexit.register(_cleanup)
    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))

    try:
        while True:
            time.sleep(1.0)
    except SystemExit:
        pass


if __name__ == "__main__":
    main()

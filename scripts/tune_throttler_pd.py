#!/usr/bin/env python3
"""Tune the SMC throttler asymmetric PD/PI/PID-loop parameters at runtime."""
import argparse
import struct
import pyluwen

TT_SMC_MSG_THROTTLER_PD_PARAM = 0x36

OP_GET, OP_SET = 0, 1

THROTTLER = {
    "tdp": 0, "fast_tdc": 1, "tdc": 2, "thm": 3,
    "board_power": 4, "gddr_thm": 5, "doppler_slow": 6,
}

PARAM = {
    "alpha_filter":   (0, "float"),
    "p_gain":         (1, "float"),  # under-limit proportional gain
    "d_gain":         (2, "float"),  # under-limit derivative gain
    "p_gain_over":    (3, "float"),  # over-limit proportional gain
    "d_gain_over":    (4, "float"),  # over-limit derivative gain
    "deadband_under": (5, "float"),
    "deadband_over":  (6, "float"),
    "du_max_up":      (7, "float"),
    "du_max_down":    (8, "float"),
    "i_gain":         (9, "float"),   # under-limit integral gain (0 -> PD, non-zero -> PI/PID)
    "i_gain_over":    (10, "float"),  # over-limit integral gain
}


def _send(chip, op, tid, pid, value_u32=0):
    header = (TT_SMC_MSG_THROTTLER_PD_PARAM
              | (op & 0xFF) << 8
              | (tid & 0xFF) << 16
              | (pid & 0xFF) << 24)
    return chip.as_bh().arc_msg_buf([header, value_u32 & 0xFFFFFFFF, 0, 0, 0, 0, 0, 0])


def get(chip, throttler, param):
    tid = THROTTLER[throttler]
    pid, kind = PARAM[param]
    rsp = _send(chip, OP_GET, tid, pid)
    if rsp[0] != 0:
        raise RuntimeError(f"GET {throttler}.{param} failed (rc={rsp[0]})")
    bits = rsp[1]
    if kind == "bool":
        return bool(bits)
    return struct.unpack("<f", struct.pack("<I", bits & 0xFFFFFFFF))[0]


def set_(chip, throttler, param, value):
    tid = THROTTLER[throttler]
    pid, kind = PARAM[param]
    if kind == "bool":
        bits = 1 if value else 0
    else:
        bits = struct.unpack("<I", struct.pack("<f", float(value)))[0]
    rsp = _send(chip, OP_SET, tid, pid, bits)
    if rsp[0] != 0:
        raise RuntimeError(f"SET {throttler}.{param}={value} rejected (rc={rsp[0]})")


def dump(chip, throttler="tdp"):
    print(f"--- {throttler} ---")
    for name in PARAM:
        print(f"  {name:16s} = {get(chip, throttler, name)}")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--asic-id", type=int, default=0)
    sub = p.add_subparsers(dest="cmd", required=True)

    g = sub.add_parser("dump");  g.add_argument("--throttler", default="tdp")
    g = sub.add_parser("get");   g.add_argument("--throttler", default="tdp"); g.add_argument("param")
    s = sub.add_parser("set")
    s.add_argument("--throttler", default="tdp"); s.add_argument("param"); s.add_argument("value")

    args = p.parse_args()
    chip = pyluwen.detect_chips()[args.asic_id]

    if args.cmd == "dump":
        dump(chip, args.throttler)
    elif args.cmd == "get":
        print(get(chip, args.throttler, args.param))
    elif args.cmd == "set":
        # Accept "1" / "true" for booleans, otherwise float.
        v = args.value
        if v.lower() in ("true", "false"):
            v = v.lower() == "true"
        else:
            try:    v = float(v)
            except ValueError: v = int(v)
        set_(chip, args.throttler, args.param, v)
        print(f"OK: {args.throttler}.{args.param} = {get(chip, args.throttler, args.param)}")

#!/usr/bin/env python3

# Copyright (c) 2026 Tenstorrent AI ULC
# SPDX-License-Identifier: Apache-2.0

"""
Exercise TT_SUB_MSG_SET_SERDES_GDDR_VCOREM_TO_TELEM on a live BH chip.

Examples:
  # Full enable / read / disable sequence
  ./scripts/test_block_power_telem.py

  # Just print current state
  ./scripts/test_block_power_telem.py status

  # Enable and leave enabled
  ./scripts/test_block_power_telem.py enable

  # Disable
  ./scripts/test_block_power_telem.py disable

  # Chip index 1
  ./scripts/test_block_power_telem.py --asic-id 1 enable
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import pyluwen
except ModuleNotFoundError:
    print("pyluwen is required (run from the firmware venv)", file=sys.stderr)
    sys.exit(1)

TELEMETRY_DATA_REG_ADDR = 0x80030430

TT_SMC_MSG_CHARACTERISATION = 0xC6
TT_SUB_MSG_SET_SERDES_GDDR_VCOREM_TO_TELEM = 0x4

TAG_FW_CAPABILITIES_0 = 78
TAG_FW_ACTIVE_CONFIG_0 = 79
TAG_VCOREM_POWER = 80
TAG_GDDR_POWER = 81
TAG_SERDES_POWER = 82
TAG_VCORE_POWER = 83
SERDES_GDDR_VCOREM_TELEM_BIT = 1

POWER_TAGS = (
    (TAG_VCOREM_POWER, "VCOREM"),
    (TAG_GDDR_POWER, "GDDR"),
    (TAG_SERDES_POWER, "SERDES"),
    (TAG_VCORE_POWER, "VCORE"),
)


def read_telem(arc_chip, tag: int) -> int:
    table_addr = arc_chip.axi_read32(TELEMETRY_DATA_REG_ADDR)
    return arc_chip.axi_read32(table_addr + tag * 4)


def set_block_power_telem(arc_chip, enabled: int):
    return arc_chip.as_bh().arc_msg_buf(
        [
            TT_SMC_MSG_CHARACTERISATION
            | (TT_SUB_MSG_SET_SERDES_GDDR_VCOREM_TO_TELEM << 8),
            enabled,
            0,
            0,
            0,
            0,
            0,
            0,
        ]
    )


def bit_set(value: int) -> bool:
    return bool(value & (1 << SERDES_GDDR_VCOREM_TELEM_BIT))


def print_status(arc_chip) -> None:
    caps = read_telem(arc_chip, TAG_FW_CAPABILITIES_0)
    active = read_telem(arc_chip, TAG_FW_ACTIVE_CONFIG_0)
    print(f"TAG_FW_CAPABILITIES_0 = 0x{caps:08x}  bit1={int(bit_set(caps))}")
    print(f"TAG_FW_ACTIVE_CONFIG_0 = 0x{active:08x}  bit1={int(bit_set(active))}")
    for tag, name in POWER_TAGS:
        print(f"  TAG_{name}_POWER ({tag}) = {read_telem(arc_chip, tag)} W")


def cmd_status(arc_chip) -> int:
    print_status(arc_chip)
    if not bit_set(read_telem(arc_chip, TAG_FW_CAPABILITIES_0)):
        print("WARN: capability bit not set — FW may not include this feature")
        return 1
    return 0


def cmd_enable(arc_chip) -> int:
    resp = set_block_power_telem(arc_chip, 1)
    print(f"enable response[0]={resp[0]}")
    if resp[0] != 0:
        return 1
    time.sleep(0.5)
    print_status(arc_chip)
    return 0 if bit_set(read_telem(arc_chip, TAG_FW_ACTIVE_CONFIG_0)) else 1


def cmd_disable(arc_chip) -> int:
    resp = set_block_power_telem(arc_chip, 0)
    print(f"disable response[0]={resp[0]}")
    if resp[0] != 0:
        return 1
    time.sleep(0.2)
    print_status(arc_chip)
    return 0 if not bit_set(read_telem(arc_chip, TAG_FW_ACTIVE_CONFIG_0)) else 1


def cmd_test(arc_chip) -> int:
    """Full sequence: check capability, enable, read, reject bad value, disable."""
    failures = 0

    caps = read_telem(arc_chip, TAG_FW_CAPABILITIES_0)
    print(f"[1] capabilities=0x{caps:08x}")
    if not bit_set(caps):
        print("FAIL: capability bit 1 not set")
        return 1
    print("PASS: capability bit present")

    active = read_telem(arc_chip, TAG_FW_ACTIVE_CONFIG_0)
    print(f"[2] active before enable=0x{active:08x}")
    if bit_set(active):
        print("NOTE: feature already enabled; continuing")

    resp = set_block_power_telem(arc_chip, 1)
    print(f"[3] enable -> response[0]={resp[0]}")
    if resp[0] != 0:
        print("FAIL: enable rejected")
        return 1
    time.sleep(0.5)

    active = read_telem(arc_chip, TAG_FW_ACTIVE_CONFIG_0)
    if not bit_set(active):
        print(f"FAIL: active bit not set after enable (0x{active:08x})")
        failures += 1
    else:
        print(f"PASS: active=0x{active:08x}")

    print("[4] power tags after enable:")
    for tag, name in POWER_TAGS:
        value = read_telem(arc_chip, tag)
        print(f"  {name:6s} = {value} W")

    resp = set_block_power_telem(arc_chip, 2)
    print(f"[5] invalid enable=2 -> response[0]={resp[0]}")
    if resp[0] == 0:
        print("FAIL: invalid value was accepted")
        failures += 1
    else:
        print("PASS: invalid value rejected")

    resp = set_block_power_telem(arc_chip, 0)
    print(f"[6] disable -> response[0]={resp[0]}")
    if resp[0] != 0:
        print("FAIL: disable rejected")
        failures += 1
        return 1
    time.sleep(0.2)

    active = read_telem(arc_chip, TAG_FW_ACTIVE_CONFIG_0)
    if bit_set(active):
        print(f"FAIL: active bit still set after disable (0x{active:08x})")
        failures += 1
    else:
        print(f"PASS: active cleared (0x{active:08x})")

    print("[7] power tags after disable (expect 0):")
    for tag, name in POWER_TAGS:
        value = read_telem(arc_chip, tag)
        ok = value == 0
        print(f"  {name:6s} = {value} W  {'PASS' if ok else 'FAIL'}")
        if not ok:
            failures += 1

    print("----")
    print("PASS" if failures == 0 else f"FAIL ({failures} check(s) failed)")
    return 0 if failures == 0 else 1


def parse_args():
    parser = argparse.ArgumentParser(
        description="Test SERDES/GDDR/VCOREM block-power telemetry ARC message",
        allow_abbrev=False,
    )
    parser.add_argument(
        "command",
        nargs="?",
        default="test",
        choices=("test", "status", "enable", "disable"),
        help="Action to run (default: test)",
    )
    parser.add_argument(
        "--asic-id",
        type=int,
        default=0,
        help="Index into pyluwen.detect_chips() (default: 0)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    chips = pyluwen.detect_chips()
    if not chips:
        print("No chips detected", file=sys.stderr)
        return 1
    if args.asic_id >= len(chips):
        print(
            f"--asic-id {args.asic_id} out of range ({len(chips)} chip(s))",
            file=sys.stderr,
        )
        return 1

    arc_chip = chips[args.asic_id]
    print(f"Using chip[{args.asic_id}]")

    handlers = {
        "test": cmd_test,
        "status": cmd_status,
        "enable": cmd_enable,
        "disable": cmd_disable,
    }
    return handlers[args.command](arc_chip)


if __name__ == "__main__":
    sys.exit(main())

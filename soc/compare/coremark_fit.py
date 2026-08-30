#!/usr/bin/env python3
"""Report a CoreMark image against the memory the comparison harness can place.

THE ANSWER THIS PRINTS IS NORMALLY "IT DOES NOT FIT", AND THAT IS THE POINT --
the same one soc/compare/dhry_fit.py already makes for Dhrystone. `make
compare-timing` places each core on an ice40 hx8k, which has 32
`SB_RAM40_4K` and no SPRAM -- 16 KB of memory for everything, including each
core's own register file. CoreMark's own image is roughly four times
Dhrystone's even at RV32IMA with no compressed encodings, so it runs in
simulation at a geometry no ice40 in this flow can hold, and the cycle counts
it produces have to be multiplied by a clock measured at the smaller placed
geometry. That mismatch is the headline caveat on the result and this script
prints it, every run, next to the numbers.

Trimming `TOTAL_DATA_SIZE` or the iteration count to force a fit is not an
option: a shrunk CoreMark is not CoreMark and its number could not be compared
with anything, which is the whole reason for running one.

Two things here are graded rather than described:

  - the simulated geometry stated by soc/compare/coremark.lds must be the
    geometry soc/compare/coremark_tb.v instantiates. Two files, one map; a
    divergence would leave the image and the memories that hold it describing
    different machines, and both would still run.
  - the image must fit that simulated geometry. `ld` already refuses a
    `.text` overflow, but `.bss` runs past the end of a too-small RAM without
    a word from anyone.

The block-RAM counts each core needs before either memory are read out of the
yosys census for that core synthesised alone -- the same logs
soc/compare/placed_vs_synth.py grades against -- rather than copied from a
table here.
"""

import argparse
import re
import sys

# yosys `stat`, the same line shape soc/compare/placed_vs_synth.py reads for
# SB_LUT4.
BLOCKS = re.compile(r"^\s+(\d+)\s+SB_RAM40_4K\s*$", re.M)
# `localparam int ROM_WORDS = 4096;` in the testbench.
PARAM = r"^\s*localparam\s+int\s+{}\s*=\s*(\d+)\s*;"

# One SB_RAM40_4K is 4096 bits, and yosys builds a 32-bit word out of two of
# them 256 words deep. So a 32-bit memory costs two blocks per 256 words.
WORDS_PER_BLOCK_PAIR = 256


def blocks_for(byte_count):
    words = (byte_count + 3) // 4
    pairs = (words + WORDS_PER_BLOCK_PAIR - 1) // WORDS_PER_BLOCK_PAIR
    return pairs * 2


def read_core_blocks(spec):
    """`name=path` -> (name, blocks), from that core's standalone yosys census."""
    if "=" not in spec:
        sys.exit(f"--core wants name=path, got '{spec}'")
    name, path = spec.split("=", 1)
    try:
        with open(path) as handle:
            text = handle.read()
    except OSError as exc:
        sys.exit(f"cannot read the standalone synthesis log for {name}: {exc}")
    found = BLOCKS.findall(text)
    if not found:
        sys.exit(
            f"no SB_RAM40_4K line in {path}, so how much block RAM {name} needs\n"
            "before either memory is unknown. A core that maps none would print a\n"
            "zero line; a log with no census at all is a synthesis that did not\n"
            "finish, and reporting it as zero would understate the shortfall."
        )
    return name, int(found[-1])


def read_tb_param(path, name):
    try:
        with open(path) as handle:
            text = handle.read()
    except OSError as exc:
        sys.exit(f"cannot read the testbench: {exc}")
    match = re.search(PARAM.format(name), text, re.M)
    if not match:
        sys.exit(
            f"no `localparam int {name}` in {path}. This check compares the\n"
            "simulated geometry the linker script states against the one the\n"
            "testbench instantiates; if the declaration was respelled, teach this\n"
            "script the new spelling rather than dropping the comparison."
        )
    return int(match.group(1))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rom-bytes", type=int, required=True)
    parser.add_argument("--ram-bytes", type=int, required=True)
    parser.add_argument("--placed-rom", type=int, required=True)
    parser.add_argument("--placed-ram", type=int, required=True)
    parser.add_argument("--sim-rom", type=int, required=True)
    parser.add_argument("--sim-ram", type=int, required=True)
    parser.add_argument("--tb", required=True, help="soc/compare/coremark_tb.v")
    parser.add_argument(
        "--core",
        action="append",
        default=[],
        metavar="NAME=LOG",
        help="a core's standalone yosys log, for its own block RAM count",
    )
    parser.add_argument(
        "--part-blocks",
        type=int,
        default=32,
        help="SB_RAM40_4K on the part the harness places on (hx8k: 32)",
    )
    parser.add_argument("--part", default="hx8k")
    args = parser.parse_args()

    for label, value in (
        ("--rom-bytes", args.rom_bytes),
        ("--ram-bytes", args.ram_bytes),
    ):
        if value <= 0:
            sys.exit(f"{label} is {value}; an empty image is not a measurement.")

    rom_words = read_tb_param(args.tb, "ROM_WORDS")
    ram_words = read_tb_param(args.tb, "RAM_WORDS")
    if rom_words * 4 != args.sim_rom or ram_words * 4 != args.sim_ram:
        sys.exit(
            f"the simulated geometry does not agree with itself: the linker script\n"
            f"gives {args.sim_rom} bytes of rom and {args.sim_ram} of ram, "
            f"{args.tb} instantiates\n"
            f"{rom_words * 4} and {ram_words * 4}. The image and the memories that "
            "hold it would be\ndescribing different machines, and both would still run."
        )
    if args.rom_bytes > args.sim_rom or args.ram_bytes > args.sim_ram:
        sys.exit(
            f"the image does not fit the simulated geometry: {args.rom_bytes} bytes\n"
            f"of rom against {args.sim_rom} and {args.ram_bytes} of ram against "
            f"{args.sim_ram}.\n"
            "A `.bss` past the end of RAM runs without a word from anyone."
        )

    rom_blocks = blocks_for(args.rom_bytes)
    ram_blocks = blocks_for(args.ram_bytes)
    print(
        f"the image needs {rom_blocks} + {ram_blocks} = {rom_blocks + ram_blocks} "
        f"SB_RAM40_4K, and {args.part} has {args.part_blocks} in total"
    )

    fits_placed = args.rom_bytes <= args.placed_rom and args.ram_bytes <= args.placed_ram
    for spec in args.core:
        name, core_blocks = read_core_blocks(spec)
        total = core_blocks + rom_blocks + ram_blocks
        verdict = "fits" if total <= args.part_blocks else "DOES NOT FIT"
        print(
            f"  {name:<10} {core_blocks:>2} of its own + {rom_blocks + ram_blocks} "
            f"for the image = {total:>3} blocks: {verdict}"
        )

    if fits_placed:
        print(
            "the image fits the placed geometry, so this run could be a placement\n"
            "rather than a simulation. Nothing below is distorted by memory size."
        )
    else:
        print(
            f"THE IMAGE DOES NOT FIT THE PLACED GEOMETRY "
            f"({args.placed_rom} rom, {args.placed_ram} ram),\n"
            "so the cycles below are simulated at a larger map than the clock they\n"
            "get multiplied by was measured at. Both cores run that same larger map,\n"
            "so the two cycle counts are comparable with each other and the absolute\n"
            "CoreMark figures are a projection, not a measured throughput."
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())

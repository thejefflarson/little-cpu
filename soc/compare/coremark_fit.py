#!/usr/bin/env python3
"""Report a CoreMark image against the memory the comparison harness can place.

THE ANSWER THIS PRINTS DECIDES WHETHER THE CYCLE COUNTS BELOW IT CARRY A CAVEAT
-- the same question soc/compare/dhry_fit.py already asks for Dhrystone. `make
compare-timing` places each core on an ice40 up5k, which has 30 `SB_RAM40_4K`
for the fetch window and the register files and four `SB_SPRAM256KA` -- 128 KB
in pairs -- for the data RAM. CoreMark's own image is roughly four times
Dhrystone's even at RV32IM with no compressed encodings, so when it outgrows
that the cycle counts have to be multiplied by a clock measured at the smaller
placed geometry, and when it does not, nothing below is distorted by memory
size. Either way this script says which, every run, next to the numbers.

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

# yosys `stat`, the same line shape soc/compare/placed_vs_synth.py reads for SB_LUT4.
BLOCKS = re.compile(r"^\s+(\d+)\s+SB_RAM40_4K\s*$", re.M)
# `localparam int ROM_WORDS = 4096;` in the testbench.
PARAM = r"^\s*localparam\s+int\s+{}\s*=\s*(\d+)\s*;"

# One SB_RAM40_4K is 4096 bits, and yosys builds a 32-bit word out of two of them 256
# words deep.
WORDS_PER_BLOCK_PAIR = 256
# One SB_SPRAM256KA is 16384 x 16, so a 32-bit word takes two of them.
WORDS_PER_SPRAM_PAIR = 16384

def blocks_for(byte_count):
    words = (byte_count + 3) // 4
    pairs = (words + WORDS_PER_BLOCK_PAIR - 1) // WORDS_PER_BLOCK_PAIR
    return pairs * 2

def spram_for(byte_count):
    words = (byte_count + 3) // 4
    pairs = (words + WORDS_PER_SPRAM_PAIR - 1) // WORDS_PER_SPRAM_PAIR
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
        default=30,
        help="SB_RAM40_4K on the part the harness places on (up5k: 30)",
    )
    parser.add_argument(
        "--part-spram",
        type=int,
        default=4,
        help="SB_SPRAM256KA on that part (up5k: 4). Zero means the data RAM has "
        "to come out of block RAM like everything else",
    )
    parser.add_argument("--part", default="up5k")
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
    # The data RAM goes to SPRAM where the part has it, which is what makes 64 KB of
    # RAM affordable on a part with 30 block RAMs; on a part with none it falls back to
    # block RAM and competes with the fetch window for the same 30.
    ram_spram = spram_for(args.ram_bytes) if args.part_spram else 0
    ram_blocks = 0 if ram_spram else blocks_for(args.ram_bytes)
    image_blocks = rom_blocks + ram_blocks
    print(
        f"the image needs {image_blocks} SB_RAM40_4K and {ram_spram} "
        f"SB_SPRAM256KA, and {args.part} has {args.part_blocks} and "
        f"{args.part_spram}"
    )

    fits_placed = args.rom_bytes <= args.placed_rom and args.ram_bytes <= args.placed_ram
    spram_fits = ram_spram <= args.part_spram
    for spec in args.core:
        name, core_blocks = read_core_blocks(spec)
        total = core_blocks + image_blocks
        fits = total <= args.part_blocks and spram_fits
        print(
            f"  {name:<10} {core_blocks:>2} of its own + {image_blocks} "
            f"for the image = {total:>3} blocks: "
            f"{'fits' if fits else 'DOES NOT FIT'}"
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

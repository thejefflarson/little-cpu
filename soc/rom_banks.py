#!/usr/bin/env python3
"""Split a flat ROM image into rtl/imemory.v's two interleaved banks.

rtl/imemory.v stores word W in `rom_even` at index W/2 when W is even and in
`rom_odd` at the same index when it is odd (ADR-0054). That is what removes the
duplicated ROM the SoC used to need for ADR-0003's dual-word fetch window, and
it means the bitstream's ROM image is two `$readmemh` files rather than one.

Two files rather than one plus a de-interleaving loop in RTL, because yosys does
not turn an `initial` block that copies between arrays into memory init: such a
design elaborates and then synthesises to an empty ROM.

Input is `objcopy -O verilog --verilog-data-width=4` output -- the same format
test/run_tests.sh feeds the cxxrtl runners, so the SoC and the simulators are
built from one image and cannot come to disagree about the program.

Both banks are written at full depth, zero-padded. A short file is not a
correctness problem for yosys (it initialises what it is given) but it IS one
for iverilog, which warns at run time, and a warning fails the build here.
"""

import argparse
import sys


def parse_verilog_hex(path):
    """word address -> 32-bit word, from objcopy's `-O verilog` format."""
    image = {}
    addr = 0
    with open(path) as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            if line.startswith("@"):
                addr = int(line[1:], 16)
                continue
            for word in line.split():
                image[addr] = int(word, 16)
                addr += 1
    return image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", help="objcopy -O verilog --verilog-data-width=4 output")
    parser.add_argument("even", help="output path for the even-word bank")
    parser.add_argument("odd", help="output path for the odd-word bank")
    parser.add_argument(
        "--rom-words",
        type=int,
        required=True,
        help="total 32-bit words of ROM; must match rtl/littlesoc.v's ROM_WORDS",
    )
    args = parser.parse_args()

    if args.rom_words % 2 != 0:
        sys.exit(f"--rom-words must be even, got {args.rom_words}")
    bank_words = args.rom_words // 2

    image = parse_verilog_hex(args.image)
    if not image:
        sys.exit(f"{args.image} contains no words; refusing to write an empty ROM")

    # A program that does not fit is a FINDING, not something to truncate. The
    # ROM ceiling is the part's block RAM and it is the whole reason this SoC
    # can be initialised from its bitstream at all (ADR-0054).
    top = max(image)
    if top >= args.rom_words:
        sys.exit(
            f"{args.image} reaches word {top} (byte 0x{top * 4:08x}), which is past "
            f"the {args.rom_words}-word ({args.rom_words * 4} byte) ROM. Shrink the "
            f"program or raise ROM_WORDS in rtl/littlesoc.v -- but read ADR-0054 "
            f"first: the ceiling is 30 block RAMs, four of which rtl/regfile.v uses."
        )

    banks = [[0] * bank_words, [0] * bank_words]
    for addr, word in image.items():
        banks[addr & 1][addr >> 1] = word

    for path, bank in ((args.even, banks[0]), (args.odd, banks[1])):
        with open(path, "w") as handle:
            for word in bank:
                handle.write(f"{word:08x}\n")

    print(
        f"{args.image}: {len(image)} words -> {args.even} + {args.odd} "
        f"({bank_words} words each, {args.rom_words * 4} byte ROM)"
    )


if __name__ == "__main__":
    main()

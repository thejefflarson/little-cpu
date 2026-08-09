#!/usr/bin/env python3
"""Write soc/compare/bench_vexriscv.v's single-bank ROM image.

soc/rom_banks.py writes rtl/imemory.v's two interleaved banks from the same
objcopy output; VexRiscv fetches one word at a time and wants one flat file.
Both images are the same program, and both are produced from the one `objcopy
-O verilog` file the Makefile builds once, so the two harnesses cannot come to
be running different code.

objcopy's format is parsed by importing soc/rom_banks.py rather than by reading
it again here. A second parser of the same format is a second thing that can
silently stop matching, and this one would be the copy nobody looks at.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# Otherwise the import drops a `soc/__pycache__` into a source directory, which
# is untracked build detritus nothing else here produces.
sys.dont_write_bytecode = True
from rom_banks import parse_verilog_hex  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", help="objcopy -O verilog --verilog-data-width=4 output")
    parser.add_argument("out", help="output path for the flat ROM image")
    parser.add_argument(
        "--rom-words",
        type=int,
        required=True,
        help="total 32-bit words of ROM; must match the harness's ROM_WORDS",
    )
    args = parser.parse_args()

    image = parse_verilog_hex(args.image)
    if not image:
        sys.exit(f"{args.image} contains no words; refusing to write an empty ROM")

    top = max(image)
    if top >= args.rom_words:
        sys.exit(
            f"{args.image} reaches word {top} (byte 0x{top * 4:08x}), which is past "
            f"the {args.rom_words}-word ({args.rom_words * 4} byte) ROM. The harness's "
            f"ROM is small so that both cores fit one hx8k; shrink the program rather "
            f"than the ROM."
        )

    # Full depth, zero-padded: iverilog warns at run time about a short
    # $readmemh, and a warning fails the build here.
    words = [0] * args.rom_words
    for addr, word in image.items():
        words[addr] = word

    with open(args.out, "w") as handle:
        for word in words:
            handle.write(f"{word:08x}\n")

    print(
        f"{args.image}: {len(image)} words -> {args.out} "
        f"({args.rom_words * 4} byte ROM)"
    )


if __name__ == "__main__":
    main()

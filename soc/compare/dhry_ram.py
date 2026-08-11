#!/usr/bin/env python3
"""Write the data RAM image both cores in soc/compare/ start the benchmark with.

soc/compare/bench_vexriscv.v gives its core no data path to the ROM -- a load
from a ROM address reads back zero there -- so a startup that copied `.data` out
of ROM would move the real bytes on one core and zeros on the other. The
initialised data is therefore poked into both cores' RAM before the run instead,
which is what test/run_tests.sh does for the assembly suite and what
soc/compare/dhry_start.S says in place of a copy loop.

Input is the raw bytes of `.data` (`objcopy -O binary -j .data`) rather than a
verilog hex file, because what has to be placed is a RUN-time address and
objcopy's verilog output carries load addresses. The one number that positions
it is `--offset-words`, computed from the linked `__data_start` by the caller.

The image is written at full depth and zero-padded: iverilog warns at run time
about a short $readmemh and a warning fails the build here, and a RAM that is X
where nothing was written is not a model of one.
"""

import argparse
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("data", help="objcopy -O binary -j .data output")
    parser.add_argument("out", help="output path for the flat RAM image")
    parser.add_argument("--ram-words", type=int, required=True)
    parser.add_argument(
        "--offset-words",
        type=int,
        required=True,
        help="(__data_start - the RAM's base) / 4",
    )
    args = parser.parse_args()

    if args.ram_words <= 0:
        sys.exit(f"--ram-words is {args.ram_words}; there is no RAM to write.")
    if args.offset_words < 0:
        sys.exit(
            f"--offset-words is {args.offset_words}: the data section starts below "
            "the RAM's base, so it is not in the RAM at all."
        )

    with open(args.data, "rb") as handle:
        blob = handle.read()
    if not blob:
        sys.exit(f"{args.data} is empty; refusing to write an image with no data.")
    if len(blob) % 4 != 0:
        sys.exit(
            f"{args.data} is {len(blob)} bytes, which is not a whole number of "
            "words. The linker script aligns .data to 4; this input did not come "
            "from it."
        )

    words = [0] * args.ram_words
    for index in range(len(blob) // 4):
        target = args.offset_words + index
        if target >= args.ram_words:
            sys.exit(
                f"{args.data} reaches word {target} of a {args.ram_words}-word RAM. "
                "The data does not fit the memory it is meant to initialise."
            )
        words[target] = int.from_bytes(blob[4 * index : 4 * index + 4], "little")

    with open(args.out, "w") as handle:
        for word in words:
            handle.write(f"{word:08x}\n")

    print(
        f"{args.data}: {len(blob)} bytes at word {args.offset_words} -> {args.out} "
        f"({args.ram_words * 4} byte RAM)"
    )


if __name__ == "__main__":
    main()

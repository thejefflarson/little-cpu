#!/usr/bin/env python3
"""Refuses a test/PROBES_EXPECTED whose comment header was sorted into order, deleted,
or cut below its tripwires. make probe-gates strips `#` lines before it compares, so it
cannot see any of those.

Usage: probes_header_test.py [manifest]    # defaults to this repo's test/PROBES_EXPECTED

A whole-file `LC_ALL=C sort` does not interleave the header with the labels -- `#` sorts
below every label -- it reorders the header in place. So the tell is a header in C order,
and the header is written so that its own lines are not.
"""

import pathlib
import sys

MIN_LINES = 2


def main(argv):
    default = pathlib.Path(__file__).resolve().parent / "PROBES_EXPECTED"
    path = pathlib.Path(argv[1]) if len(argv) > 1 else default
    if not path.is_file():
        print(f"error: {path} does not exist, so there is no header to grade.", file=sys.stderr)
        return 1
    header = []
    for line in path.read_bytes().split(b"\n"):
        if not line.startswith(b"#"):
            break
        header.append(line)
    if len(header) < MIN_LINES:
        print(f"error: {path} has {len(header)} header line(s), under {MIN_LINES}: the multiset\n"
              "       and never-regenerate tripwires are gone.", file=sys.stderr)
        return 1
    if header == sorted(header):
        print(f"error: {path}'s header is in LC_ALL=C order, so a whole-file sort would be\n"
              "       undetectable. If a sort was the edit, restore the header; if the lines\n"
              "       only happen to fall in that order, reorder them.", file=sys.stderr)
        return 1
    print(f"probes-header: {path.name} opens with {len(header)} comment lines, not in C order.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

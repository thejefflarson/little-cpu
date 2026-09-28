#!/usr/bin/env python3
"""Counts sky130_fd_sc_hd cell instantiations in a hardened gate-level netlist.

Refuses a netlist with zero instantiations of a `--require`d family -- the silent
case ADR-0213 names: nano/tt/src/config.json turns clock gating on, and nothing short
of reading the netlist's own text says whether the flow actually applied it.
"""

import argparse
import re
import sys

CELL_RE = re.compile(r"\bsky130_fd_sc_hd__([A-Za-z0-9]+)_(\d+)\s+\\?[\w$.\[\]]+\s*\(")


def census(text):
    counts = {}
    for name, _strength in CELL_RE.findall(text):
        counts[name] = counts.get(name, 0) + 1
    return counts


def main(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("netlist")
    parser.add_argument(
        "--require",
        action="append",
        default=[],
        help="a cell family (e.g. dlclkp) that must appear at least once; repeatable",
    )
    args = parser.parse_args(argv)

    with open(args.netlist) as f:
        text = f.read()

    counts = census(text)
    total = sum(counts.values())
    if total == 0:
        print(
            "error: no sky130_fd_sc_hd cell instantiations found in %s -- this does "
            "not read as a hardened netlist." % args.netlist,
            file=sys.stderr,
        )
        return 1

    for name in sorted(counts):
        print("%6d  %s" % (counts[name], name))
    print("%6d  TOTAL" % total)

    missing = [name for name in args.require if counts.get(name, 0) == 0]
    if missing:
        print(
            "error: expected at least one instance of each of %s, found zero of: %s"
            % (args.require, missing),
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

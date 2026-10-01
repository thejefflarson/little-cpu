#!/usr/bin/env python3
"""Counts sky130_fd_sc_hd cell instantiations in a hardened gate-level netlist.

Refuses a file with none, which is RTL or an empty flow output rather than a netlist, so a
gate-level run never passes by simulating the wrong thing. Refuses a netlist with zero
instances of a `--require`d family too: nano_gated_reg instantiates sky130's clock gate only
under the flow's SCL define, and a flow that stopped defining it would otherwise pass an
ungated netlist.
"""

import argparse
import re
import sys

CELL_RE = re.compile(r"\bsky130_fd_sc_hd__([A-Za-z0-9_]+?)_(\d+)\b")


def census(text):
    counts = {}
    types = set()
    for name, strength in CELL_RE.findall(text):
        counts[name] = counts.get(name, 0) + 1
        types.add("sky130_fd_sc_hd__%s_%s" % (name, strength))
    return counts, types


def main(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("netlist")
    parser.add_argument(
        "--require",
        action="append",
        default=[],
        help="a cell family (e.g. dlclkp) that must appear at least once; repeatable",
    )
    parser.add_argument(
        "--includes",
        help="write one `include per cell type here, so every model is read in one "
        "compilation unit and its include guards hold across drive strengths",
    )
    args = parser.parse_args(argv)

    with open(args.netlist) as f:
        text = f.read()

    counts, types = census(text)
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

    if args.includes:
        with open(args.includes, "w") as f:
            f.writelines('`include "%s.v"\n' % t for t in sorted(types))

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

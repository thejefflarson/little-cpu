#!/usr/bin/env python3
"""Summarise an `icetime -r` report: the critical path's LOGIC/ROUTING split.

THE SPLIT IS THE FINDING, not the frequency (ADR-0054). The only real timing
number this project had before the SoC placed was `rtl/regfile.v` on its own --
2.04 ns logic + 12.78 ns routing, 86% routing -- and what made it useful was the
breakdown, because it said the cost was a distributed mux's wires rather than
its gates. A whole-SoC figure without the same breakdown would be a number with
nowhere to go.

icetime prints one line per hop. A `LogicCell40` hop is a LUT or a flip-flop
evaluating; every other cell on the path -- `LocalMux`, `InMux`, `Odrv4`, the
`Span4Mux` family, `CascadeMux`, `ICE_CARRY_IN_MUX` -- is interconnect. That is
the split, and it is derived from the report rather than estimated.

Also prints the path's start point, its end point and its logic depth, because
"the critical path is named, not just timed" is the difference between a number
you can act on and one you can only quote.
"""

import argparse
import collections
import re
import sys

# A LUT or flip-flop evaluating. Everything else icetime lists is interconnect.
LOGIC_CELLS = {"LogicCell40"}

# `<indent><instance> (<CellType>) [something]: <t> ns`
HOP = re.compile(r"^\s+(\S+) \((\w+)\)[^:]*: ([0-9.]+) ns")
# The named nets between hops: `<cumulative> ns <net> (<name>)`
NET = re.compile(r"^\s+[0-9.]+ ns \S+ \((.+)\)\s*$")
# The last line of the path listing: `              lcout -> <endpoint>`
ENDPOINT = re.compile(r"^\s+\S+ -> (\S+)\s*$")
TOTAL = re.compile(r"^Total path delay: ([0-9.]+) ns")
LEVELS = re.compile(r"^Total number of logic levels: (\d+)")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", help="output of `icetime -t -r <report> <asc>`")
    args = parser.parse_args()

    logic = 0.0
    routing = 0.0
    per_cell = collections.defaultdict(float)
    counts = collections.Counter()
    # The first named net on the path is the readable name for where it starts:
    # icetime names the placed instance, yosys names the net, and the net is the
    # one a person can find in rtl/.
    start_point = None
    end_point = None
    seen_a_hop = False
    reported_total = None
    levels = None

    for line in open(args.report):
        hop = HOP.match(line)
        if hop:
            kind, delay = hop.group(2), float(hop.group(3))
            counts[kind] += 1
            per_cell[kind] += delay
            if kind in LOGIC_CELLS:
                logic += delay
            else:
                routing += delay
            seen_a_hop = True
            continue
        net = NET.match(line)
        if net and seen_a_hop and start_point is None:
            start_point = net.group(1)
            continue
        end = ENDPOINT.match(line)
        if end:
            end_point = end.group(1)
            continue
        total = TOTAL.match(line)
        if total:
            reported_total = float(total.group(1))
            continue
        lvl = LEVELS.match(line)
        if lvl:
            levels = int(lvl.group(1))

    walked = logic + routing
    if reported_total is None or start_point is None:
        sys.exit(
            f"{args.report} does not look like an `icetime -r` report: no critical "
            f"path was found in it. That is a failed measurement, not a fast design."
        )

    # The walk must account for the delay icetime reports. A regex that silently
    # stopped matching a hop class would otherwise shift the split -- toward
    # logic or toward routing depending on which class it dropped -- and the
    # summary would still look like a summary.
    if abs(walked - reported_total) > 0.05:
        sys.exit(
            f"{args.report}: summed hops come to {walked:.2f} ns but icetime "
            f"reports {reported_total:.2f} ns. Some hop class is not being "
            f"counted; fix this script rather than trusting the split."
        )

    print(f"critical path : {reported_total:.2f} ns  ({1000 / reported_total:.2f} MHz)")
    print(f"  logic       : {logic:6.2f} ns  {100 * logic / walked:4.1f}%")
    print(f"  routing     : {routing:6.2f} ns  {100 * routing / walked:4.1f}%")
    print(f"  logic levels: {levels}")
    print(f"  start       : {start_point}")
    print(f"  end         : {end_point}")
    print()
    print("  per cell class:")
    for kind in sorted(per_cell, key=lambda k: -per_cell[k]):
        tag = "logic  " if kind in LOGIC_CELLS else "routing"
        print(f"    {kind:18s} {tag}  x{counts[kind]:<4d} {per_cell[kind]:7.2f} ns")


if __name__ == "__main__":
    main()

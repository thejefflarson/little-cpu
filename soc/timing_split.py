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

THE DEPTH IS PRINTED AS TWO NUMBERS, because icetime's single one is not a cost.
A `carryin -> carryout` hop stays inside the carry chain and needs no
interconnect at all; every other LogicCell40 hop is entered through a LocalMux
and an InMux. On the SoC's own critical path that is 0.34 ns against 3.31 --
roughly tenfold -- so a design that trades a carry hop for a LUT level gets
shorter by icetime's count and slower in nanoseconds. Read the two apart before
deciding a path is deep (ADR-0058).

`--min-mhz` grades the result. The check lives HERE rather than in the Makefile
because this is the thing that already parses the report: a second parser --
which is what the first version of `make soc-timing` had, as a `python3 -c` -- is
a second thing that can silently stop matching, and it would be the one holding
the ratchet.
"""

import argparse
import collections
import re
import sys

# A LUT or flip-flop evaluating. Everything else icetime lists is interconnect.
LOGIC_CELLS = {"LogicCell40"}

# A hop that stays inside the carry chain: `carryin -> carryout`. The chain's
# entry hop (`in2 -> carryout`) is driven by an ordinary LUT input and counts
# with the LUT levels, which is what makes this the interconnect-free set.
CARRY_HOP = "carryin -> carryout"
# The interconnect that belongs to the carry chain rather than to a LUT level.
CARRY_ROUTING = {"ICE_CARRY_IN_MUX"}

# `<indent><instance> (<CellType>)<description>: <t> ns`
HOP = re.compile(r"^\s+(\S+) \((\w+)\)([^:]*): ([0-9.]+) ns")
# The named nets between hops: `<cumulative> ns <net> (<name>)`
NET = re.compile(r"^\s+[0-9.]+ ns \S+ \((.+)\)\s*$")
# The last line of the path listing: `              lcout -> <endpoint>`
ENDPOINT = re.compile(r"^\s+\S+ -> (\S+)\s*$")
TOTAL = re.compile(r"^Total path delay: ([0-9.]+) ns")
LEVELS = re.compile(r"^Total number of logic levels: (\d+)")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", help="output of `icetime -t -r <report> <asc>`")
    parser.add_argument(
        "--min-mhz",
        type=float,
        help="fail if the critical path is slower than this (the regression "
        "ratchet; see SOC_MIN_MHZ in the Makefile for why it sits below the "
        "measurement rather than at ADR-0038's declared 12 MHz)",
    )
    args = parser.parse_args()

    logic = 0.0
    routing = 0.0
    carry_ns = 0.0
    carry_hops = 0
    lut_hops = 0
    per_cell = collections.defaultdict(float)
    counts = collections.Counter()
    # The first named net on the path is the readable name for where it starts:
    # icetime names the placed instance, yosys names the net, and the net is the
    # one a person can find in rtl/.
    start_point = None
    end_point = None
    reported_total = None
    levels = None

    for line in open(args.report):
        hop = HOP.match(line)
        if hop:
            kind, what, delay = hop.group(2), hop.group(3).strip(), float(hop.group(4))
            counts[kind] += 1
            per_cell[kind] += delay
            if kind in LOGIC_CELLS:
                logic += delay
                if what == CARRY_HOP:
                    carry_hops += 1
                    carry_ns += delay
                else:
                    lut_hops += 1
            else:
                routing += delay
                if kind in CARRY_ROUTING:
                    carry_ns += delay
            continue
        net = NET.match(line)
        if net and start_point is None:
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
    if reported_total is None:
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
    print(f"  logic levels: {levels} by icetime; {lut_hops} LUT/setup + {carry_hops} carry by hop")
    per_lut = (walked - carry_ns) / lut_hops if lut_hops else 0.0
    per_carry = carry_ns / carry_hops if carry_hops else 0.0
    print(f"  per level   : {per_lut:.2f} ns per LUT level, {per_carry:.2f} ns per carry hop")
    print(f"  start       : {start_point}")
    print(f"  end         : {end_point}")
    print()
    print("  per cell class:")
    for kind in sorted(per_cell, key=lambda k: -per_cell[k]):
        tag = "logic  " if kind in LOGIC_CELLS else "routing"
        print(f"    {kind:18s} {tag}  x{counts[kind]:<4d} {per_cell[kind]:7.2f} ns")

    if args.min_mhz is not None:
        mhz = 1000 / reported_total
        if mhz < args.min_mhz:
            sys.exit(
                f"\n*** {mhz:.2f} MHz is under the {args.min_mhz:.2f} MHz floor.\n"
                f"*** This is a ratchet (ADR-0054), not a suggestion. Find what\n"
                f"*** lengthened the fetch -> decode -> next-PC loop; raising\n"
                f"*** SOC_MIN_MHZ in the Makefile needs a reason in the commit."
            )
        print(f"\nRATCHET: {mhz:.2f} MHz against a {args.min_mhz:.2f} MHz floor -- OK")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Read nextpnr-ecp5's JSON report and its Trellis text configuration: name the
corner that was placed, publish the frequency, and refuse every shape of
"nothing was measured".

THE ONE READER OF THAT REPORT, and that matters more here than it does for
`make soc-timing`. There the recipe deliberately ignores nextpnr's exit status,
because nextpnr grades its own default with its own estimator, and icetime is
the second reader that says what the placement really costs. There is no icetime
for ECP5. The engine that drove the placement is the engine that grades it and
nothing checks it, so the answer is not a second parser -- it is that every
number below is refused unless the report names the design's clock, the
configuration names the part that was asked for, and the design missed the
constraint it was placed against.

That last one is the load-bearing check. nextpnr places timing-driven and stops
working a path once it meets the constraint, so a frequency reported at or above
the target measures the TARGET. The Makefile hands it a pinned constant far
above anything this design can reach for exactly that reason, and this is what
says the constant is still doing its job.

"No constraint found" must never read as a pass: an empty `fmax` object, a
report naming some other clock, and a report that is not JSON at all are each
red here, because a silent zero is the shape of the five comparison defects this
repo has recorded.
"""

import argparse
import json
import sys

# nextpnr types every hop on a critical path. Everything that is not
# interconnect is charged to logic -- `clk-to-q` is a driver's output delay,
# `setup` the endpoint's requirement, `source` an asynchronous launch. A type
# outside this table is refused rather than bucketed, because a hop class
# quietly dropped into either side moves the split while the summary goes on
# looking like a summary.
LOGIC_HOPS = {"clk-to-q", "logic", "setup", "source"}
ROUTING_HOPS = {"routing"}

# The walked path must reproduce the frequency nextpnr published from it. Half a
# LUT hop on this fabric is worth more than this, so a dropped hop cannot hide
# inside the tolerance.
RECONCILE_MHZ = 0.05

# Printed on every run, beside the number rather than in the Makefile recipe, so
# that a figure quoted from here cannot arrive without the four things it is not.
REFUSALS = """WHAT THIS TARGET REFUSES TO CLAIM
  * This is nextpnr's own estimate from the model it routed with. There is no
    icetime for ECP5, so the engine that drove the placement is the engine that
    graded it, with no second reader behind it.
  * The constraint above is an input to the placer and therefore part of the
    measurement. It travels with the number or the number means nothing.
  * There is no bitstream and no board. The Trellis configuration says the
    placement is expressible on the part; nothing here was packed, programmed
    or run.
  * One placement is a sample, not a measurement.
  * AN ECP5 NUMBER IS NEVER MERGED WITH OR SUBTRACTED FROM AN UP5K ONE.
    Different part, different fabric, different estimator. `make fit` and
    `make soc-timing` are already two instruments over two designs; this is a
    third and the rule binds harder, not less."""


def load_report(path):
    try:
        with open(path) as handle:
            return json.load(handle)
    except FileNotFoundError:
        sys.exit(
            f"*** make ecp5-timing: {path} does not exist, so NOTHING was\n"
            "*** measured. That is a failed run, not a fast design."
        )
    except json.JSONDecodeError as exc:
        sys.exit(
            f"*** make ecp5-timing: {path} is not JSON ({exc}). nextpnr writes\n"
            "*** this file last, so a truncated one means it died mid-report."
        )


def clock_entry(fmax, clock):
    """The report's fmax entry for the design's clock, or exit saying why not.

    nextpnr names the net rather than the port: a promoted global reads
    `$glbnet$clk$TRELLIS_IO_IN`. Matching on the `$`-separated components picks
    the port out of that without accepting a net that merely contains the
    letters.
    """
    named = [name for name in fmax if clock in name.split("$")]
    if not named:
        sys.exit(
            f"*** make ecp5-timing: no clock named '{clock}' in the report's\n"
            f"*** fmax table, which lists {sorted(fmax) or 'nothing at all'}.\n"
            "*** A report that constrains no clock has measured nothing, and\n"
            "*** reading that as a pass is how a gate goes quiet."
        )
    if len(named) > 1:
        sys.exit(
            f"*** make ecp5-timing: {len(named)} clocks in the report name\n"
            f"*** '{clock}': {sorted(named)}. Which one the frequency below\n"
            "*** describes is then a guess, so this refuses to guess."
        )
    return named[0], fmax[named[0]]


def walk(path):
    """Split one critical path into logic and interconnect."""
    logic = routing = 0.0
    levels = 0
    for hop in path:
        kind = hop["type"]
        delay = float(hop["delay"])
        if kind in ROUTING_HOPS:
            routing += delay
        elif kind in LOGIC_HOPS:
            logic += delay
            if kind == "logic":
                levels += 1
        else:
            sys.exit(
                f"*** make ecp5-timing: hop type '{kind}' is not one this\n"
                "*** script classifies. Charging it to the wrong side would\n"
                "*** move the split silently; fix soc/ecp5_report.py rather\n"
                "*** than trusting the summary."
            )
    return logic, routing, levels


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", help="nextpnr-ecp5 --report output, e.g. ecp5.report.json")
    parser.add_argument("config", help="nextpnr-ecp5 --textcfg output, e.g. ecp5.config")
    parser.add_argument("--clock", required=True, help="the design's clock port, e.g. clk")
    parser.add_argument(
        "--part",
        required=True,
        help="the part the recipe declared, in Trellis's spelling: the corner "
        "has to be graded off the configuration rather than trusted, because "
        "nextpnr's log never names it",
    )
    parser.add_argument(
        "--constraint-mhz",
        type=float,
        required=True,
        help="ECP5_TARGET_MHZ: the constant handed to --freq. Graded against "
        "what the report says it was placed at, so the two cannot drift.",
    )
    args = parser.parse_args()

    # The textcfg first: it is the cheapest thing here and it answers the
    # question every number below depends on -- which part was this placed for.
    try:
        with open(args.config) as handle:
            head = handle.read(4096)
    except FileNotFoundError:
        head = ""
    if not head.strip():
        sys.exit(
            f"*** make ecp5-timing: {args.config} is empty or missing, so the\n"
            "*** placement was never expressed in the part's configuration.\n"
            "*** Nothing below would describe a placeable design."
        )
    if args.part not in head:
        sys.exit(
            f"*** make ecp5-timing: {args.config} does not name {args.part}.\n"
            "*** The device, package and speed grade are declared in the\n"
            "*** Makefile; a run that placed some other part -- nextpnr's own\n"
            "*** defaults, say -- reports a corner nobody chose."
        )

    report = load_report(args.report)
    fmax = report.get("fmax")
    if not isinstance(fmax, dict) or not fmax:
        sys.exit(
            f"*** make ecp5-timing: {args.report} carries no fmax table. That\n"
            "*** is a run that constrained nothing, not a design with no\n"
            "*** critical path."
        )
    utilisation = report.get("utilization")
    if not isinstance(utilisation, dict) or not utilisation:
        sys.exit(
            f"*** make ecp5-timing: {args.report} carries no utilisation table,\n"
            "*** so the mapping half of this measurement is missing."
        )

    net, entry = clock_entry(fmax, args.clock)
    if "achieved" not in entry or "constraint" not in entry:
        sys.exit(
            f"*** make ecp5-timing: the fmax entry for {net} carries "
            f"{sorted(entry)},\n"
            "*** not an achieved frequency and the constraint it was placed\n"
            "*** against. nextpnr has changed the shape of its report; fix\n"
            "*** soc/ecp5_report.py rather than reading what is left."
        )
    achieved = float(entry["achieved"])
    placed_at = float(entry["constraint"])

    if abs(placed_at - args.constraint_mhz) > 1e-6:
        sys.exit(
            f"*** make ecp5-timing: the report was placed against a "
            f"{placed_at:.2f} MHz\n"
            f"*** constraint, but ECP5_TARGET_MHZ declares {args.constraint_mhz:.2f}.\n"
            "*** The constraint is an input to the placer, so two spellings of\n"
            "*** it are two different measurements."
        )
    if achieved >= placed_at:
        sys.exit(
            f"*** make ecp5-timing: {achieved:.2f} MHz met the "
            f"{placed_at:.2f} MHz constraint.\n"
            "*** nextpnr stops working a path once it meets the target, so this\n"
            "*** number measures the CONSTRAINT and not the design. Raise\n"
            "*** ECP5_TARGET_MHZ clear of the design again before quoting it."
        )

    critical = [
        p for p in report.get("critical_paths", [])
        if p.get("from") == f"posedge {net}" and p.get("to") == f"posedge {net}"
    ]
    if len(critical) != 1:
        sys.exit(
            f"*** make ecp5-timing: {len(critical)} critical paths run from "
            f"{net} back\n"
            "*** to itself. The frequency above is then a number with no path\n"
            "*** under it, which is not a measurement."
        )
    hops = critical[0]["path"]
    logic, routing, levels = walk(hops)
    walked = logic + routing
    if walked <= 0 or abs(1000.0 / walked - achieved) > RECONCILE_MHZ:
        sys.exit(
            f"*** make ecp5-timing: the reported path sums to {walked:.2f} ns "
            f"({1000.0 / walked if walked else 0:.2f} MHz)\n"
            f"*** but nextpnr publishes {achieved:.2f} MHz. Some hop is not\n"
            "*** being counted; fix soc/ecp5_report.py rather than trusting the\n"
            "*** split."
        )

    print(f"part          : {args.part}")
    print(f"clock         : {net}  (design port '{args.clock}')")
    print(f"Fmax          : {achieved:.2f} MHz  ({walked:.2f} ns)")
    print(f"  constraint  : {placed_at:.2f} MHz -- an INPUT to the placer, not a threshold")
    print(f"  logic       : {logic:6.2f} ns  {100 * logic / walked:4.1f}%")
    print(f"  routing     : {routing:6.2f} ns  {100 * routing / walked:4.1f}%")
    print(f"  logic hops  : {levels} of {len(hops)} hops on the path")
    print(f"  start       : {hops[0]['from']['cell']}")
    print(f"  end         : {hops[-1]['to']['cell']}")
    print()
    print("  utilisation (published, NOT graded -- the censuses in the recipe are")
    print("  what grade the mapping, off yosys's own cell table):")
    for cell in sorted(utilisation):
        used = utilisation[cell]["used"]
        if not used:
            continue
        available = utilisation[cell]["available"]
        share = 100.0 * used / available if available else 0.0
        print(f"    {cell:16s} {used:6d} / {available:6d}  {share:5.1f}%")

    print()
    print(REFUSALS)


if __name__ == "__main__":
    main()

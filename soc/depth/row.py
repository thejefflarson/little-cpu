#!/usr/bin/env python3
"""One CSV row per placement, for soc/depth/sweep.sh and soc/baseline_sweep.sh.

The walk is soc/timing_split.py's, imported rather than repeated: that file
already reconciles its summed hops against the delay icetime reports, and a
second copy of the walk here would be a second thing that can stop matching. The
ECP5 walk is soc/ecp5_report.py's for the same reason, imported the same way, so
a row and `make ecp5-timing` cannot come to two answers about one placement.

`--header` prints the column names, so the sweep does not carry a second copy of
the order this writes them in. A header and a row that disagree are a table that
reads correctly and means something else.

FOUR COLUMNS ARE ICETIME'S AND THERE IS NO ICETIME FOR ECP5, so an ECP5 row
writes `NA` in them rather than a number that would line up under an up5k one and
mean something else. nextpnr does publish its own logic/routing split, but it is
the estimator that drove the placement rather than a second reader of it, and its
`logic` hop count is not icetime's LUT level count -- there are no carry hops in
that report at all. Filling those columns from it would put two instruments in
one column, which is the one thing the three instruments here may never do. The
ECP5 split stays readable in the per-seed `ecp5.report.json` the sweep keeps.

The start and end points travel in the row on purpose. The whole spike is about
where the fetch loop begins and ends, so a variant whose critical path did not
move is a variant that measured nothing, and the row has to be able to say so.
"""

import os
import sys

# No `soc/__pycache__` for the sake of one import: nothing else here is a
# package, and a build artifact in a source directory is one more thing that has
# to be told not to be committed.
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import ecp5_report  # noqa: E402
import timing_split  # noqa: E402

COLUMNS = ["part", "variant", "seed", "ns", "mhz", "lut_levels", "carry_hops",
           "logic_ns", "routing_ns", "lc", "start", "end"]

# The four columns an ECP5 row cannot fill, and the spelling soc/baseline_summary.py
# recognises. A literal rather than an empty field: a blank column reads as a bug
# in whatever wrote the row, and this is a statement that no such number exists.
NA = "NA"

USAGE = ("usage: row.py --header\n"
         "       row.py <icetime-report> <part> <variant> <seed> <lc>\n"
         "       row.py --ecp5 <report.json> <config> <trellis-part> <clock> "
         "<constraint-mhz> <variant> <seed>")


def ecp5_row(argv):
    """One ECP5 placement, through soc/ecp5_report.py's own refusals.

    The cell count is taken from the report rather than passed in: nextpnr's
    utilisation table is already the authority the summary reads, and a count
    handed in on the command line is one a caller can get wrong silently.
    """
    report, config, part, clock, constraint, variant, seed = argv
    s = ecp5_report.summarise(report, config, clock, part, float(constraint))
    comb = s["utilisation"].get("TRELLIS_COMB", {}).get("used")
    if not comb:
        sys.exit(
            "*** row.py: the report's utilisation table has no TRELLIS_COMB\n"
            "*** count, so the row would carry no placed cell count at all."
        )
    return ["ecp5", variant, seed, f"{s['walked']:.2f}", f"{s['achieved']:.2f}",
            NA, NA, NA, NA, comb,
            s["hops"][0]["from"]["cell"], s["hops"][-1]["to"]["cell"]]


def main():
    if len(sys.argv) == 2 and sys.argv[1] == "--header":
        print(",".join(COLUMNS))
        return
    if len(sys.argv) > 1 and sys.argv[1] == "--ecp5":
        if len(sys.argv) != 9:
            sys.exit(USAGE)
        values = ecp5_row(sys.argv[2:])
    elif len(sys.argv) == 6:
        report, part, variant, seed, lc = sys.argv[1:]
        s = timing_split.summarise(report)
        values = [part, variant, seed, f"{s['total']:.2f}", f"{1000 / s['total']:.2f}",
                  s["lut_hops"], s["carry_hops"], f"{s['logic']:.2f}",
                  f"{s['routing']:.2f}", lc, s["start"], s["end"]]
    else:
        sys.exit(USAGE)
    print(",".join(str(v) for v in values))


if __name__ == "__main__":
    main()

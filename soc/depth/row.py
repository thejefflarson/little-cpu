#!/usr/bin/env python3
"""One CSV row per placement, for soc/depth/sweep.sh.

The walk is soc/timing_split.py's, imported rather than repeated: that file
already reconciles its summed hops against the delay icetime reports, and a
second copy of the walk here would be a second thing that can stop matching.

`--header` prints the column names, so the sweep does not carry a second copy of
the order this writes them in. A header and a row that disagree are a table that
reads correctly and means something else.

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
import timing_split  # noqa: E402

COLUMNS = ["part", "variant", "seed", "ns", "mhz", "lut_levels", "carry_hops",
           "logic_ns", "routing_ns", "lc", "start", "end"]


def main():
    if len(sys.argv) == 2 and sys.argv[1] == "--header":
        print(",".join(COLUMNS))
        return
    if len(sys.argv) != 6:
        sys.exit("usage: row.py --header | row.py <report> <part> <variant> <seed> <lc>")
    report, part, variant, seed, lc = sys.argv[1:]
    s = timing_split.summarise(report)
    values = [part, variant, seed, f"{s['total']:.2f}", f"{1000 / s['total']:.2f}",
              s["lut_hops"], s["carry_hops"], f"{s['logic']:.2f}", f"{s['routing']:.2f}",
              lc, s["start"], s["end"]]
    print(",".join(str(v) for v in values))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Print one sweep's worst, median, best and spread, from the same arithmetic the
stamped product artifact uses.

soc/compare/product_write.py already owns `clock_stats`, so this imports it rather
than restating it: a second formula for the median is a second thing that can stop
agreeing with the stamp a pull request quotes beside it.

Reports, grades nothing. On up5k the pass/fail verdict is soc/compare/step_gate.py's
and this column is only there to say how wide the sample was; on ECP5 the frequency
publishes with no ratchet, because soc/bands.py has no band for that part.

  spread.py --part ecp5 --core littlecpu 28.93 29.41 30.02
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from product_write import clock_stats  # noqa: E402

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--part", required=True)
    parser.add_argument("--core", required=True)
    parser.add_argument("ns", nargs="+", type=float, help="one critical path per seed")
    args = parser.parse_args()

    if any(value <= 0 for value in args.ns):
        sys.exit(
            f"*** {args.core} on {args.part}: a critical path of zero or less is not "
            "a placement. Something upstream reported nothing and it was read as a "
            "number."
        )

    stats = clock_stats(args.ns)
    print(
        f"  {args.core} on {args.part}, n={stats['n']}: "
        f"worst {stats['worst_mhz']:.2f} MHz / "
        f"median {stats['median_mhz']:.2f} / "
        f"best {stats['best_mhz']:.2f}, spread {stats['spread_pct']:.2f}%"
    )

if __name__ == "__main__":
    main()

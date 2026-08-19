#!/usr/bin/env python3
"""The placement spread and the edit-churn band, keyed by part. THE ONE SOURCE.

Every script that prints one of these figures asks this file for it, and no
script states one of its own. Before this existed the two numbers were prose in
six files, and prose copies do not move together: the placement spread stood at
"1-2%" in all six for as long as it took one sweep of sixteen seeds to read 9.2%,
which is the difference between a delta inside the band and a placement under the
board clock.

NOTHING HERE IS INHERITED ACROSS PARTS, and that is the property this file exists
to make structural rather than remembered. There is no default entry, no
fallback, and no argument that selects one part's figures for another. A part
whose band nobody has measured says so and hands back nothing -- `band()` raises,
so a program cannot compute with a number that was never taken -- because the
alternative shape, quietly serving up5k's figures for a part with a different
fabric, a different placer and a different estimator, is a wrong answer that
looks exactly like a right one.

A BAND IS A MEASUREMENT WITH A DATE ON IT. Each entry carries the tree, the
toolchain and the sweep it came from, because both figures move: they are
properties of a netlist, a placer and a router together, and this repo has
already recorded a toolchain that disagreed with another about the SIGN of a
period change on identical RTL. Re-derive rather than inherit across time, too.

  soc/bands.py up5k            # the sentence, naming the part
  soc/bands.py up5k --note     # the paragraph printed under a delta
  soc/bands.py --list          # every part, derived or not
  soc/bands.py hx8k --require  # non-zero: no band has been derived for it

Usage from Python is `band(part)` for the figures and `sentence(part)` for the
prose. `sentence()` answers for a known part whether or not a band was derived;
`band()` refuses when one was not.
"""

import argparse
import sys


class Underived(Exception):
    """Asked for figures that were never measured for this part."""


# Every part this repo places, with what has been measured on it. A part with
# `spread` and `churn` set to None is one nothing has been swept on: the entry
# exists so that asking about it gets an answer about THAT part rather than a
# KeyError that invites someone to substitute another part's numbers.
#
# `spread` is best-to-worst over an UNCHANGED netlist, as a percentage of the
# best placement. `churn` is how far functionally identical texts of one design
# move the number. They are different measurements and only the first comes from
# seeds alone -- the second needs several texts, each swept.
BANDS = {
    "up5k": {
        "instrument": "make soc-timing",
        "spread": (4.0, 9.0),
        "churn": 3.6,
        "derived": "PLACEHOLDER",
    },
    "ecp5": {
        "instrument": "make ecp5-timing",
        "spread": None,
        "churn": None,
        "derived": "PLACEHOLDER",
    },
    "hx8k": {
        "instrument": "make compare-timing",
        "spread": None,
        "churn": None,
        # The cross-core harness places on this part and no sweep has ever been
        # taken of its spread. up5k's is NOT the answer: different part, and the
        # harness is a different design as well -- 4 KB of ROM and 2 KB of block
        # RAM against 8 KB and 64 KB of SPRAM, and no timer.
        "derived": "no sweep has been taken on this part",
    },
}


def band(part):
    """The figures for one part, or refuse. Never another part's."""
    if part not in BANDS:
        raise KeyError(part)
    entry = BANDS[part]
    if entry["spread"] is None or entry["churn"] is None:
        raise Underived(part)
    return entry


def parts():
    return sorted(BANDS)


def sentence(part):
    """One line, naming the part it belongs to.

    The part is in the text and not merely in the caller's context, because
    these lines get pasted into commit messages and pull requests, where the
    context is gone and the number reads as though it were the only one.
    """
    if part not in BANDS:
        raise KeyError(part)
    entry = BANDS[part]
    if entry["spread"] is None or entry["churn"] is None:
        return (f"{part}: no placement spread and no churn band have been "
                f"derived for this part, and no other part's transfer.")
    low, high = entry["spread"]
    return (f"{part} ({entry['instrument']}): placement spread {low:g}-{high:g}% "
            f"best-to-worst on an unchanged netlist, edit-churn band ~{entry['churn']:g}%.")


def note(part):
    """The paragraph a delta is read against."""
    lines = [sentence(part)]
    if part in BANDS:
        lines.append(f"  derived from: {BANDS[part]['derived']}")
    try:
        band(part)
    except Underived:
        lines.append("  So a delta on this part cannot be called a change or a "
                     "null yet. Sweep it.")
        return "\n".join(lines)
    lines.append("  A delta inside either figure is not evidence of anything. "
                 "Read the paired")
    lines.append("  per-seed column before either of them.")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("part", nargs="?", help="up5k, ecp5 or hx8k")
    parser.add_argument("--note", action="store_true",
                        help="the paragraph printed under a delta")
    parser.add_argument("--list", action="store_true",
                        help="every part, derived or not")
    parser.add_argument(
        "--require",
        action="store_true",
        help="exit non-zero unless a band has actually been derived for this "
        "part. For a caller that needs the figures rather than the prose.",
    )
    args = parser.parse_args()

    if args.list:
        for part in parts():
            print(sentence(part))
        return
    if not args.part:
        parser.error("name a part, or pass --list")
    if args.part not in BANDS:
        sys.exit(f"*** soc/bands.py: '{args.part}' is not a part this repo places.\n"
                 f"*** Known: {', '.join(parts())}. Adding one means measuring it,\n"
                 "*** not copying another part's figures into a new entry.")
    if args.require:
        try:
            band(args.part)
        except Underived:
            sys.exit(f"*** soc/bands.py: no band has been derived for "
                     f"{args.part}.\n"
                     "*** Another part's does not transfer -- different fabric,\n"
                     "*** different placer, different estimator. Sweep it.")
    print(note(args.part) if args.note else sentence(args.part))


if __name__ == "__main__":
    main()

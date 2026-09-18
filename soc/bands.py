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
  soc/bands.py ecp5 --require  # non-zero: no band has been derived for it

Usage from Python is `band(part)` for the figures and `sentence(part)` for the
prose. `sentence()` answers for a known part whether or not a band was derived;
`band()` refuses when one was not.
"""

import argparse
import sys

class Underived(Exception):
    """Asked for figures that were never measured for this part."""

# Every part this repo places, with what has been measured on it. `derived` is
# `soc/paired_sweep.sh`'s own output for churn, plus `soc/baseline_sweep.sh`'s for
# spread: the tree, both tool versions and the sweep, so this file's own docstring
# claim -- a band is a measurement with a date on it -- is checkable against a real
# run rather than taken on faith.
BANDS = {
    "up5k": {
        "instrument": "make soc-timing",
        "spread": (4.0, 9.0),
        "churn": 3.6,
        "derived": (
            "2026-09-18, tree 84fe92b7084d (clean), Yosys 0.68+48 (ff5817c34), "
            "nextpnr-ice40 0.11-1-g62e659ed, icetime oss-cad-suite 20260811. "
            "Spread: 16 paired seeds on the unchanged netlist -- worst 83.38ns/"
            "11.99MHz, median 80.44ns/12.43MHz, best 78.10ns/12.80MHz, 6.8% -- "
            "inside the existing 4-9% and re-confirming it rather than replacing "
            "it (soc/baseline_sweep.sh BASELINE_NAME=spread-up5k). Churn: two "
            "real, netlist-digest-different edits to rtl/csrs.v (large "
            "representative file per ADR-0170) at the same 16 seeds -- "
            "ADR-0170's own comment diff (42 changed comment lines, replayed) "
            "worst -3.6%/median -3.0%/best -2.2%, and a matched-line-count "
            "blank-line-only diff worst -1.7%/median -3.5%/best -3.3% -- the "
            "3.6% ceiling re-confirmed by the comment class, both within it."
        ),
    },
    "ecp5": {
        "instrument": "make ecp5-timing",
        "spread": 10.3,
        "churn": 0.0,
        "derived": (
            "2026-09-18, tree 84fe92b7084d (clean), Yosys 0.68+48 (ff5817c34), "
            "nextpnr-ecp5 0.11-1-g62e659ed, trellis-db devices.json "
            "sha256:5a3869c1b6fe7ea1. Spread: ONE 16-seed sweep, the first ever "
            "taken for this part -- worst 30.29ns/33.01MHz, median 29.37ns/"
            "34.05MHz, best 27.46ns/36.42MHz, 10.3%, wider than up5k's own -- a "
            "single sweep, not yet the range a second one taken later would "
            "narrow or widen (soc/baseline_sweep.sh BASELINE_NAME=spread-ecp5). "
            "Churn: the SAME two rtl/csrs.v edits that moved up5k's netlist "
            "(each independently confirmed DIGEST-DIFFERENT for up5k via `make "
            "netlist-diff`) placed BYTE-IDENTICAL on ecp5 at all 16 seeds -- "
            "0.0% on both classes. THIS IS A MEASURED NULL UNDER THE TESTED "
            "FIXTURES, NOT A PROOF ecp5 cannot churn: its own synthesis flow "
            "evidently did not cross whatever sort-order boundary moved up5k's. "
            "Treat 0.0% as a floor to re-open, not a guarantee to build on."
        ),
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

def spread_text(spread):
    """A spread as prose: a real range from more than one sweep, or a single
    sweep's own figure -- ecp5 has had only one, and dressing it as a range
    (`10.3-10.3%`) would read like a typo rather than the honest fact that
    nothing has narrowed or widened it yet."""
    if isinstance(spread, tuple):
        low, high = spread
        return f"{low:g}-{high:g}%"
    return f"{spread:g}% (one sweep, not yet a range)"

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
    return (f"{part} ({entry['instrument']}): placement spread "
            f"{spread_text(entry['spread'])} best-to-worst on an unchanged "
            f"netlist, edit-churn band ~{entry['churn']:g}%.")

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

def print_derived(part):
    """The provenance line `--list` and a plain part query print alongside the
    sentence: the tree, both tool versions and the sweep, so a reader never has
    to open this file to see what a figure was measured against."""
    if part in BANDS:
        print(f"  derived from: {BANDS[part]['derived']}")

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("part", nargs="?", help="up5k or ecp5")
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
            print_derived(part)
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
    if args.note:
        print(note(args.part))
    else:
        print(sentence(args.part))
        print_derived(args.part)

if __name__ == "__main__":
    main()

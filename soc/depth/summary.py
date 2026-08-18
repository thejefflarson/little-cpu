#!/usr/bin/env python3
"""Turn soc/depth/sweep.sh's CSV into the distribution a churn band can be read
against.

WORST AND BEST, NEVER A MEAN. `make soc-timing` has a ~3.6% edit-churn band and a
4-9% placement spread (CLAUDE.md states both), so one placement is a sample. A
variant is ahead only if its distribution is ahead, and the two readings are
printed together because a change that wins on one end and loses on the other
has not been measured yet.

THE LEVEL COUNT IS THE PART THAT IS NOT NOISE. Nanoseconds move with placement;
LUT levels are a property of the mapped netlist and move only when the netlist
does. A variant whose levels are unchanged across every seed and whose
nanoseconds moved by a couple of percent has changed nothing.

Usage: summary.py <csv> [<csv> ...]
"""

import collections
import csv
import statistics
import sys

ORDER = ["base", "addr", "data", "both"]


def endpoint(name):
    """The readable part of an icetime endpoint name.

    Everything from the first `_SB_` on is yosys's mangling of the cells between
    this net and whatever it was named after, so it is ancestry rather than
    signal. Cutting it keeps the column readable and keeps anyone reading the
    tail as a place in rtl/.
    """
    head = name.split("_SB_")[0]
    return head + "..." if head != name else name


def span(values):
    """`22-23` where the seeds disagree, `22` where they do not."""
    low, high = min(values), max(values)
    return f"{low}-{high}" if low != high else str(low)


def load(path):
    """The placements in one sweep, grouped by variant, worst placement last.

    `make soc-rom` and `make compare-rom` print to the same stream the rows go
    to, so the file starts with whatever they had to say. The header line is
    where the measurement starts.
    """
    lines = open(path).read().splitlines()
    start = next((i for i, line in enumerate(lines) if line.startswith("part,")), None)
    if start is None:
        sys.exit(f"{path}: no CSV header in it. That is a failed sweep, not a fast design.")
    by = collections.defaultdict(list)
    for row in csv.DictReader(lines[start:]):
        if row.get("variant") in ORDER:
            by[row["variant"]].append(row)
    if not by:
        sys.exit(f"{path}: no placements in it. That is a failed sweep, not a fast design.")
    for group in by.values():
        group.sort(key=lambda r: float(r["ns"]))
    return by


def report(path):
    by = load(path)
    print(f"== {path} ==")
    print(f"{'variant':8} {'n':>2} {'worst ns':>9} {'MHz':>7} {'best ns':>8} "
          f"{'LUT levels':>12} {'carry':>7} {'LC':>6}  critical path")
    for variant in ORDER:
        group = by.get(variant)
        if not group:
            continue
        ns = [float(r["ns"]) for r in group]
        worst = group[-1]
        print(f"{variant:8} {len(ns):>2} {ns[-1]:>9.2f} {1000 / ns[-1]:>7.2f} {ns[0]:>8.2f} "
              f"{span([int(r['lut_levels']) for r in group]):>12} "
              f"{span([int(r['carry_hops']) for r in group]):>7} {group[0]['lc']:>6}  "
              f"{endpoint(worst['start'])} -> {endpoint(worst['end'])}")
    print()
    base = [float(r["ns"]) for r in by["base"]]
    for variant in ORDER[1:]:
        group = by.get(variant)
        if not group:
            continue
        ns = [float(r["ns"]) for r in group]
        worst = 100 * (ns[-1] - base[-1]) / base[-1]
        median = 100 * (statistics.median(ns) - statistics.median(base)) / statistics.median(base)
        print(f"  {variant} against base: worst {worst:+.1f}%, median {median:+.1f}%")
    print()


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: summary.py <csv> [<csv> ...]")
    for path in sys.argv[1:]:
        report(path)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Read one baseline sweep, or refuse to subtract two that were not measured the
same way.

WORST, MEDIAN AND SPREAD, NEVER A MEAN. `make soc-timing` has a ~3.6% edit-churn
band and a 1-2% placement spread, so one placement is a sample and the tail is
the part a requirement is read against.

THE REFUSAL IS THE POINT OF THIS SCRIPT. Two sweeps taken on different trees, or
by different toolchains, do not have a difference: the number subtracting them
produces is a property of what changed underneath, and it reads exactly like a
property of the design. That has happened here -- one pair of toolchains
disagreed about the SIGN of a period change on identical RTL. So a mismatch in
soc/baseline_sweep.sh's provenance block stops this script rather than printing a
warning above the delta. A warning that can be scrolled past is a gate that is
already off, and the thing it has to survive is a reader in a hurry.

`--allow-mismatch` prints the delta anyway, with the mismatch printed beside it
rather than instead of it, for the case where the difference is understood and
deliberate.

A sweep whose provenance block is missing, truncated or short of a field is
rejected instead of summarised: an unstamped file is a failed measurement, not a
comparable one, and the whole reason this format exists is that a bare column of
frequencies looks equally usable either way.

Usage: baseline_summary.py <csv> [<csv> [--allow-mismatch]]
"""

import argparse
import csv
import statistics
import sys

MARKER = "# baseline-sweep v1"
END = "# end-provenance"

# Every field soc/baseline_sweep.sh writes. A file short of one was written by
# something else, or by a version of that script that stamped less.
REQUIRED = ["date", "base", "dirty", "yosys", "nextpnr-ice40", "icetime",
            "prog", "rom_words", "seeds", "host", "reproduce"]

# The fields that have to agree before a delta means anything: the tree, the
# three tools, and the program and ROM size that decide what was placed. `date`,
# `seeds` and `host` are recorded but not compared -- two sweeps of different
# sizes on different days are exactly what a before-and-after is.
COMPARED = ["base", "yosys", "nextpnr-ice40", "icetime", "prog", "rom_words"]


def reject(path, why):
    sys.exit(f"*** {path}: {why}\n"
             f"*** That is a failed measurement, not a comparable one.")


def load(path):
    """One sweep's provenance block and its placements, or exit saying why not."""
    try:
        lines = open(path).read().splitlines()
    except OSError as err:
        sys.exit(f"*** {path}: {err.strerror}, so there is nothing to summarise.")

    lines = [line.strip() for line in lines]
    if not lines or lines[0] != MARKER:
        reject(path, f"no provenance block -- the first line is not '{MARKER}'")
    if END not in lines:
        reject(path, f"the provenance block is truncated: no '{END}' line")

    end = lines.index(END)
    provenance = {}
    for line in lines[1:end]:
        key, _, value = line.lstrip("# ").partition(":")
        provenance[key.strip()] = value.strip()
    missing = [key for key in REQUIRED if not provenance.get(key)]
    if missing:
        reject(path, "the provenance block is missing " + ", ".join(missing))

    rest = lines[end + 1:]
    if not rest or not rest[0].startswith("part,"):
        reject(path, "no CSV header after the provenance block")
    rows = [row for row in csv.DictReader(rest) if row.get("ns")]
    if not rows:
        reject(path, "no placements in it")
    for row in rows:
        try:
            row["ns"] = float(row["ns"])
        except (TypeError, ValueError):
            reject(path, f"a row whose ns column reads '{row['ns']}'")
    rows.sort(key=lambda r: r["ns"])
    return provenance, rows


def span(rows, column):
    """`23-25` where the placements disagree, `23` where they do not."""
    try:
        values = sorted(int(row[column]) for row in rows)
    except (TypeError, ValueError):
        return "?"
    return f"{values[0]}-{values[-1]}" if values[0] != values[-1] else str(values[0])


def stats(rows):
    ns = [row["ns"] for row in rows]
    worst, best = max(ns), min(ns)
    return {
        "n": len(ns),
        "worst": worst,
        "best": best,
        "median": statistics.median(ns),
        "spread": 100 * (worst - best) / best,
    }


def field(label, text):
    print(f"  {label:13s}: {text}")


def report(path, provenance, rows):
    s = stats(rows)
    print(f"== {path} ==")
    field("base", f"{provenance['base'][:12]} "
                  f"({'DIRTY TREE' if provenance['dirty'] != 'no' else 'clean'})")
    field("program", f"{provenance['prog']} into {provenance['rom_words']} ROM words")
    for tool in ("yosys", "nextpnr-ice40", "icetime"):
        field(tool, provenance[tool])
    field("measured", f"{provenance['date']} on {provenance['host']}")
    print()
    print(f"  {s['n']} placement{'' if s['n'] == 1 else 's'} of seeds: {provenance['seeds']}")
    for label in ("worst", "median", "best"):
        field(label, f"{s[label]:6.2f} ns  {1000 / s[label]:6.2f} MHz")
    field("spread", f"{s['spread']:.1f}% of the best placement")
    field("LUT levels", f"{span(rows, 'lut_levels')}   "
                        f"carry hops: {span(rows, 'carry_hops')}   LC: {span(rows, 'lc')}")
    field("worst path", f"{rows[-1]['start']} -> {rows[-1]['end']}")
    print()


def mismatches(first, second):
    """Every recorded reason these two sweeps are not one experiment.

    A dirty tree is one of them on its own: the base line then names a commit
    that is not what was placed, so two sweeps agreeing on it have agreed about
    nothing.
    """
    first_path, first_prov = first
    second_path, second_prov = second
    out = []
    for path, provenance in (first, second):
        if provenance["dirty"] != "no":
            out.append(f"{path} was measured on a tree with uncommitted changes, "
                       f"so its base names no tree")
    for key in COMPARED:
        if first_prov[key] != second_prov[key]:
            out.append(f"{key}:\n"
                       f"  {first_prov[key]}  ({first_path})\n"
                       f"  {second_prov[key]}  ({second_path})")
    return out


def emit(reasons, prefix):
    """One reason may be three lines long; the prefix belongs on all of them."""
    for reason in reasons:
        for line in reason.splitlines():
            print(f"{prefix}{line}")


def delta(first, second):
    before, after = stats(first), stats(second)
    print("== delta, second sweep against first ==")
    for label in ("worst", "median", "best"):
        change = 100 * (after[label] - before[label]) / before[label]
        print(f"  {label:8s}: {before[label]:6.2f} -> {after[label]:6.2f} ns  {change:+.1f}%")
    print("  A positive number is slower. Read both against the ~3.6% edit-churn")
    print("  band and the 1-2% placement spread before calling either a change.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="+", help="one or two baseline_sweep.sh CSVs")
    parser.add_argument(
        "--allow-mismatch",
        action="store_true",
        help="print the delta of two sweeps that were not measured the same way, "
        "with the mismatch beside it",
    )
    args = parser.parse_args()

    if len(args.csv) > 2:
        sys.exit("*** baseline_summary.py takes one sweep, or two to subtract.")

    loaded = [(path, *load(path)) for path in args.csv]
    for path, provenance, rows in loaded:
        report(path, provenance, rows)

    if len(loaded) != 2:
        return
    (first_path, first_prov, first_rows), (second_path, second_prov, second_rows) = loaded

    reasons = mismatches((first_path, first_prov), (second_path, second_prov))
    if reasons and not args.allow_mismatch:
        print("*** these two sweeps were not measured the same way, so the")
        print("*** difference between them is not a measurement of the design:")
        emit(reasons, "***   ")
        sys.exit("*** Re-measure both sides on one tree with one toolchain, or pass\n"
                 "*** --allow-mismatch to print the delta with the mismatch beside it.")
    if reasons:
        print("MISMATCH, and the delta below is printed anyway because")
        print("--allow-mismatch was passed:")
        emit(reasons, "  ")
        print()
    delta(first_rows, second_rows)


if __name__ == "__main__":
    main()

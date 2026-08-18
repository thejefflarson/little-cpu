#!/usr/bin/env python3
"""Read one baseline sweep, or refuse to subtract two that were not measured the
same way.

WORST, MEDIAN AND SPREAD, NEVER A MEAN. One placement is a sample and the tail is
the part a requirement is read against, so twelve to sixteen placements a side is
what a distribution costs to see. The two band figures a delta is read against
are soc/bands.py's, asked for by part; nothing here carries a figure of its own,
and neither does any other script. Prose copies do not move together -- the ones
this replaced were four times too narrow for as long as it took one sixteen-seed
sweep to say so, in six files at once, with nothing able to go red for it.

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

IT DOES NOT COVER THE PART, AND NOTHING DOES. Every other mismatch names two runs
of one experiment that a reader may have good reason to subtract anyway: a
different tree is a before-and-after, a different program is a workload, a
toolchain bump is the thing being measured. A different part is not a variant of
one experiment. up5k is placed by nextpnr-ice40 and read back by icetime; ECP5
has no icetime at all, so nextpnr's own estimator both places and grades. The
subtraction of those two numbers is not a large difference or an unfair one --
it is not a quantity, and printing a percentage for it would dress that up as
one. So the part is compared before anything else and stops the script whatever
flags were passed.

A sweep whose provenance block is missing, truncated or short of a field is
rejected instead of summarised: an unstamped file is a failed measurement, not a
comparable one, and the whole reason this format exists is that a bare column of
frequencies looks equally usable either way.

Usage: baseline_summary.py <csv> [<csv> [--allow-mismatch]]
"""

import argparse
import csv
import os
import statistics
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bands  # noqa: E402

MARKER = "# baseline-sweep v1"
END = "# end-provenance"

# The fields every sweep writes whatever it placed. A file short of one was
# written by something else, or by a version of soc/baseline_sweep.sh that
# stamped less.
COMMON_REQUIRED = ["date", "base", "dirty", "part", "yosys", "prog", "rom_words",
                   "seeds", "host", "reproduce"]

# What each part's measurement is additionally a property of. The tool sets are
# genuinely different rather than two spellings of one: up5k is placed by
# nextpnr-ice40 and then READ BACK by icetime, and there is no icetime for ECP5,
# so nextpnr-ecp5's own estimator is the placer and the grader both -- which is
# why the database it places against is stamped beside it, and why the constraint
# it was handed is an input to the number rather than a description of it.
#
# A part carrying another part's fields is rejected, not merged: that file's
# header describes an experiment nobody ran, and the likeliest way to produce one
# is to hand-edit a stamp until a comparison stops complaining.
PART_REQUIRED = {
    "up5k": ["nextpnr-ice40", "icetime"],
    "ecp5": ["nextpnr-ecp5", "trellis-db", "corner", "constraint_mhz"],
}

# The fields common to both parts that have to agree before a delta means
# anything: the tree, the shared tool, and the program and ROM size that decide
# what was placed. `date`, `seeds` and `host` are recorded but not compared --
# two sweeps of different sizes on different days are exactly what a
# before-and-after is.
COMMON_COMPARED = ["base", "yosys", "prog", "rom_words"]

# Compared ahead of everything else and NOT coverable by --allow-mismatch. See
# the module docstring: a cross-part difference is not a quantity.
PART_FIELD = "part"


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
    part = provenance.get(PART_FIELD)
    if not part:
        reject(path, "the provenance block names no part, so nothing says which "
                     "instrument\n*** produced these numbers")
    if part not in PART_REQUIRED:
        reject(path, f"the provenance block names part '{part}', which is not "
                     f"one this\n*** script knows how to grade a stamp for "
                     f"({', '.join(sorted(PART_REQUIRED))})")

    missing = [key for key in COMMON_REQUIRED + PART_REQUIRED[part]
               if not provenance.get(key)]
    if missing:
        reject(path, "the provenance block is missing " + ", ".join(missing))
    # The other direction, and the one a hand-edited stamp trips: a sweep that
    # says up5k while carrying nextpnr-ecp5, or says ecp5 while carrying icetime,
    # is a header describing a run that did not happen. Either half could be the
    # wrong one, so neither is believed over the other.
    foreign = sorted({key for other, keys in PART_REQUIRED.items() if other != part
                      for key in keys} - set(PART_REQUIRED[part]))
    carried = [key for key in foreign if provenance.get(key)]
    if carried:
        reject(path, f"the provenance block says part '{part}' but carries "
                     f"{', '.join(carried)},\n*** which belongs to another part's "
                     "instrument. One of the two is wrong")

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
    """`23-25` where the placements disagree, `23` where they do not.

    `NA` where the part has no instrument that measures this column at all --
    soc/depth/row.py writes that literal rather than a zero, and it is repeated
    here rather than turned into one. A zero in a LUT-level column is a number
    someone will subtract.
    """
    raw = [row[column] for row in rows]
    if all(value == "NA" for value in raw):
        return "NA"
    try:
        values = sorted(int(value) for value in raw)
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


# What the row's cell count is a count OF, per part. Named on every report
# because the two are not the same unit and neither is `SB_LUT4`: the up5k figure
# is nextpnr's packed ICESTORM_LC, which is what the +/-50 band is in, and the
# ECP5 figure is its TRELLIS_COMB. A column headed `lc` for both would invite the
# subtraction the part check exists to refuse.
CELL_UNIT = {"up5k": "packed ICESTORM_LC", "ecp5": "TRELLIS_COMB"}


def report(path, provenance, rows):
    s = stats(rows)
    part = provenance[PART_FIELD]
    print(f"== {path} ==")
    field("part", f"{part}" + (f"  corner {provenance['corner']}, placed against "
                               f"{provenance['constraint_mhz']} MHz"
                               if part == "ecp5" else ""))
    field("base", f"{provenance['base'][:12]} "
                  f"({'DIRTY TREE' if provenance['dirty'] != 'no' else 'clean'})")
    field("program", f"{provenance['prog']} into {provenance['rom_words']} ROM words")
    for tool in ["yosys"] + PART_REQUIRED[part]:
        if tool in ("corner", "constraint_mhz"):
            continue
        field(tool, provenance[tool])
    field("measured", f"{provenance['date']} on {provenance['host']}")
    print()
    print(f"  {s['n']} placement{'' if s['n'] == 1 else 's'} of seeds: {provenance['seeds']}")
    for label in ("worst", "median", "best"):
        field(label, f"{s[label]:6.2f} ns  {1000 / s[label]:6.2f} MHz")
    field("spread", f"{s['spread']:.1f}% of the best placement")
    field("LUT levels", f"{span(rows, 'lut_levels')}   "
                        f"carry hops: {span(rows, 'carry_hops')}   "
                        f"{CELL_UNIT[part]}: {span(rows, 'lc')}")
    if span(rows, "lut_levels") == "NA":
        field("", "those two read NA because this part has no icetime walk "
                  "behind it,")
        field("", "not because the placement had no logic on its path.")
    field("worst path", f"{rows[-1]['start']} -> {rows[-1]['end']}")
    print()
    print(bands.note(part))
    print()


def refuse_across_parts(first, second):
    """Stop, whatever flags were passed, if these two placed different parts.

    Separate from mismatches() because it is not one: --allow-mismatch exists for
    differences a reader may have a reason to subtract anyway, and this is not
    one of those. Checked first so that the diagnostic is the real reason rather
    than the four tool mismatches a cross-part pair also produces.
    """
    (first_path, first_prov), (second_path, second_prov) = first, second
    if first_prov[PART_FIELD] == second_prov[PART_FIELD]:
        return
    sys.exit(
        f"*** {first_path} placed {first_prov[PART_FIELD]} and {second_path} "
        f"placed {second_prov[PART_FIELD]}.\n"
        "*** There is no difference between them. Two parts are two fabrics,\n"
        "*** two placers and -- since only one of them has an icetime behind\n"
        "*** it -- two classes of estimate, so subtracting one from the other\n"
        "*** produces a percentage that describes nothing.\n"
        "*** --allow-mismatch does NOT cover this and is not meant to: it is\n"
        "*** for differences that can be deliberate, and this one cannot.\n"
        "*** Summarise each sweep on its own, against its own part's band."
    )


def mismatches(first, second):
    """Every recorded reason these two sweeps are not one experiment.

    A dirty tree is one of them on its own: the base line then names a commit
    that is not what was placed, so two sweeps agreeing on it have agreed about
    nothing.

    The part is not in here; refuse_across_parts() has already stopped the run.
    """
    first_path, first_prov = first
    second_path, second_prov = second
    part = first_prov[PART_FIELD]
    out = []
    for path, provenance in (first, second):
        if provenance["dirty"] != "no":
            out.append(f"{path} was measured on a tree with uncommitted changes, "
                       f"so its base names no tree")
    for key in COMMON_COMPARED + PART_REQUIRED[part]:
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


def delta(part, first, second):
    before, after = stats(first), stats(second)
    print("== delta, second sweep against first ==")
    for label in ("worst", "median", "best"):
        change = 100 * (after[label] - before[label]) / before[label]
        print(f"  {label:8s}: {before[label]:6.2f} -> {after[label]:6.2f} ns  {change:+.1f}%")
    print("  A positive number is slower. Read it against this part's own band:")
    print(bands.note(part))


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

    refuse_across_parts((first_path, first_prov), (second_path, second_prov))
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
    delta(first_prov[PART_FIELD], first_rows, second_rows)


if __name__ == "__main__":
    main()

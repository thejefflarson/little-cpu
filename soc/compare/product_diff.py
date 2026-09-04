#!/usr/bin/env python3
"""Print what moved between two soc/compare/product.json snapshots, and answer
whether it is news.

Read by the scheduled re-take (.github/workflows/compare-product-schedule.yml)
to write a pull request body a maintainer can review in one pass, and safe to
run by hand: `python3 soc/compare/product_diff.py before.json after.json`.

"NEWS" IS soc/compare/product_check.py's OWN QUESTION, ASKED OF THE OLDER
SNAPSHOT. No churn band has ever been derived for hx8k, the part this harness
places (soc/bands.py's own entry says so), so this script does not invent a
percentage threshold to decide whether a moved number is worth a pull request
-- CLAUDE.md's own rule is to measure a conflict rather than assume one, and a
threshold nobody swept is an assumption wearing a number. Instead `--require-news`
imports product_check.stale_reasons() and asks it whether the OLDER snapshot
would have called itself stale against the CURRENT tree: if `rtl/` or
`soc/compare/` moved since the older stamp's commit, or a `--current` value
disagrees with what it recorded, or a pair went from not-yet-measured to
measured, that is news regardless of how large the resulting numbers moved by.
If nothing that could move a factor moved, the newer snapshot's numbers are
expected to match the older one's and are not news even though this script
still re-measured them (`make compare-product`'s own doc says why it measures
the full pair every time rather than trying to predict whether it needs to).
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from product_check import stale_reasons  # noqa: E402


def load(path):
    try:
        with open(path) as handle:
            return json.load(handle)
    except (OSError, json.JSONDecodeError) as exc:
        sys.exit(f"*** {path}: {exc}")


def diff_pair(name, before, after):
    lines = [f"## {name}"]
    b = before.get("pairs", {}).get(name)
    a = after.get("pairs", {}).get(name)
    if a is None:
        lines.append("no longer recorded")
        return lines
    if a.get("status") != "measured":
        lines.append(f"not yet measured -- {a.get('reason', 'no reason recorded')}")
        return lines
    if b is None or b.get("status") != "measured":
        lines.append(f"measured for the first time, ISA {a['isa']}, target "
                     f"core {a['target_core']}:")
        for core in sorted(a["products"]):
            ratio = a["products"][core]["ratio"]
            lines.append(f"  {core}: {ratio['worst']:.3f}x worst, "
                         f"{ratio['median']:.3f}x median")
        return lines

    lines.append(f"ISA: {b['isa']} -> {a['isa']}")
    for core in sorted(set(a["products"]) | set(b["products"])):
        pb, pa = b["products"].get(core), a["products"].get(core)
        if pa is None:
            lines.append(f"  {core}: no longer a product in this pair")
            continue
        if pb is None:
            lines.append(f"  {core}: a new product in this pair, "
                         f"{pa['ratio']['worst']:.3f}x worst, "
                         f"{pa['ratio']['median']:.3f}x median")
            continue
        for label, key in (("worst", "worst"), ("median", "median")):
            rb, ra = pb["ratio"][key], pa["ratio"][key]
            pct = 100.0 * (ra - rb) / rb
            lines.append(f"  {core} ratio, {label}: {rb:.3f}x -> {ra:.3f}x  "
                         f"({pct:+.1f}%)")
    for core in sorted(set(a["cores"]) & set(b["cores"])):
        cb, ca = b["cores"][core]["clock_mhz"], a["cores"][core]["clock_mhz"]
        lines.append(f"  {core} clock, worst: {cb['worst_mhz']:.2f} -> "
                     f"{ca['worst_mhz']:.2f} MHz")
        fb, fa = b["cores"][core]["cycle_factor"], a["cores"][core]["cycle_factor"]
        lines.append(f"  {core} cycle factor: {fb:.3f} -> {fa:.3f} {a['unit']}")
    lines.append(f"base: {b['base'][:12]} -> {a['base'][:12]}")
    return lines


def is_news(name, before, after, repo, current):
    """Whether `before`'s copy of pair `name` would call itself stale against
    the tree `repo` names right now -- see the module docstring.
    """
    b = before.get("pairs", {}).get(name)
    if b is None:
        return after.get("pairs", {}).get(name) is not None
    if b.get("status") != "measured":
        a = after.get("pairs", {}).get(name)
        return a is not None and a.get("status") == "measured"
    return bool(stale_reasons(b, repo, current.get(name, {})))


def parse_current(specs):
    """`BENCHMARK:FIELD=VALUE` strings into {benchmark: {field: value}}."""
    current = {}
    for spec in specs:
        if ":" not in spec or "=" not in spec:
            sys.exit(f"*** --current wants BENCHMARK:FIELD=VALUE, got '{spec}'")
        benchmark, rest = spec.split(":", 1)
        field, value = rest.split("=", 1)
        current.setdefault(benchmark, {})[field] = value
    return current


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("before")
    parser.add_argument("after")
    parser.add_argument("--repo", default=".", help="the git worktree "
                        "--require-news checks the older snapshot against")
    parser.add_argument("--current", action="append", default=[],
                        metavar="BENCHMARK:FIELD=VALUE", help="a value the "
                        "build has right now, for --require-news's use of "
                        "product_check.stale_reasons()")
    parser.add_argument(
        "--require-news",
        action="store_true",
        help="exit 0 if any pair is news by product_check's own definition, "
        "1 if nothing is (the workflow's PR/no-PR decision); with this flag, "
        "nothing is printed except a one-line verdict",
    )
    args = parser.parse_args()
    current = parse_current(args.current)

    before, after = load(args.before), load(args.after)
    names = sorted(set(before.get("pairs", {})) | set(after.get("pairs", {})))
    if not names:
        if args.require_news:
            print("neither snapshot names a benchmark pair")
            sys.exit(1)
        print("neither snapshot names a benchmark pair.")
        return

    if args.require_news:
        news = any(is_news(name, before, after, args.repo, current) for name in names)
        print("news" if news else "no news")
        sys.exit(0 if news else 1)

    for name in names:
        for line in diff_pair(name, before, after):
            print(line)
        print()


if __name__ == "__main__":
    main()

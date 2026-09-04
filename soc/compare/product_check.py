#!/usr/bin/env python3
"""Refuse to print a cross-core product from a stamp the tree has moved past.

THE DEFECT THIS CLOSES: the cross-core throughput product is a number derived
from two factors, stored only in CLAUDE.md's prose, with nothing that notices
when either factor moves under it. That has happened twice on record -- once
inside a day (ADR-0098's amendment) and once by the maintainer re-deriving it
by hand a second time -- because nothing was checking. This is the check.

soc/compare/product.json (soc/compare/product_write.py's output) stamps every
measured pair with the commit it was taken at, whether the tree was dirty, and
every value that can move a factor without moving a tracked file (CFLAGS, the
ROM/RAM geometry) -- the same fields soc/baseline_sweep.sh/soc/baseline_summary.py
already stamp a timing sweep with, for the same reason. A pair is STALE when
any of three things is true:

  - the tree has moved under it: `rtl/` or `soc/compare/` differ between the
    stamped base and now, checked with a real `git diff` rather than trusted.
  - a value the caller hands in as --current FIELD=VALUE disagrees with the
    same-named field in the stamp -- `cflags`, `rom_words`, `ram_words`,
    whichever fields the caller can currently ask the build for. Generic
    rather than one hardcoded `--current-cflags` flag: CFLAGS was the first
    value found to live outside both watched path prefixes, and it will not be
    the last, so the check takes any field name rather than growing a new flag
    per future one.
  - it was measured DIRTY. `dirty: yes` means the base commit does not fully
    describe what was measured -- there were uncommitted changes in the tree
    at measurement time -- so a base that still "matches" the tree today is not
    good enough; the stamp never named a reproducible tree to begin with.

NOT ON `make test`'s PATH. `make compare-dhrystone` and (once it exists)
`make compare-coremark` call this after their own measurement, so a stale
artifact silently stops being quoted at the one place a human would otherwise
read it and not from a check nobody asked for on every `rtl/` commit -- the
same reasoning CLAUDE.md gives for keeping every `make compare-*` off CI.

soc/compare/product_diff.py imports stale_reasons() from here rather than
inventing a second opinion about what counts as stale: the only thing that
script adds is a plain-English diff between two already-written snapshots for
a pull request body, and whether either one is current is this file's question
alone.

Usage:
  product_check.py product.json dhrystone --repo . \\
    --current cflags='...' \\
    --current rom_words=1024 --current ram_words=512
"""

import argparse
import json
import subprocess
import sys


def load(path):
    try:
        with open(path) as handle:
            return json.load(handle)
    except FileNotFoundError:
        sys.exit(f"*** {path} does not exist. Run `make compare-product` first; "
                 "there is no product to report on without it.")
    except (OSError, json.JSONDecodeError) as exc:
        sys.exit(f"*** {path} could not be read as the product artifact: {exc}")


def moved_paths(repo, base):
    """Every path under rtl/ or soc/compare/ that differs between `base` and the
    working tree (committed or not -- a stamp is stale the moment either
    factor's inputs move, whether or not the move has been committed yet).
    """
    try:
        out = subprocess.run(
            ["git", "-C", repo, "diff", "--name-only", base, "--",
             "rtl/", "soc/compare/"],
            capture_output=True, text=True, check=True,
        )
    except FileNotFoundError:
        sys.exit("*** no git on PATH, so the stamped commit cannot be compared "
                 "against the tree.")
    except subprocess.CalledProcessError as exc:
        sys.exit(f"*** git could not diff '{base}' against the tree in '{repo}': "
                 f"{exc.stderr.strip()}\n*** That is a stamp this script cannot "
                 "grade, which is the same as a stale one -- it names no tree "
                 "this check can confirm is still current.")
    return [line for line in out.stdout.splitlines() if line]


def stale_reasons(pair, repo, current):
    """Every reason a MEASURED `pair` is stale, or [] if it is fresh.

    `current` is a dict of field name to the value the build has right now --
    only the fields the caller actually asked about are checked, so a caller
    that does not know a field's current value simply omits it rather than
    forcing a guess.
    """
    reasons = []
    if pair.get("dirty") == "yes":
        reasons.append("it was measured on a tree with uncommitted changes, so "
                       "its base commit does not fully describe what was "
                       "measured")
    paths = moved_paths(repo, pair["base"])
    if paths:
        reasons.append("rtl/ or soc/compare/ changed since "
                       f"{pair['base'][:12]}: {', '.join(paths)}")
    for field, value in current.items():
        stamped = pair.get(field)
        if stamped is not None and str(value) != str(stamped):
            reasons.append(f"{field} changed: stamped '{stamped}', now '{value}'")
    return reasons


def report_pair(benchmark, pair, args):
    status = pair.get("status")
    if status == "not_yet_measured":
        print(f"{benchmark}: not yet measured -- {pair.get('reason', 'no reason recorded')}")
        return 0
    if status != "measured":
        sys.exit(f"*** {benchmark}'s status is '{status}', which is neither "
                 "'measured' nor 'not_yet_measured'. That is not a stamp this "
                 "script knows how to grade.")

    reasons = stale_reasons(pair, args.repo, args.current)
    if reasons:
        print(f"*** STALE: {benchmark}'s product stamp is from "
             f"{pair['base'][:12]} ({pair['date']}), and:")
        for reason in reasons:
            print(f"***   {reason}")
        print(f"*** Re-run `make compare-product` before quoting {benchmark}'s "
             "product again.")
        return 1

    cores = pair["cores"]
    target = pair["target_core"]
    print(f"{benchmark}: fresh, stamped {pair['base'][:12]} ({pair['date']}), "
         f"ISA {pair['isa']}")
    for core in sorted(cores):
        c = cores[core]["clock_mhz"]
        print(f"  {core:12s} clock {c['worst_mhz']:.2f}/{c['median_mhz']:.2f} MHz "
             f"worst/median   cycle factor {cores[core]['cycle_factor']:.3f} "
             f"{pair['unit']}")
    for core in sorted(pair["products"]):
        p = pair["products"][core]
        ratio = p["ratio"]
        print(f"  {core} against {target}: {ratio['worst']:.3f}x worst-on-worst, "
             f"{ratio['median']:.3f}x median-on-median")
    return 0


def parse_current(specs):
    current = {}
    for spec in specs:
        if "=" not in spec:
            sys.exit(f"*** --current wants FIELD=VALUE, got '{spec}'")
        field, value = spec.split("=", 1)
        current[field] = value
    return current


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("product_json")
    parser.add_argument("benchmark", help="dhrystone or coremark")
    parser.add_argument("--repo", default=".", help="the git worktree to check "
                        "the stamp's paths against")
    parser.add_argument("--current", action="append", default=[],
                        metavar="FIELD=VALUE", help="a value the build has "
                        "right now for a field the stamp also records (cflags, "
                        "rom_words, ram_words, ...); repeatable, omit to skip "
                        "that field's check")
    args = parser.parse_args()
    args.current = parse_current(args.current)

    doc = load(args.product_json)
    pairs = doc.get("pairs", {})
    pair = pairs.get(args.benchmark)
    if pair is None:
        sys.exit(f"*** {args.product_json} has no '{args.benchmark}' pair. Known: "
                 f"{', '.join(sorted(pairs)) or '(none)'}. Run "
                 "`make compare-product` first.")

    sys.exit(report_pair(args.benchmark, pair, args))


if __name__ == "__main__":
    main()

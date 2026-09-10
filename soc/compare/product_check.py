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
import os
import subprocess
import sys

# THREE exit statuses, and the third one is why this is not `sys.exit(message)`.
REFUSED = 2

def refuse(message):
    print(message, file=sys.stderr)
    sys.exit(REFUSED)

def load(path):
    try:
        with open(path) as handle:
            return json.load(handle)
    except FileNotFoundError:
        refuse(f"*** {path} does not exist. Run `make compare-product` first; "
               "there is no product to report on without it.")
    except (OSError, json.JSONDecodeError) as exc:
        refuse(f"*** {path} could not be read as the product artifact: {exc}")

# The artifact lives INSIDE a watched prefix, so writing it counts as the tree moving:
# stamping dhrystone and then writing coremark made dhrystone stale against its own file,
# and `make compare-product` could never exit 0. Excluded by the path the caller actually
# gave, not by a prefix -- a prefix would stop this noticing a real soc/compare/ change.
def moved_paths(repo, base, artifact=None):
    """Every path under rtl/ or soc/compare/ that differs between `base` and the
    working tree (committed or not -- a stamp is stale the moment either
    factor's inputs move, whether or not the move has been committed yet).
    """
    try:
        pathspec = ["rtl/", "soc/compare/"]
        if artifact is not None:
            rel = os.path.relpath(os.path.abspath(artifact), os.path.abspath(repo))
            if not rel.startswith(os.pardir):
                pathspec.append(":(exclude)" + rel)
        out = subprocess.run(
            ["git", "-C", repo, "diff", "--name-only", base, "--"] + pathspec,
            capture_output=True, text=True, check=True,
        )
    except FileNotFoundError:
        refuse("*** no git on PATH, so the stamped commit cannot be compared "
               "against the tree.")
    except subprocess.CalledProcessError as exc:
        refuse(f"*** git could not diff '{base}' against the tree in '{repo}': "
               f"{exc.stderr.strip()}\n*** That is a stamp this script cannot "
               "grade, which is the same as a stale one -- it names no tree "
               "this check can confirm is still current.")
    return [line for line in out.stdout.splitlines() if line]

def stale_reasons(pair, repo, current, artifact=None):
    """Every reason a MEASURED `pair` is stale, or [] if it is fresh.

    `current` is a dict of field name to the value the build has right now --
    only the fields the caller actually asked about are checked, so a caller
    that does not know a field's current value simply omits it rather than
    forcing a guess.

    An EMPTY value is refused rather than compared: a caller that could not
    determine a field's current value (a `make print-VAR` on a VAR that does
    not exist on this tree resolves empty rather than erroring) must omit the
    field, the same way a caller that does not know it does. Comparing an
    empty value against a real stamped one still catches drift, but comparing
    it against a stamp that -- through some future writer bug -- also stamped
    empty would compare equal and call a moved field fresh; refusing the
    empty value outright closes that whether or not it has happened yet.
    """
    for field, value in current.items():
        # `.strip()`, not `== ""`: a `make print-VAR` that resolved to whitespace is the
        # same non-answer, and it passes a shell `[ -n ]` test on the way here.
        if str(value).strip() == "":
            refuse(f"*** --current {field}= is empty. That is not a value "
                   "to compare against the stamp -- it means whatever "
                   "produced it could not determine the field, most often "
                   "a `make print-VAR` on a VAR this tree does not define. "
                   "Omit the field instead of asking to compare it.")
    reasons = []
    if pair.get("dirty") == "yes":
        reasons.append("it was measured on a tree with uncommitted changes, so "
                       "its base commit does not fully describe what was "
                       "measured")
    paths = moved_paths(repo, pair["base"], artifact)
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
        refuse(f"*** {benchmark}'s status is '{status}', which is neither "
               "'measured' nor 'not_yet_measured'. That is not a stamp this "
               "script knows how to grade.")

    reasons = stale_reasons(pair, args.repo, args.current, args.product_json)
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
            refuse(f"*** --current wants FIELD=VALUE, got '{spec}'")
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
        refuse(f"*** {args.product_json} has no '{args.benchmark}' pair. Known: "
               f"{', '.join(sorted(pairs)) or '(none)'}. Run "
               "`make compare-product` first.")

    sys.exit(report_pair(args.benchmark, pair, args))

if __name__ == "__main__":
    main()

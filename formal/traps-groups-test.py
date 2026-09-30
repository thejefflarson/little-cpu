#!/usr/bin/env python3
"""Grades formal/traps.sv's TRAPS_CHECK_* split: every assert() and cover() under
`ifdef FORMAL must belong to EXACTLY one group, every group must be non-empty, and
the counts must match formal/TRAPS_GROUPS -- both directions, the way
formal/EXPECTED_CHECKS is graded, so a group that silently lost its last assertion
(or gained an untagged one, invisible to every split task) is caught here rather
than passing every split task by omission.

Usage: traps-groups-test.py [--repo DIR] [--traps FILE]
"""

import argparse
import pathlib
import re
import sys

GROUPS = ("PC", "CAUSE", "STATUS", "QUIESCENCE")
IFDEF = re.compile(r"^\s*`(ifdef|ifndef)\s+(\S+)")
ENDIF = re.compile(r"^\s*`endif\b")
STMT = re.compile(r"\b(assert|cover)\s*\(")

def stop(message):
    print(f"error: {message}", file=sys.stderr)
    sys.exit(2)

def scan(text):
    """Returns {line_no: (kind, [group_names])} for every assert/cover statement."""
    stack = []
    found = {}
    for lineno, line in enumerate(text.splitlines(), 1):
        m = IFDEF.match(line)
        if m:
            stack.append(m.group(2))
            continue
        if ENDIF.match(line):
            if stack:
                stack.pop()
            continue
        for kind_match in STMT.finditer(line):
            groups = [g for g in stack if g.startswith("TRAPS_CHECK_")]
            found.setdefault(lineno, []).append((kind_match.group(1), groups))
    return found

def main():
    here = pathlib.Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", default=str(here.parent))
    parser.add_argument("--traps", default=None)
    parser.add_argument("--manifest", default=None)
    args = parser.parse_args()

    repo = pathlib.Path(args.repo).resolve()
    traps_path = pathlib.Path(args.traps) if args.traps else repo / "formal" / "traps.sv"
    if not traps_path.is_file():
        stop(f"{traps_path} is missing, so there is nothing to grade.")
    manifest_path = pathlib.Path(args.manifest) if args.manifest else repo / "formal" / "TRAPS_GROUPS"
    if not manifest_path.is_file():
        stop(f"{manifest_path} is missing; it is the tracked count this file grades against.")

    text = traps_path.read_text()
    found = scan(text)

    errors = []
    counts = {g: 0 for g in GROUPS}
    for lineno, stmts in found.items():
        for kind, groups in stmts:
            unknown = [g for g in groups if g[len("TRAPS_CHECK_"):] not in GROUPS]
            if unknown:
                errors.append(f"line {lineno}: {kind}() tagged with unknown group(s) {unknown}")
                continue
            names = [g[len("TRAPS_CHECK_"):] for g in groups]
            if len(names) == 0:
                errors.append(
                    f"line {lineno}: {kind}() is not inside any TRAPS_CHECK_* block -- "
                    "a real split (TRAPS_SPLIT defined) would silently drop it from "
                    "every task, and no group's induction hypothesis would ever see it"
                )
            elif len(names) > 1:
                errors.append(
                    f"line {lineno}: {kind}() is nested inside more than one TRAPS_CHECK_* "
                    f"block ({names}) -- pick one group per statement"
                )
            else:
                counts[names[0]] += 1

    empty = [g for g in GROUPS if counts[g] == 0]
    if empty:
        errors.append(f"group(s) with no assert or cover at all: {empty}")

    manifest = {}
    for line in manifest_path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        name, count = line.split()
        manifest[name] = int(count)

    if set(manifest) != set(GROUPS):
        errors.append(
            f"{manifest_path} names {sorted(manifest)}, traps.sv's own groups are "
            f"{sorted(GROUPS)} -- the two must name the same set"
        )
    else:
        for g in GROUPS:
            if manifest[g] != counts[g]:
                errors.append(
                    f"group {g}: traps.sv has {counts[g]} assert/cover statements, "
                    f"{manifest_path} says {manifest[g]} -- update the manifest with the "
                    "reason, not the number alone"
                )

    if errors:
        for e in errors:
            print(f"*** {e}", file=sys.stderr)
        sys.exit(1)

    total = sum(counts.values())
    print(f"traps-groups: {total} assert/cover statements, each in exactly one of "
          f"{len(GROUPS)} groups, matching {manifest_path.name}.")
    for g in GROUPS:
        print(f"  {g}: {counts[g]}")

if __name__ == "__main__":
    main()

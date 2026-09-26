#!/usr/bin/env python3
"""Grades rtl/executor.v's TRAPS_SKIP_EXEC_ARITH exclusion: the number of assert()
statements it guards must match formal/TRAPS_ARITH_EXCLUDED -- both directions, the
way formal/EXPECTED_CHECKS is graded -- and the macro must be defined on exactly the
five traps-composition tasks' own read of executor.v in formal/components.sby, never
on components_executor's (or any other task's), which is what makes "excluded from
traps.sv" and "still proven by components_executor" the same fact rather than two
that could drift apart.

Usage: traps-arith-excluded-test.py [--repo DIR] [--executor FILE] [--manifest FILE]
                                     [--components-sby FILE]
"""

import argparse
import pathlib
import re
import sys

TRAPS_TASKS = ("traps", "traps_pc", "traps_cause", "traps_status", "traps_quiescence")
OTHER_TASKS = ("decoder", "executor", "accessor", "pcloop", "busarbiter")
MACRO = "TRAPS_SKIP_EXEC_ARITH"

IFDEF = re.compile(r"^\s*`(ifdef|ifndef)\s+(\S+)")
ENDIF = re.compile(r"^\s*`endif\b")
ASSERT = re.compile(r"\bassert\s*\(")
LABEL = re.compile(r"^([A-Za-z_][A-Za-z0-9_ ]*):\s*$")
SECTION = re.compile(r"^\[(\w+)\]\s*$")

def stop(message):
    print(f"error: {message}", file=sys.stderr)
    sys.exit(2)

def count_excluded_asserts(text):
    """The number of assert() statements textually nested inside `ifndef MACRO."""
    stack = []
    count = 0
    for line in text.splitlines():
        m = IFDEF.match(line)
        if m:
            stack.append((m.group(1), m.group(2)))
            continue
        if ENDIF.match(line):
            if stack:
                stack.pop()
            continue
        if ASSERT.search(line) and any(
            kind == "ifndef" and name == MACRO for kind, name in stack
        ):
            count += 1
    return count

def script_tasks(components_sby):
    """{task_name: [lines]} for every task in components.sby's [script] section."""
    section, label, tasks = None, None, {}
    for line in components_sby.splitlines():
        found = SECTION.match(line)
        if found:
            section, label = found.group(1), None
            continue
        if section != "script":
            continue
        if line.strip() == "--":
            break
        found = LABEL.match(line)
        if found:
            label = found.group(1)
            tasks[label] = []
            continue
        if label is not None:
            tasks[label].append(line)
    return tasks

def main():
    here = pathlib.Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", default=str(here.parent))
    parser.add_argument("--executor", default=None)
    parser.add_argument("--manifest", default=None)
    parser.add_argument("--components-sby", default=None)
    args = parser.parse_args()

    repo = pathlib.Path(args.repo).resolve()
    executor_path = pathlib.Path(args.executor) if args.executor else repo / "rtl" / "executor.v"
    if not executor_path.is_file():
        stop(f"{executor_path} is missing, so there is nothing to grade.")
    manifest_path = (
        pathlib.Path(args.manifest) if args.manifest else repo / "formal" / "TRAPS_ARITH_EXCLUDED"
    )
    if not manifest_path.is_file():
        stop(f"{manifest_path} is missing; it is the tracked count this file grades against.")
    sby_path = (
        pathlib.Path(args.components_sby)
        if args.components_sby
        else repo / "formal" / "components.sby"
    )
    if not sby_path.is_file():
        stop(f"{sby_path} is missing, so there is nothing to grade.")

    errors = []

    found = count_excluded_asserts(executor_path.read_text())
    manifest_lines = [
        line.strip()
        for line in manifest_path.read_text().splitlines()
        if line.strip() and not line.strip().startswith("#")
    ]
    if len(manifest_lines) != 1 or not manifest_lines[0].isdigit():
        stop(f"{manifest_path} must hold exactly one integer, found {manifest_lines!r}.")
    expected = int(manifest_lines[0])
    if found != expected:
        errors.append(
            f"rtl/executor.v guards {found} assert() statement(s) behind "
            f"`ifndef {MACRO}, {manifest_path.name} says {expected} -- update the "
            "manifest with the reason, not the number alone"
        )

    tasks = script_tasks(sby_path.read_text())
    for task in TRAPS_TASKS:
        lines = tasks.get(task)
        if lines is None:
            errors.append(f"{sby_path.name} has no '{task}:' task to grade")
            continue
        executor_reads = [l for l in lines if "executor.v" in l]
        if not any(f"-D {MACRO}" in l for l in executor_reads):
            errors.append(
                f"task '{task}' reads executor.v without -D {MACRO} -- the exclusion "
                "this file grades would not actually apply there"
            )
    for task in OTHER_TASKS:
        lines = tasks.get(task)
        if lines is None:
            errors.append(f"{sby_path.name} has no '{task}:' task to grade")
            continue
        if any(MACRO in l for l in lines):
            errors.append(
                f"task '{task}' defines {MACRO} -- that macro belongs only to the "
                "traps family, since components_executor must prove every excluded "
                "assertion unconditionally"
            )

    if errors:
        for e in errors:
            print(f"*** {e}", file=sys.stderr)
        sys.exit(1)

    print(
        f"traps-arith-excluded: {found} assert() statement(s) behind `ifndef {MACRO}, "
        f"matching {manifest_path.name}; the macro is defined on exactly "
        f"{', '.join(TRAPS_TASKS)}'s own read of executor.v."
    )

if __name__ == "__main__":
    main()

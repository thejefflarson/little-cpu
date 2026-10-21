#!/usr/bin/env python3
"""Asserts that every workflow job runs on the self-hosted pool.

Usage: workflow_runner_test.py [repo-root]     # defaults to this script's parent

WHY THIS EXISTS. CI runs on `little-cpu-runners`; a job on a hosted runner is spend the
pool was meant to absorb, and a job quietly left on `ubuntu-latest` stays there. The pool
is self-hosted and the repo is public, so the other direction matters more: a pull-request
workflow that names the pool bare would run a fork's code on it.

A `runs-on:` value is accepted when it is
  - `little-cpu-runners`, in a workflow with no pull-request trigger; or
  - the fork-fallback expression, which picks the pool only when the PR's head repo is
    this repo and `ubuntu-latest` otherwise.
A workflow triggered by `pull_request_target` is refused outright: it runs with the base
repository's token against the PR's contents.

EXCEPTIONS is {workflow file: reason} for a file allowed other runners. It is empty.

Hermetic: file reads only.
"""

import pathlib
import re
import sys

EXCEPTIONS = {}

FALLBACK = (
    "${{ (github.event_name != 'pull_request' || "
    "github.event.pull_request.head.repo.full_name == github.repository) "
    "&& 'little-cpu-runners' || 'ubuntu-latest' }}"
)

RUNS_ON = re.compile(r"^\s*runs-on:\s*(.*?)\s*$")
PR_TRIGGER = re.compile(r"^(on:\s*pull_request\s*$|\s+pull_request:)", re.M)
PR_TARGET = re.compile(r"^(on:\s*pull_request_target\b|\s+pull_request_target:)", re.M)


def problems(path):
    text = path.read_text()
    if path.name in EXCEPTIONS:
        return []
    found = []
    if PR_TARGET.search(text):
        found.append(f"{path.name}: pull_request_target runs on the base repo's token")
    on_pr = bool(PR_TRIGGER.search(text))
    seen = 0
    for number, line in enumerate(text.splitlines(), 1):
        match = RUNS_ON.match(line)
        if not match:
            continue
        seen += 1
        value = match.group(1)
        if value == FALLBACK:
            continue
        where = f"{path.name}:{number}"
        if value == "little-cpu-runners":
            if on_pr:
                found.append(f"{where}: bare little-cpu-runners in a pull-request workflow "
                             "would run a fork's code on the pool; use the fork-fallback expression")
        else:
            found.append(f"{where}: runs-on {value!r} is not little-cpu-runners")
    if not seen:
        found.append(f"{path.name}: no runs-on line found")
    return found


def main(argv):
    root = pathlib.Path(argv[1]) if len(argv) > 1 else pathlib.Path(__file__).resolve().parent.parent
    if not root.is_dir():
        print(f"error: {root} is not a directory", file=sys.stderr)
        return 1
    files = sorted((root / ".github" / "workflows").glob("*.yml"))
    if not files:
        print("error: no workflow files found", file=sys.stderr)
        return 1
    found = [p for f in files for p in problems(f)]
    for p in found:
        print(f"WORKFLOW RUNNERS: {p}", file=sys.stderr)
    if found:
        return 1
    print(f"WORKFLOW RUNNERS: PASS ({len(files)} workflows)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

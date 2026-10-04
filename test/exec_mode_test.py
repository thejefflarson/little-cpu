#!/usr/bin/env python3
"""Refuse a tracked script the Makefile runs directly that git does not record as executable.

A recipe that runs `./soc/x.sh` needs mode 100755 in the index: a checkout of a 100644
file stops with "permission denied", and only on the recipe that runs it, which may be one
nobody exercises before a board is plugged in. Reads the root Makefile and every tracked
*.mk, joins continuation lines, and grades each `./path` in command position -- the start
of a recipe line, after a shell separator, or after a word that runs its argument -- that
names a tracked file. `python3 ./x.py` is an argument, not a command, and is not graded.

Usage: exec_mode_test.py [REPO]
"""

import re
import subprocess
import sys

COMMAND_BEFORE = re.compile(
    r"(?:^\t[@+-]*|[;|&({`,!][@+-]*|\$\(|\b(?:then|do|else|exec|sudo)|\$\(ICEPROG_SUDO\))\s*$")
PATH_TOKEN = re.compile(r"\./([A-Za-z0-9_./-]+)")


def tracked_modes(repo):
    out = subprocess.run(["git", "-C", repo, "ls-files", "-s"], capture_output=True,
                         text=True, check=False)
    if out.returncode != 0 or not out.stdout:
        sys.exit(f"error: git cannot enumerate any tracked files under {repo}")
    modes = {}
    for line in out.stdout.splitlines():
        meta, path = line.split("\t", 1)
        modes[path] = meta.split()[0]
    return modes


def logical_lines(text):
    number, held, start = 0, "", 1
    for raw in text.splitlines():
        number += 1
        if not held:
            start = number
        if raw.endswith("\\"):
            held += raw[:-1] + " "
            continue
        yield start, held + raw
        held = ""
    if held:
        yield start, held


def direct_runs(text):
    for number, line in logical_lines(text):
        if not line.startswith("\t"):
            continue
        for match in PATH_TOKEN.finditer(line):
            if COMMAND_BEFORE.search(line[:match.start()]):
                yield number, match.group(1)


def main():
    repo = sys.argv[1] if len(sys.argv) > 1 else "."
    modes = tracked_modes(repo)
    makefiles = ["Makefile"] + sorted(p for p in modes if p.endswith(".mk"))
    bad, graded = [], 0
    for makefile in makefiles:
        if makefile not in modes:
            continue
        with open(f"{repo}/{makefile}") as handle:
            text = handle.read()
        for number, path in direct_runs(text):
            if path not in modes:
                continue
            graded += 1
            if modes[path] != "100755":
                bad.append(f"{makefile}:{number}: runs ./{path}, which git records as "
                           f"{modes[path]}; `git update-index --chmod=+x {path}`")
    if not graded:
        sys.exit("error: no recipe runs a tracked script directly, so nothing was graded")
    for line in bad:
        print(f"error: {line}", file=sys.stderr)
    if bad:
        sys.exit(1)
    print(f"exec-mode: {graded} direct run(s) of a tracked script, every one executable")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Grades the three layers that keep a nano figure bound to its design stamp.

Usage: figure_stamp_test.py [repo-root]     # defaults to this script's parent

  1. ONE SOURCE. nano/figure_stamp.py is the only file that writes the stamp grammar or the
     design list. Another tracked file that joins or splits on the stamp separator, or lists
     both designs as a literal, is a second copy.
  2. EMISSION. Every file under nano/ (and each nano workflow) that prints a figure with a
     unit routes it through figure_stamp. A new emitter that does not is named here.
  3. PROSE. A line in a tracked markdown file that names nano and holds a figure carries a
     valid stamp. docs/adr/ is a record of dated measurements: ADRs below
     figure_stamp.FIRST_STAMPED_ADR are left as written and later ones are held to the rule.
     docs/ideas/ is exempt for the reason band_source_test.py exempts it.

The runtime halves are `figure_stamp.py check` over each emitter's output and
`figure_stamp.py ratio`, both driven by test/probe_gates.sh. Hermetic: git and file reads.
"""

import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SOURCE = "nano/figure_stamp.py"

EXEMPT_FILES = {SOURCE, "test/figure_stamp_test.py", "test/probe_gates.sh", "test/PROBES_EXPECTED"}
RESTATEMENT = re.compile(
    "[\"'][ ]*·[ ]*[\"']|"
    r"[\"']littlecpu[\"']\s*,\s*[\"']nanocpu[\"']|[\"']nanocpu[\"']\s*,\s*[\"']littlecpu[\"']"
)
EMITTER_PATHS = re.compile(r"^(nano/.*\.(py|sh)|\.github/workflows/nano-.*\.yml)$")
PRINTER = re.compile(r"print\(|\becho\b|\bprintf\b|summary_line")
NANO_NAME = re.compile(r"\bnano(cpu)?\b", re.IGNORECASE)
ADR_NUMBER = re.compile(r"^docs/adr/(\d{4})-.*\.md$")


def load_module(root):
    sys.path.insert(0, os.path.join(root, "nano"))
    import figure_stamp
    return figure_stamp


def tracked(root):
    out = subprocess.run(["git", "-C", root, "ls-files"], check=True,
                         capture_output=True, text=True).stdout
    return [line for line in out.splitlines() if line]


def read(root, path):
    try:
        with open(os.path.join(root, path), encoding="utf-8") as handle:
            return handle.read().splitlines()
    except (OSError, UnicodeDecodeError):
        return []


def second_copies(root, files):
    return [f"{path}:{n}: restates the stamp grammar or the design list; it lives only in {SOURCE}"
            for path in files if path not in EXEMPT_FILES
            for n, line in enumerate(read(root, path), 1) if RESTATEMENT.search(line)]


def unrouted_emitters(root, files, stamp):
    problems = []
    for path in files:
        if path == SOURCE or not EMITTER_PATHS.match(path):
            continue
        lines = read(root, path)
        prints_figure = any(PRINTER.search(line) and stamp.FIGURE.search(line) for line in lines)
        if prints_figure and not any("figure_stamp" in line for line in lines):
            problems.append(f"{path}: prints a figure and never routes it through figure_stamp")
    return problems


def bare_prose_figures(root, files, stamp):
    problems = []
    for path in files:
        if not (path.endswith(".md") or path == "README"):
            continue
        if path.startswith("docs/ideas/") or path == "docs/adr/README.md":
            continue
        adr = ADR_NUMBER.match(path)
        if adr and int(adr.group(1)) < stamp.FIRST_STAMPED_ADR:
            continue
        for n, line in enumerate(read(root, path), 1):
            if NANO_NAME.search(line) and stamp.FIGURE.search(line):
                try:
                    stamp.parse(line)
                except stamp.StampError as exc:
                    problems.append(f"{path}:{n}: nano figure without a valid stamp ({exc})")
    return problems


def main(argv):
    root = os.path.abspath(argv[1]) if len(argv) > 1 else os.path.dirname(HERE)
    stamp = load_module(root)
    files = tracked(root)
    problems = second_copies(root, files)
    problems += unrouted_emitters(root, files, stamp)
    problems += bare_prose_figures(root, files, stamp)
    if problems:
        print("figure stamp check FAILED:", file=sys.stderr)
        for problem in problems:
            print(f"  {problem}", file=sys.stderr)
        return 1
    print(f"every nano figure is stamped, and the grammar is {SOURCE}'s alone")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

#!/usr/bin/env python3
"""Refuses a backslash-newline inside the quoted script of a `yosys -p` recipe line,
whether the program is spelled `yosys` or `$(YOSYS)` and whether `-p` follows it on the
same line or on a continuation.

GNU Make 3.81 (macOS) strips the backslash-newline inside single quotes; 4.x keeps both
characters and hands yosys `\\`, which dies with `No such command: \\`. A script that
passes locally and fails on the runners is the shape this exists to stop.

Usage: yosys_script_oneline_test.py [repo-root]
"""

import pathlib
import re
import sys

FILES = ["Makefile", "nano/*.mk"]
JOINED = "\x01"  # stands for a backslash-newline once a recipe's physical lines are joined
YOSYS_P = re.compile(
    r"(?:\byosys\b|\$[({]YOSYS[)}])[^#\n]*?[\s" + JOINED + r"]-p[\s" + JOINED + r"]+(?=['\"])")


def logical_lines(text):
    """(first physical line number, joined text) per make logical line."""
    out = []
    physical = text.splitlines()
    i = 0
    while i < len(physical):
        start = i
        parts = [physical[i]]
        while physical[i].rstrip().endswith("\\") and i + 1 < len(physical):
            parts[-1] = parts[-1].rstrip()[:-1]
            i += 1
            parts.append(physical[i])
        out.append((start + 1, JOINED.join(parts)))
        i += 1
    return out


def violations(path):
    found = []
    for first, logical in logical_lines(path.read_text()):
        for m in YOSYS_P.finditer(logical):
            quote = None
            for at in range(m.end(), len(logical)):
                ch = logical[at]
                if quote is None:
                    if ch in "'\"":
                        quote = ch
                elif ch == quote:
                    break
                elif ch == JOINED:
                    found.append(first + logical.count(JOINED, 0, at))
    return sorted(set(found))


def main(argv):
    root = pathlib.Path(argv[1] if len(argv) > 1 else pathlib.Path(__file__).parent.parent)
    paths = sorted(p for pat in FILES for p in root.glob(pat))
    if not paths:
        print(f"error: no Makefile or nano/*.mk under {root}", file=sys.stderr)
        return 1
    failures = [f"{p.relative_to(root)}:{n}" for p in paths for n in violations(p)]
    if failures:
        for f in failures:
            print(f"*** {f}: a backslash-newline inside a quoted `yosys -p` script; "
                  "GNU Make 4.x passes the backslash to yosys. Put the script on one line.",
                  file=sys.stderr)
        return 1
    print(f"yosys-script-oneline: no quoted `yosys -p` script in {len(paths)} files "
          "spans a backslash-newline.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

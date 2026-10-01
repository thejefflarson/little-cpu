#!/usr/bin/env python3
"""Refuses a backslash-newline inside the quoted script of a `yosys -p` recipe line.

GNU Make 3.81 (macOS) strips the backslash-newline inside single quotes; 4.x keeps both
characters and hands yosys `\\`, which dies with `No such command: \\`. A script that
passes locally and fails on the runners is the shape this exists to stop.

Usage: yosys_script_oneline_test.py [repo-root]
"""

import pathlib
import re
import sys

FILES = ["Makefile", "nano/*.mk"]
YOSYS_P = re.compile(r"\byosys\b[^#\n]*?\s-p\s+(?=['\"])")


def quote_open_at_end(line, in_quote):
    """The quote character still open after `line`, given the one open before it."""
    for ch in line.rstrip("\n").rstrip("\\"):
        if in_quote:
            if ch == in_quote:
                in_quote = None
        elif ch in "'\"":
            in_quote = ch
    return in_quote


def violations(path):
    found = []
    quote = None
    for n, line in enumerate(path.read_text().splitlines(), 1):
        continued = line.rstrip().endswith("\\")
        if quote is None:
            m = YOSYS_P.search(line)
            if not m:
                continue
            quote = quote_open_at_end(line[m.end():], None)
        else:
            quote = quote_open_at_end(line, quote)
        if quote and continued:
            found.append(n)
        if not continued:
            quote = None
    return found


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

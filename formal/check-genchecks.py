#!/usr/bin/env python3
"""Fail if formal/genchecks-local.py has drifted from the pinned riscv-formal.

genchecks-local.py is a vendored copy of riscv-formal's checks/genchecks.py at
the SHA in formal/pin.mk. Exactly two things may differ: this repo's header
block, and `basedir` (spelled in two places -- the assignment and the isa-table
open -- which are one change, not two).

This repo previously ended up carrying a pre-2021 fork that had independently
reinvented an engine-selection option upstream already had, could not generate
ten upstream checks, and blocked the six csrc_* models outright. The repo
already solves this for the other vendored generated artifact -- `make
monitor-check` regenerates test/monitor.v from a clone at the pin and diffs it
-- and this is the same control for this file.

Rather than diffing and eyeballing the result, this UNDOES the two documented
edits and then requires byte equality with the clone. A residual diff is drift
by construction, and it is printed.

Usage: check-genchecks.py <upstream genchecks.py> <vendored genchecks-local.py>
"""

import difflib
import sys

# The header this repo prepends, delimited by text that exists in both files:
# everything between the shebang's trailing bare `#` and upstream's copyright
# line. Upstream has nothing there, so removing it should leave the two equal.
HEADER_START = '#!/usr/bin/env python3\n#\n'
HEADER_END = '# Copyright (C) 2017'

# The `basedir` change, in both the places it is spelled. Each must
# appear exactly once: a re-vendor that forgets one leaves genchecks resolving
# paths against the caller's cwd, which fails in a way that looks like a broken
# clone rather than a botched re-sync.
BASEDIR_EDITS = (
    (
        'basedir assignment',
        'basedir = os.path.abspath(os.path.join('
        'os.path.dirname(os.path.realpath(__file__)), "riscv-formal"))',
        'basedir = f"{os.getcwd()}/../.."',
    ),
    (
        'isa-table open',
        'open(f"{basedir}/insns/isa_{isa}.txt", "r")',
        'open(f"../../insns/isa_{isa}.txt", "r")',
    ),
)


def strip_header(text, path):
    """Remove this repo's header block, returning upstream-shaped text."""
    if not text.startswith(HEADER_START):
        raise SystemExit(
            f'{path}: does not start with the shebang and bare `#` both files\n'
            f'share, so the vendored header cannot be located. Re-sync from the\n'
            f'pin per this file\'s docstring.'
        )
    start = len(HEADER_START)
    end = text.find(HEADER_END, start)
    if end < 0:
        raise SystemExit(
            f'{path}: no {HEADER_END!r} line found. This is upstream\'s own\n'
            f'copyright banner; its absence means the file is not a copy of\n'
            f'checks/genchecks.py at all.'
        )
    header = text[start:end]
    # Guard the delete: only comment lines may be removed this way. Code hidden
    # in the header region would otherwise be stripped before the comparison and
    # so would never show up as drift.
    stray = [line for line in header.splitlines() if line and not line.startswith('#')]
    if stray:
        raise SystemExit(
            f'{path}: the header region above the copyright banner contains\n'
            f'non-comment lines, which this check will not silently drop:\n'
            + '\n'.join(f'    {line}' for line in stray)
        )
    return text[:start] + text[end:]


def main(argv):
    if len(argv) != 3:
        sys.stderr.write(f'usage: {argv[0]} <upstream> <vendored>\n')
        return 2

    upstream_path, vendored_path = argv[1], argv[2]
    with open(upstream_path) as handle:
        upstream = handle.read()
    with open(vendored_path) as handle:
        vendored = handle.read()

    normalized = strip_header(vendored, vendored_path)
    for name, ours, theirs in BASEDIR_EDITS:
        count = normalized.count(ours)
        if count != 1:
            sys.stderr.write(
                f'{argv[0]}: {vendored_path}: the documented "{name}" change appears\n'
                f'{count} time(s), expected 1. ADR-0031 says this is one of exactly two\n'
                f'permitted differences from the pin; re-apply it, or -- if upstream has\n'
                f'restructured this code -- update BASEDIR_EDITS in {argv[0]} and say so\n'
                f'in the ADR.\n'
            )
            return 1
        normalized = normalized.replace(ours, theirs)

    if normalized == upstream:
        print(f'genchecks-check: {vendored_path} matches {upstream_path} at the pin '
              f'(header + basedir only)')
        return 0

    sys.stderr.write(
        f'{argv[0]}: {vendored_path} has DRIFTED from {upstream_path}.\n'
        f'ADR-0031 permits exactly two differences -- this repo\'s header and the\n'
        f'`basedir` change -- and both are undone before the comparison below, so\n'
        f'everything shown is drift. Re-sync per the recipe in the vendored file\'s\n'
        f'header; do not hand-patch upstream code in place.\n\n'
    )
    sys.stderr.writelines(difflib.unified_diff(
        upstream.splitlines(keepends=True),
        normalized.splitlines(keepends=True),
        fromfile=f'{upstream_path} (at the pin)',
        tofile=f'{vendored_path} (header and basedir normalized away)',
    ))
    return 1


if __name__ == '__main__':
    sys.exit(main(sys.argv))

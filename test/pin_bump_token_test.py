#!/usr/bin/env python3
"""Grades the one property the pin-bump workflow's shape exists for: the step that
executes upstream code holds no credential.

`make test/monitor.v` runs upstream's own monitor/generate.py from a freshly cloned,
unreviewed SHA. If that step also carries GH_TOKEN, or the checkout left a token in
.git/config, one upstream commit reaches a token that can push here.

Usage: pin_bump_token_test.py [repo-root]
"""

import pathlib
import re
import sys

WORKFLOW = ".github/workflows/riscv-formal-pin-bump.yml"
UPSTREAM_CODE_SCRIPT = "bump-riscv-formal-pin.sh"


def steps(text):
    """The workflow's steps as (header, body) chunks, split on the '- ' item marker."""
    lines = text.splitlines()
    start = next(i for i, l in enumerate(lines) if re.match(r"^\s*steps:\s*$", l))
    chunks, cur = [], None
    for line in lines[start + 1:]:
        if re.match(r"^\s{6}- ", line):
            if cur is not None:
                chunks.append(cur)
            cur = [line]
        elif cur is not None:
            cur.append(line)
    if cur is not None:
        chunks.append(cur)
    return ["\n".join(c) for c in chunks]


def main(argv):
    root = pathlib.Path(argv[1] if len(argv) > 1 else pathlib.Path(__file__).parent.parent)
    path = root / WORKFLOW
    if not path.is_file():
        print(f"error: {WORKFLOW} is missing", file=sys.stderr)
        return 1
    text = path.read_text()

    found = [s for s in steps(text) if UPSTREAM_CODE_SCRIPT in s]
    if not found:
        print(
            f"error: no step in {WORKFLOW} runs {UPSTREAM_CODE_SCRIPT}. If it was renamed,"
            " rename it here too -- this check is what keeps a token away from it.",
            file=sys.stderr,
        )
        return 1

    failures = []
    for step in found:
        if "GH_TOKEN" in step:
            failures.append(
                f"the step running {UPSTREAM_CODE_SCRIPT} sets GH_TOKEN. That step executes"
                " upstream's generate.py; it must hold no credential."
            )

    checkout = [s for s in steps(text) if "actions/checkout" in s]
    if not checkout:
        failures.append(f"error: {WORKFLOW} has no actions/checkout step to grade")
    for step in checkout:
        if "persist-credentials: false" not in step:
            failures.append(
                "actions/checkout does not set persist-credentials: false, so a token stays"
                " in .git/config while upstream code runs."
            )

    if failures:
        for f in failures:
            print(f"*** {f}", file=sys.stderr)
        return 1

    print(f"pin-bump-token: {WORKFLOW} keeps credentials away from the upstream-code step.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

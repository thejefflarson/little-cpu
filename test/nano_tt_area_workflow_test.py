#!/usr/bin/env python3
"""Grades two shapes of the nano-tt-area-selfhosted workflow that a hand edit can break
silently -- each one still runs and still prints a summary, just the wrong one.

The stop-after-synthesis mode is resolved once into $NANO_TT_STOP_AFTER_SYNTHESIS so every
step reads the same value; a step that instead re-derives it from `inputs.` directly is
one GitHub Actions expression away from `false || 'true'`, which is `'true'` -- `||`
treats the string "false" as present and the boolean `false` as absent alike.

The sky130 PDK cache must be a restore step and a save step, not one combined
`actions/cache` step: a combined step only saves when the whole job succeeds, and this
workflow's job usually does not.

Usage: nano_tt_area_workflow_test.py [repo-root]
"""

import pathlib
import re
import sys

WORKFLOW = ".github/workflows/nano-tt-area-selfhosted.yml"


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
    found = steps(text)

    failures = []

    summary = [s for s in found if "stop after synthesis:" in s]
    if not summary:
        failures.append("no step echoes 'stop after synthesis:' into the job summary")
    for step in summary:
        if "inputs.stop_after_synthesis" in step:
            failures.append(
                "the summary step re-derives the mode from inputs.stop_after_synthesis"
                " instead of reading $NANO_TT_STOP_AFTER_SYNTHESIS -- a boolean input"
                " reads as the string \"false\", which `||` cannot tell from absent"
            )
        elif "NANO_TT_STOP_AFTER_SYNTHESIS" not in step:
            failures.append(
                "the summary step's stop-after-synthesis line names neither"
                " inputs.stop_after_synthesis nor $NANO_TT_STOP_AFTER_SYNTHESIS"
            )

    restore = [s for s in found if "actions/cache/restore@" in s]
    save = [s for s in found if "actions/cache/save@" in s]
    combined = [s for s in found if re.search(r"uses:\s*actions/cache@", s)]
    if not restore:
        failures.append("no actions/cache/restore step restores the sky130 PDK")
    if not save:
        failures.append("no actions/cache/save step saves the sky130 PDK")
    for step in save:
        if "if: always()" not in step:
            failures.append(
                "the actions/cache/save step has no if: always(), so it only saves the"
                " PDK when the rest of the job succeeded"
            )
    if combined:
        failures.append(
            "a combined actions/cache step is still present; it saves only on job"
            " success, which is what split it into restore + save in the first place"
        )

    if failures:
        for f in failures:
            print(f"*** {f}", file=sys.stderr)
        return 1

    print(f"nano-tt-area-workflow: {WORKFLOW} resolves its mode once and saves its PDK cache unconditionally.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

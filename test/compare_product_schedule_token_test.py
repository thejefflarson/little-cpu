#!/usr/bin/env python3
"""Grades this workflow's credential hygiene the same way
test/pin_bump_token_test.py grades the pin-bump workflow's: the token stays
out of any step that does not need it, and a `workflow_dispatch` cannot push a
branch or open a PR from anywhere but main.

Usage: compare_product_schedule_token_test.py [repo-root]
"""

import pathlib
import re
import sys

WORKFLOW = ".github/workflows/compare-product-schedule.yml"
PUBLISH_STEP_NAME = "Open a PR with the refreshed stamp"


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
    failures = []

    if not re.search(r"^\s*if:\s*github\.ref == 'refs/heads/main'\s*$", text, re.M):
        failures.append(
            "the job has no `if: github.ref == 'refs/heads/main'` guard, so a "
            "workflow_dispatch against any ref could push a branch and open a "
            "PR from it."
        )

    checkout = [s for s in steps(text) if "actions/checkout" in s]
    if not checkout:
        failures.append(f"{WORKFLOW} has no actions/checkout step to grade")
    for step in checkout:
        if "persist-credentials: false" not in step:
            failures.append(
                "actions/checkout does not set persist-credentials: false, so a "
                "token stays in .git/config for every step after it."
            )

    publish = [s for s in steps(text) if PUBLISH_STEP_NAME in s]
    if not publish:
        failures.append(
            f"no step named '{PUBLISH_STEP_NAME}' was found. If it was renamed, "
            "rename it here too -- this check is what keeps GH_TOKEN confined to it."
        )
    for step in publish:
        if "GH_TOKEN" not in step:
            failures.append(f"the '{PUBLISH_STEP_NAME}' step has no GH_TOKEN, so "
                            "it cannot push the branch or open the PR it exists for.")
        if "gh auth setup-git" not in step:
            failures.append(f"the '{PUBLISH_STEP_NAME}' step does not run "
                            "`gh auth setup-git`, so `git push` there has no "
                            "credential once persist-credentials is false.")

    other_steps = [s for s in steps(text) if PUBLISH_STEP_NAME not in s]
    for step in other_steps:
        if "GH_TOKEN" in step:
            name = step.splitlines()[0]
            failures.append(f"a step other than '{PUBLISH_STEP_NAME}' carries "
                            f"GH_TOKEN ({name.strip()}); confine it to the "
                            "one step that pushes and opens a PR.")

    if failures:
        for f in failures:
            print(f"*** {f}", file=sys.stderr)
        return 1

    print(f"compare-product-schedule-token: {WORKFLOW} confines its credential "
         "and restricts publishing to main.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

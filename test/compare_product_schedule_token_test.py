#!/usr/bin/env python3
"""Grades this workflow's credential hygiene the same way
test/pin_bump_token_test.py grades the pin-bump workflow's, across two jobs: `measure`
runs the build tools and holds a read-only token; `publish` holds the write scopes and
runs none of the tools. The token stays out of any step that does not need it, no job
that executes measurement tools carries a write scope, and a `workflow_dispatch` cannot
push a branch or open an issue from anywhere but main.

Usage: compare_product_schedule_token_test.py [repo-root]
"""

import pathlib
import re
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from workflow_steps import jobs, steps  # noqa: E402

WORKFLOW = ".github/workflows/compare-product-schedule.yml"
PUBLISH_STEP_NAME = "Publish the refreshed stamp"
MAIN_GUARD = re.compile(r"^\s*if:.*github\.ref == 'refs/heads/main'", re.M)
WRITE_SCOPE = re.compile(r"^\s+[a-z-]+:\s*write\b", re.M)
# The one make invocation the publish job may carry: it renders the committed comparison
# document from the stamp with python3 alone, and nothing else may ride beside the token.
ALLOWED_MAKE = re.compile(r"^\s*make compare-doc[ \t]*$", re.M)
MEASUREMENT_TOOLS = re.compile(r"\bmake\b|\./\.github/actions/|setup-riscv-gcc|setup-oss-cad-suite")


def code_lines(text):
    return "\n".join(l for l in text.splitlines() if not l.lstrip().startswith("#"))


def main(argv):
    root = pathlib.Path(argv[1] if len(argv) > 1 else pathlib.Path(__file__).parent.parent)
    path = root / WORKFLOW
    if not path.is_file():
        print(f"error: {WORKFLOW} is missing", file=sys.stderr)
        return 1
    text = path.read_text()
    failures = []

    by_name = jobs(text)
    for need in ("measure", "publish"):
        if need not in by_name:
            failures.append(f"{WORKFLOW} has no `{need}` job to grade.")
    if failures:
        for f in failures:
            print(f"*** {f}", file=sys.stderr)
        return 1
    measure, publish = by_name["measure"], by_name["publish"]

    if WRITE_SCOPE.search(code_lines(text.split("\njobs:")[0])):
        failures.append("the workflow-level permissions grant a write scope, so every "
                        "job inherits it; only the publish job may hold one.")

    for name, body in by_name.items():
        if not MAIN_GUARD.search(body):
            failures.append(
                f"the {name} job has no `if: github.ref == 'refs/heads/main'` guard, so a "
                "workflow_dispatch against any ref could measure or publish from it."
            )
        writes = WRITE_SCOPE.search(code_lines(body))
        if writes and name != "publish":
            failures.append(f"the {name} job holds a write-scoped token "
                            f"({writes.group(0).strip()}); only the publish job may.")
        if writes and MEASUREMENT_TOOLS.search(ALLOWED_MAKE.sub("", code_lines(body))):
            failures.append(f"the {name} job holds a write-scoped token and also runs "
                            "measurement tools (make, a toolchain setup action); a tool "
                            "could rewrite the publish script before it runs with the token.")
        checkout = [s for s in steps(body) if "actions/checkout" in s]
        if not checkout:
            failures.append(f"the {name} job has no actions/checkout step to grade")
        for step in checkout:
            if "persist-credentials: false" not in step:
                failures.append(
                    f"the {name} job's actions/checkout does not set "
                    "persist-credentials: false, so a token stays in .git/config for "
                    "every step after it."
                )

    if not re.search(r"^\s+contents:\s*read\s*$", code_lines(measure.split("steps:")[0]), re.M):
        failures.append("the measure job does not declare `contents: read`, so it runs "
                        "with whatever the repository default token carries.")
    if not WRITE_SCOPE.search(code_lines(publish)):
        failures.append("the publish job holds no write scope, so it cannot push the "
                        "branch or open the issue it exists for.")
    if not re.search(r"^\s+needs:\s*measure\s*$", publish, re.M):
        failures.append("the publish job does not `needs: measure`, so it could run "
                        "before the stamp exists.")
    if "needs.measure.outputs.moved == 'true'" not in publish.split("steps:")[0]:
        failures.append("the publish job is not conditional on the measure job's "
                        "`moved` output, so it would publish an unmoved stamp.")
    if not re.search(r"^\s+runs-on:\s*ubuntu-latest\s*$", publish, re.M):
        failures.append("the publish job does not run on ubuntu-latest, so the one job "
                        "holding write scopes shares the self-hosted pool with the tools.")
    if not ALLOWED_MAKE.search(code_lines(publish)):
        failures.append("the publish job never runs `make compare-doc`, so the refresh "
                        "branch would carry a stamp its committed comparison document "
                        "disagrees with.")
    publish_checkout = [s for s in steps(publish) if "actions/checkout" in s]
    if not any(re.search(r"^\s+ref:\s*\$\{\{\s*github\.sha\s*\}\}\s*$", s, re.M)
               for s in publish_checkout):
        failures.append("the publish job does not check out `${{ github.sha }}`, so it "
                        "would publish against whatever main holds by then.")
    if not re.search(r'^\s*run:\s*python3 soc/compare/product_verify\.py .*--sha "\$GITHUB_SHA"',
                     code_lines(publish), re.M):
        failures.append("the publish job never runs product_verify.py against "
                        "$GITHUB_SHA, so it would publish a stamp nothing re-derived.")
    if not [s for s in steps(publish) if "actions/download-artifact" in s]:
        failures.append("the publish job downloads no artifact, so it has no stamp to publish.")

    if "gh auth setup-git" in code_lines(text):
        failures.append("the workflow runs `gh auth setup-git`, which leaves a global "
                        "credential helper behind; pass the helper on the push instead.")

    publish_steps = [s for s in steps(publish) if PUBLISH_STEP_NAME in s]
    if not publish_steps:
        failures.append(
            f"no step named '{PUBLISH_STEP_NAME}' was found in the publish job. If it was "
            "renamed, rename it here too -- this check is what keeps GH_TOKEN confined to it."
        )
    for step in publish_steps:
        if "GH_TOKEN" not in step:
            failures.append(f"the '{PUBLISH_STEP_NAME}' step has no GH_TOKEN, so "
                            "it cannot push the branch or open the issue it exists for.")

    for name, body in by_name.items():
        for step in steps(body):
            if PUBLISH_STEP_NAME not in step and "GH_TOKEN" in step:
                failures.append(f"a step other than '{PUBLISH_STEP_NAME}' carries "
                                f"GH_TOKEN ({step.splitlines()[0].strip()} in {name}); "
                                "confine it to the one step that pushes and opens an issue.")

    if failures:
        for f in failures:
            print(f"*** {f}", file=sys.stderr)
        return 1

    print(f"compare-product-schedule-token: {WORKFLOW} confines its credential "
          "to a publish job that runs no measurement tools, and restricts both jobs to main.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

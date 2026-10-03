#!/usr/bin/env python3
"""Runs .github/scripts/publish-product-refresh.sh for real, so the shape of failure that
lost a 58-minute re-take -- `git commit -m ... -F ...`, which git refuses -- cannot recur
unnoticed. The script only runs once a week, on the pool, after `make compare-product`
finishes, so nothing else would execute it.

Copies the script into a throwaway git repository and runs it there with `gh` and
`git push` stubbed on PATH. Everything else -- `git config`, `git commit`,
`git checkout -b` -- runs for real, so a broken commit message, a missing branch or a
malformed issue body shows up the same way it would on the pool. Two runs: no refresh
open (the script must push a branch and open an issue) and one already open (it must do
neither).

Usage: compare_product_schedule_publish_test.py [repo-root]
"""

import json
import os
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile

SCRIPT = ".github/scripts/publish-product-refresh.sh"
TITLE = "Refresh the cross-core product stamp (2026-09-28)"
DIFF_LINE = "dhrystone: 1 -> 2 cycles/dhry"
PUSH_HELPER = "-c credential.helper= -c credential.helper=!gh auth git-credential"

GIT_STUB = """#!/bin/bash
args=("$@")
while [ "${args[0]:-}" = -c ]; do args=("${args[@]:2}"); done
case "${args[0]:-}" in
  push) printf '%s\\n' "$*" >> "$GIT_PUSH_LOG"; exit 0 ;;
esac
exec "$REAL_GIT" "$@"
"""

GH_STUB = """#!/bin/bash
printf '%s\\n' "$*" >> "$GH_LOG"
case "$1 $2" in
  "issue list")
    jq=""
    prev=""
    for a in "$@"; do
      [ "$prev" = --jq ] && jq=$a
      prev=$a
    done
    printf '%s' "${STUB_ISSUES_JSON:-[]}" | jq -r "$jq"
    exit $? ;;
  "pr list") printf '%s' "${STUB_OPEN_PRS:-}"; exit 0 ;;
  "issue create")
    title="" body="" prev=""
    for a in "$@"; do
      case "$prev" in
        --body-file) body=$a ;;
        --title) title=$a ;;
      esac
      prev=$a
    done
    [ -n "$body" ] && cp "$body" "$GH_ISSUE_BODY"
    printf '%s' "$title" > "$GH_ISSUE_TITLE"
    echo "https://github.com/o/r/issues/1"
    exit 0 ;;
  *)
    echo "stub gh: refusing '$1 $2'" >&2
    exit 9 ;;
esac
"""


def issue(number, login, is_bot, title=TITLE):
    return {"number": number, "title": title,
            "author": {"is_bot": is_bot, "login": login}}


def run_case(root, real_git, tmp, name, issues=(), open_prs="", args=None):
    """One run of the script in a fresh repository; returns the observations."""
    case = tmp / name
    bindir = case / "bin"
    bindir.mkdir(parents=True)
    (bindir / "git").write_text(GIT_STUB)
    (bindir / "gh").write_text(GH_STUB)
    for stub in ("git", "gh"):
        (bindir / stub).chmod(0o755)

    repo = case / "repo"
    (repo / "soc" / "compare").mkdir(parents=True)
    (repo / "docs").mkdir()
    (repo / ".github" / "scripts").mkdir(parents=True)
    shutil.copy(root / SCRIPT, repo / SCRIPT)

    out = case / "out"
    out.mkdir()
    (out / "product-diff.md").write_text(DIFF_LINE + "\n")

    logs = {k: case / f"{k}.txt" for k in
            ("gh-log", "push-log", "issue-body", "issue-title", "summary")}
    for k in ("gh-log", "push-log", "summary"):
        logs[k].write_text("")

    # Under a git hook GIT_DIR and GIT_INDEX_FILE are exported, and inheriting them
    # would point every git call here at the real repository.
    env = {k: v for k, v in os.environ.items() if not k.startswith("GIT_")}
    env.update({
        "GIT_CONFIG_GLOBAL": os.devnull,
        "GIT_CONFIG_NOSYSTEM": "1",
        "HOME": str(case),
        "PATH": f"{bindir}{os.pathsep}{env.get('PATH', '')}",
        "REAL_GIT": real_git,
        "GH_TOKEN": "test-token",
        "GH_LOG": str(logs["gh-log"]),
        "GH_ISSUE_BODY": str(logs["issue-body"]),
        "GH_ISSUE_TITLE": str(logs["issue-title"]),
        "GIT_PUSH_LOG": str(logs["push-log"]),
        "GITHUB_STEP_SUMMARY": str(logs["summary"]),
        "GITHUB_RUN_ID": "4242",
        "GITHUB_REPOSITORY": "o/r",
        "STUB_ISSUES_JSON": json.dumps(list(issues)),
        "STUB_OPEN_PRS": open_prs,
    })

    def real(*a):
        return subprocess.run([real_git, *a], check=True, cwd=repo, env=env,
                              capture_output=True, text=True).stdout

    real("init", "-q", "-b", "main")
    real("config", "user.email", "committer@example.com")
    real("config", "user.name", "Committer")
    (repo / "soc" / "compare" / "product.json").write_text('{"pairs": {"dhrystone": 1}}\n')
    (repo / "docs" / "comparison.md").write_text("old render\n")
    (repo / "README.md").write_text("tracked\n")
    real("add", "-A")
    real("commit", "-q", "-m", "initial")
    (repo / "soc" / "compare" / "product.json").write_text('{"pairs": {"dhrystone": 2}}\n')
    (repo / "docs" / "comparison.md").write_text("new render\n")
    (repo / "README.md").write_text("a stray edit the script must not commit\n")

    result = subprocess.run(
        ["bash", str(repo / SCRIPT)] + ([str(out)] if args is None else args),
        cwd=repo, env=env, capture_output=True, text=True)
    return {"result": result, "real": real, "logs": logs}


def read(p):
    return p.read_text() if p.is_file() else ""


def check_publish(case):
    failures = []
    result, real, logs = case["result"], case["real"], case["logs"]
    if result.returncode != 0:
        return [f"the script exited {result.returncode}: {result.stderr.strip()}"]
    pushes = read(logs["push-log"]).splitlines()
    m = re.fullmatch(re.escape(PUSH_HELPER) + r" push --quiet origin (compare-product/refresh-\d{8}-4242)",
                     pushes[0]) if len(pushes) == 1 else None
    if not m:
        return [f"the script did not push exactly one refresh branch to origin: {pushes!r}"]
    branch = m.group(1)
    if "issue create" not in read(logs["gh-log"]):
        failures.append("the script never opened an issue")
    if "pr create" in read(logs["gh-log"]):
        failures.append("the script opened a pull request with the workflow's own token")
    if not read(logs["issue-title"]).startswith("Refresh the cross-core product stamp ("):
        failures.append("the issue was not given the expected title")
    body = read(logs["issue-body"])
    if DIFF_LINE not in body:
        failures.append("the issue body does not carry the measured diff")
    if "soc/compare/CYCLE_FLOOR" not in body:
        failures.append("the issue body does not say CYCLE_FLOOR is updated by hand")
    if f"https://github.com/o/r/compare/main...{branch}?expand=1" not in body:
        failures.append("the issue body does not link the compare page for the pushed branch")
    if read(logs["summary"]).strip() == "":
        failures.append("the script wrote nothing to GITHUB_STEP_SUMMARY")
    subject = real("log", "-1", "--pretty=%s").strip()
    if subject != "Refresh the cross-core product stamp":
        failures.append(f"the commit's subject line is {subject!r}")
    committed = real("show", "--name-only", "--format=", "HEAD").split()
    if sorted(committed) != ["docs/comparison.md", "soc/compare/product.json"]:
        failures.append("the commit touches files other than soc/compare/product.json and "
                        f"docs/comparison.md: {committed!r}")
    if DIFF_LINE not in real("log", "-1", "--pretty=%b"):
        failures.append("the commit message does not carry the measured diff")
    if real("config", "user.name").strip() != "github-actions[bot]":
        failures.append("git config user.name is not 'github-actions[bot]'")
    if real("config", "user.email").strip() != "41898282+github-actions[bot]@users.noreply.github.com":
        failures.append("git config user.email is unexpected")
    return failures


def check_skip(case, label):
    failures = []
    result, real, logs = case["result"], case["real"], case["logs"]
    if result.returncode != 0:
        return [f"{label}: the script exited {result.returncode}: {result.stderr.strip()}"]
    if read(logs["push-log"]).strip():
        failures.append(f"{label}: the script pushed a second refresh branch")
    if "issue create" in read(logs["gh-log"]):
        failures.append(f"{label}: the script opened a second issue")
    if real("rev-list", "--count", "HEAD").strip() != "1":
        failures.append(f"{label}: the script committed anyway")
    if "already open" not in read(logs["summary"]):
        failures.append(f"{label}: the script did not say a refresh is already open")
    return failures


def main(argv):
    root = pathlib.Path(argv[1] if len(argv) > 1 else pathlib.Path(__file__).parent.parent)
    if not (root / SCRIPT).is_file():
        print(f"error: {SCRIPT} is missing", file=sys.stderr)
        return 1
    real_git = shutil.which("git")
    if real_git is None:
        print("error: no real git on PATH to back the stub", file=sys.stderr)
        return 1

    failures = []
    with tempfile.TemporaryDirectory(prefix="compare-product-publish-test.") as raw:
        tmp = pathlib.Path(raw)
        failures += check_publish(run_case(root, real_git, tmp, "fresh"))
        for label, login in (("gh's app/ login", "app/github-actions"),
                             ("the [bot] login", "github-actions[bot]")):
            failures += check_skip(
                run_case(root, real_git, tmp, f"open-issue-{login[0]}",
                         issues=[issue(7, login, True)]),
                f"an open refresh issue from the bot ({label})")
        failures += check_publish(
            run_case(root, real_git, tmp, "human-issue",
                     issues=[issue(7, "mallory", False)]))
        failures += check_publish(
            run_case(root, real_git, tmp, "human-lookalike-bot",
                     issues=[issue(7, "app/mallory-github-actions", True),
                             issue(8, "github-actions-evil", False)]))
        failures += check_publish(
            run_case(root, real_git, tmp, "bot-other-title",
                     issues=[issue(7, "app/github-actions", True, "Unrelated")]))
        failures += check_skip(
            run_case(root, real_git, tmp, "open-pr", open_prs="#8\n"),
            "an open refresh pull request")
        usage = run_case(root, real_git, tmp, "usage", args=[])["result"]
        if usage.returncode != 2:
            failures.append(f"no argument exited {usage.returncode}, not 2")

    if failures:
        for f in failures:
            print(f"*** {f}", file=sys.stderr)
        return 1
    print("compare-product-schedule-publish: the script pushed one refresh branch, opened one "
          "issue with the compare link, stood down when the bot's own refresh was already open, and "
          "ignored an open issue with the same title from anyone else.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

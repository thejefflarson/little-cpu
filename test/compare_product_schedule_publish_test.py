#!/usr/bin/env python3
"""Runs the schedule workflow's publish step for real, so the shape of failure
that lost a 58-minute re-take -- `git commit -m ... -F ...`, which git refuses --
cannot recur unnoticed. Nothing on `make test`'s path had ever executed this step;
it only runs once a week, on the pool, after `make compare-product` finishes.

Extracts the `run:` block of the step named PUBLISH_STEP_NAME out of the workflow
(the step-chunking is test/compare_product_schedule_token_test.py's, duplicated
here the way that file is itself duplicated from test/pin_bump_token_test.py) and
runs it as a shell script against a throwaway git repository, with `gh` and
`git push` stubbed on PATH. Everything else -- `git config`, `git commit`,
`git checkout -b` -- runs for real against that repository, so a broken commit
message, a missing branch, or a malformed PR body shows up the same way it would
on the pool.

Usage: compare_product_schedule_publish_test.py [repo-root]
"""

import os
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile

WORKFLOW = ".github/workflows/compare-product-schedule.yml"
PUBLISH_STEP_NAME = "Open a PR with the refreshed stamp"

GIT_STUB = """#!/bin/bash
if [ "$1" = "push" ]; then
  printf 'push %s\\n' "$*" >> "$GIT_PUSH_LOG"
  exit 0
fi
exec "$REAL_GIT" "$@"
"""

GH_STUB = """#!/bin/bash
printf '%s\\n' "$*" >> "$GH_LOG"
case "$1 $2" in
  "auth setup-git")
    exit 0 ;;
  "pr create")
    body="" head="" title="" prev=""
    for a in "$@"; do
      case "$prev" in
        --body-file) body=$a ;;
        --head) head=$a ;;
        --title) title=$a ;;
      esac
      prev=$a
    done
    [ -n "$body" ] && cp "$body" "$GH_PR_BODY"
    printf '%s' "$head" > "$GH_PR_HEAD"
    printf '%s' "$title" > "$GH_PR_TITLE"
    echo "https://github.com/o/r/pull/1"
    exit 0 ;;
  *)
    echo "stub gh: refusing '$1 $2'" >&2
    exit 9 ;;
esac
"""


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


def extract_run_script(step_text):
    """The body of a `run: |` block scalar, dedented to column zero. YAML-naive
    on purpose: it only has to agree with this one workflow's own indentation."""
    out, indent, in_run = [], None, False
    for line in step_text.splitlines():
        if not in_run:
            if re.match(r"^\s*run:\s*\|\s*$", line):
                in_run = True
            continue
        if line.strip() == "":
            out.append("")
            continue
        cur = len(line) - len(line.lstrip(" "))
        if indent is None:
            indent = cur
        if cur < indent:
            break
        out.append(line[indent:])
    return "\n".join(out)


def main(argv):
    root = pathlib.Path(argv[1] if len(argv) > 1 else pathlib.Path(__file__).parent.parent)
    path = root / WORKFLOW
    if not path.is_file():
        print(f"error: {WORKFLOW} is missing", file=sys.stderr)
        return 1
    text = path.read_text()

    publish = [s for s in steps(text) if PUBLISH_STEP_NAME in s]
    if not publish:
        print(
            f"error: no step named '{PUBLISH_STEP_NAME}' was found in {WORKFLOW}. "
            "If it was renamed, rename it here too.", file=sys.stderr)
        return 1
    if len(publish) > 1:
        print(f"error: more than one step named '{PUBLISH_STEP_NAME}' was found", file=sys.stderr)
        return 1

    script = extract_run_script(publish[0])
    if not script.strip():
        print(f"error: the '{PUBLISH_STEP_NAME}' step has no `run:` block to extract", file=sys.stderr)
        return 1

    real_git = shutil.which("git")
    if real_git is None:
        print("error: no real git on PATH to back the stub", file=sys.stderr)
        return 1

    with tempfile.TemporaryDirectory(prefix="compare-product-publish-test.") as raw_tmp:
        tmp = pathlib.Path(raw_tmp)
        bindir = tmp / "bin"
        bindir.mkdir()
        (bindir / "git").write_text(GIT_STUB)
        (bindir / "git").chmod(0o755)
        (bindir / "gh").write_text(GH_STUB)
        (bindir / "gh").chmod(0o755)

        repo = tmp / "repo"
        (repo / "soc" / "compare").mkdir(parents=True)

        def real(*args):
            return subprocess.run([real_git, *args], check=True, cwd=repo,
                                   capture_output=True, text=True).stdout

        real("init", "-q", "-b", "main")
        real("config", "user.email", "committer@example.com")
        real("config", "user.name", "Committer")
        (repo / "soc" / "compare" / "product.json").write_text('{"pairs": {"dhrystone": 1}}\n')
        real("add", "-A")
        real("commit", "-q", "-m", "initial")
        (repo / "soc" / "compare" / "product.json").write_text('{"pairs": {"dhrystone": 2}}\n')

        diff_path = pathlib.Path("/tmp/product-diff.md")
        commit_msg_path = pathlib.Path("/tmp/commit-message.md")
        pr_body_path = pathlib.Path("/tmp/pr-body.md")
        diff_path.write_text("dhrystone: 1 -> 2 cycles/dhry\n")
        for stale in (commit_msg_path, pr_body_path):
            stale.unlink(missing_ok=True)

        script_path = tmp / "publish.sh"
        script_path.write_text(script)

        gh_log = tmp / "gh.log"
        git_push_log = tmp / "git-push.log"
        pr_body = tmp / "pr-body-seen.md"
        pr_head = tmp / "pr-head.txt"
        pr_title = tmp / "pr-title.txt"
        step_summary = tmp / "step-summary.md"
        for f in (gh_log, git_push_log, step_summary):
            f.write_text("")

        env = dict(os.environ)
        env["PATH"] = f"{bindir}{os.pathsep}{env.get('PATH', '')}"
        env["REAL_GIT"] = real_git
        env["GH_TOKEN"] = "test-token"
        env["GH_LOG"] = str(gh_log)
        env["GH_PR_BODY"] = str(pr_body)
        env["GH_PR_HEAD"] = str(pr_head)
        env["GH_PR_TITLE"] = str(pr_title)
        env["GIT_PUSH_LOG"] = str(git_push_log)
        env["GITHUB_STEP_SUMMARY"] = str(step_summary)

        try:
            result = subprocess.run(["bash", str(script_path)], cwd=repo, env=env,
                                     capture_output=True, text=True)
        finally:
            for stale in (diff_path, commit_msg_path, pr_body_path):
                stale.unlink(missing_ok=True)

        if result.returncode != 0:
            print(f"error: the '{PUBLISH_STEP_NAME}' step exited {result.returncode}",
                  file=sys.stderr)
            print(result.stdout, file=sys.stderr)
            print(result.stderr, file=sys.stderr)
            return 1

        failures = []
        if not pr_head.is_file() or not re.fullmatch(
                r"compare-product/refresh-\d{8}", pr_head.read_text()):
            failures.append("gh pr create was not given the expected --head branch name")
        if not pr_title.is_file() or not pr_title.read_text().startswith(
                "Refresh the cross-core product stamp ("):
            failures.append("gh pr create was not given the expected --title")
        if not pr_body.is_file() or "dhrystone: 1 -> 2 cycles/dhry" not in pr_body.read_text():
            failures.append("gh pr create's --body-file does not carry the measured diff")
        if "auth setup-git" not in gh_log.read_text():
            failures.append("the step never ran `gh auth setup-git`")
        if not git_push_log.is_file() or "compare-product/refresh-" not in git_push_log.read_text():
            failures.append("the step never pushed the refresh branch")
        if step_summary.read_text().strip() == "":
            failures.append("the step wrote nothing to GITHUB_STEP_SUMMARY")

        commit_subject = real("log", "-1", "--pretty=%s").strip()
        if commit_subject != "Refresh the cross-core product stamp":
            failures.append(f"the commit's subject line is {commit_subject!r}, "
                             "not 'Refresh the cross-core product stamp'")
        commit_body = real("log", "-1", "--pretty=%b")
        if "dhrystone: 1 -> 2 cycles/dhry" not in commit_body:
            failures.append("the commit message does not carry the measured diff")

        committer_name = real("config", "user.name").strip()
        if committer_name != "github-actions[bot]":
            failures.append(f"git config user.name is {committer_name!r}, "
                             "not 'github-actions[bot]'")
        committer_email = real("config", "user.email").strip()
        if committer_email != "41898282+github-actions[bot]@users.noreply.github.com":
            failures.append(f"git config user.email is {committer_email!r}, unexpected")

        if failures:
            for f in failures:
                print(f"*** {f}", file=sys.stderr)
            return 1

    print(f"compare-product-schedule-publish: '{PUBLISH_STEP_NAME}' reached "
          "gh pr create with the expected branch, title and body file.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

#!/bin/bash
# Opens an issue naming a pin-bump branch that has already been committed and
# pushed. A human opens the pull request from that branch.
#
# It must not open the pull request itself. GitHub fires no `pull_request` event
# for a PR the Actions default GITHUB_TOKEN opens, and no `push` for a commit it
# pushes, so such a PR gets no checks and branch protection blocks it forever.
# `gh workflow run` does not rescue it: those check runs land on the commit and
# show up in `gh pr checks`, but never in the pull request's statusCheckRollup,
# which is what the merge gate reads. Do not put that call back.
#
# Usage: propose-pin-bump.sh <branch> <title> <body-file>
set -euo pipefail

if [ "$#" -ne 3 ]; then
  echo "usage: propose-pin-bump.sh <branch> <title> <body-file>" >&2
  exit 2
fi

BRANCH=$1
TITLE=$2
BODY_FILE=$3

if [ ! -r "$BODY_FILE" ]; then
  echo "propose-pin-bump.sh: cannot read body file '$BODY_FILE'" >&2
  exit 2
fi

ISSUE_BODY=$(mktemp "${TMPDIR:-/tmp}/pin-bump-issue.XXXXXX") || {
  echo "propose-pin-bump.sh: could not create a temporary file" >&2
  exit 1
}
trap 'rm -f "$ISSUE_BODY"' EXIT

{
  echo "The work is done and is on \`$BRANCH\`: the pin is bumped and"
  echo "\`test/monitor.v\` is regenerated against it. **Open a pull request from"
  echo "that branch** and the checks will run on it normally."
  if [ -n "${GITHUB_REPOSITORY:-}" ]; then
    echo
    echo "${GITHUB_SERVER_URL:-https://github.com}/$GITHUB_REPOSITORY/compare/main...$BRANCH?expand=1"
  fi
  echo
  echo "This is an issue rather than a pull request because a PR opened by the"
  echo "workflow itself would get no CI and could never merge. Opening it under"
  echo "a human account is what makes the checks count."
  echo
  cat "$BODY_FILE"
} > "$ISSUE_BODY"

URL=$(gh issue create --title "$TITLE" --body-file "$ISSUE_BODY")
echo "opened issue $URL"

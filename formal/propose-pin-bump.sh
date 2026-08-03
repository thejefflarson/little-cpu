#!/bin/bash
# Proposes a pin bump whose branch has already been committed and pushed.
# Opens a pull request when PIN_BUMP_TOKEN is set, and an issue naming the
# branch when it is not.
#
# The token is what makes the pull request mergeable, and this is measured on
# this repository rather than assumed. GitHub fires no `pull_request` event for
# a PR the Actions default GITHUB_TOKEN opens, and no `push` event for a commit
# it pushes, so such a PR collects no checks at all and branch protection blocks
# it forever. Asking for a run with `gh workflow run` does not rescue it: those
# check runs land on the commit and are visible in `gh pr checks`, but they
# never enter the pull request's statusCheckRollup, which is the only thing the
# merge gate reads. Do not put that call back.
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

if [ -n "${PIN_BUMP_TOKEN:-}" ]; then
  URL=$(GH_TOKEN="$PIN_BUMP_TOKEN" GITHUB_TOKEN="$PIN_BUMP_TOKEN" \
    gh pr create --title "$TITLE" --body-file "$BODY_FILE" \
    --head "$BRANCH" --base main)
  echo "opened pull request $URL"
  exit 0
fi

COMPARE=""
if [ -n "${GITHUB_REPOSITORY:-}" ]; then
  SERVER=${GITHUB_SERVER_URL:-https://github.com}
  COMPARE="$SERVER/$GITHUB_REPOSITORY/compare/main...$BRANCH?expand=1"
fi

ISSUE_BODY=$(mktemp "${TMPDIR:-/tmp}/pin-bump-issue.XXXXXX") || {
  echo "propose-pin-bump.sh: could not create a temporary file" >&2
  exit 1
}
trap 'rm -f "$ISSUE_BODY"' EXIT

{
  echo "No PIN_BUMP_TOKEN secret is configured, so this run pushed the branch"
  echo "and opened this issue instead of a pull request. A PR opened with the"
  echo "Actions default GITHUB_TOKEN gets no CI and can never merge."
  echo
  echo "The work is done and is on \`$BRANCH\`: the pin is bumped and"
  echo "\`test/monitor.v\` is regenerated against it. Open a pull request from"
  echo "that branch yourself and the checks will run normally."
  if [ -n "$COMPARE" ]; then
    echo
    echo "$COMPARE"
  fi
  echo
  echo "To make future bumps open their own pull request, add a repository"
  echo "secret named \`PIN_BUMP_TOKEN\` holding a token that is not the Actions"
  echo "default one -- a fine-grained PAT or a GitHub App installation token"
  echo "with contents and pull-requests write on this repository."
  echo
  cat "$BODY_FILE"
} > "$ISSUE_BODY"

URL=$(gh issue create --title "$TITLE" --body-file "$ISSUE_BODY")
echo "opened issue $URL"

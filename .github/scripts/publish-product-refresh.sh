#!/bin/bash
# Pushes the refreshed stamp to a branch and opens an issue linking its compare page, since
# a PR opened with the workflow's own token gets no CI. Skips when a refresh is open.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."

if [ "$#" -ne 1 ]; then
  echo "usage: publish-product-refresh.sh <out-dir>" >&2
  exit 2
fi

OUT_DIR=$1
DIFF="$OUT_DIR/product-diff.md"
[ -r "$DIFF" ] || {
  echo "publish-product-refresh.sh: cannot read $DIFF" >&2
  exit 2
}

TITLE_PREFIX="Refresh the cross-core product stamp"
BRANCH_PREFIX="compare-product/refresh-"
BRANCH="$BRANCH_PREFIX$(date -u +%Y%m%d)-${GITHUB_RUN_ID:-local}"
TITLE="$TITLE_PREFIX ($(date -u +%Y-%m-%d))"

git_push() {
  git -c credential.helper= -c 'credential.helper=!gh auth git-credential' push --quiet "$@"
}

summary() {
  [ -z "${GITHUB_STEP_SUMMARY:-}" ] || cat >> "$GITHUB_STEP_SUMMARY"
}

open_issues=$(gh issue list --state open --limit 200 --json number,title \
  --jq ".[] | select(.title | startswith(\"$TITLE_PREFIX\")) | \"#\\(.number)\"")
open_prs=$(gh pr list --state open --limit 200 --json number,headRefName,isCrossRepository \
  --jq ".[] | select((.isCrossRepository | not) and (.headRefName | startswith(\"$BRANCH_PREFIX\"))) | \"#\\(.number)\"")
if [ -n "$open_issues$open_prs" ]; then
  {
    echo "### Cross-core product re-take: a refresh is already open"
    echo
    echo "Open: $(echo $open_issues $open_prs). Not pushing a second branch; the new"
    echo "measurement is in this run's artifact."
    echo
    cat "$DIFF"
  } | summary
  exit 0
fi

git config user.name "github-actions[bot]"
git config user.email "41898282+github-actions[bot]@users.noreply.github.com"
git checkout -b "$BRANCH"
git add soc/compare/product.json
{
  echo "$TITLE_PREFIX"
  echo
  cat "$DIFF"
} > "$OUT_DIR/commit-message.md"
git commit --quiet -F "$OUT_DIR/commit-message.md"
git_push origin "$BRANCH"

{
  echo "The refreshed stamp is committed on \`$BRANCH\`. **Open a pull request from"
  echo "that branch** and the checks will run on it normally."
  if [ -n "${GITHUB_REPOSITORY:-}" ]; then
    echo
    echo "${GITHUB_SERVER_URL:-https://github.com}/$GITHUB_REPOSITORY/compare/main...$BRANCH?expand=1"
  fi
  echo
  echo "This is an issue rather than a pull request because a PR opened by the"
  echo "workflow itself would get no CI and could never merge."
  echo
  cat "$DIFF"
  echo
  echo "Machine-refreshed by \`.github/workflows/compare-product-schedule.yml\`."
  echo "The branch only updates \`soc/compare/product.json\`. CLAUDE.md's cross-core"
  echo "paragraph and any ADR that quotes this pair's numbers still need a person to"
  echo "read this diff and decide whether the prose needs updating."
} > "$OUT_DIR/issue-body.md"

# A pushed branch with no issue would be invisible to everyone.
if ! gh issue create --title "$TITLE" --body-file "$OUT_DIR/issue-body.md"; then
  echo "could not open the issue; removing $BRANCH so a later run retries" >&2
  git_push --delete origin "$BRANCH" || true
  exit 1
fi

{
  echo "### Cross-core product re-take: pushed \`$BRANCH\` and opened an issue"
  echo
  cat "$DIFF"
} | summary

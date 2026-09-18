#!/bin/bash
# Reads the GitHub API, so it is non-hermetic and does not run under `make test`; see
# CLAUDE.md's ADR pointer for how this pairs with test/adr_numbering_test.sh.
set -euo pipefail

: "${REPO:?REPO must be set to owner/repo}"
: "${PR_NUMBER:?PR_NUMBER must be set to the current pull request number}"

added_numbers() {
  gh api "repos/$REPO/pulls/$1/files" --paginate \
    -q '.[] | select(.status == "added" or .status == "renamed") | .filename' \
    | { grep -E '^docs/adr/[0-9]{4}-.*\.md$' || true; } \
    | sed -E 's#^docs/adr/([0-9]{4})-.*#\1#' \
    | sort -u
}

mine=$(added_numbers "$PR_NUMBER")
if [ -z "$mine" ]; then
  echo "this pull request adds no docs/adr/NNNN-*.md file; nothing to check."
  exit 0
fi

echo "#$PR_NUMBER adds:"
echo "$mine" | sed -e 's/^/  /'

others=$(gh api "repos/$REPO/pulls" --paginate -X GET -f state=open -q '.[].number' \
  | grep -vx "$PR_NUMBER" || true)

rc=0
while IFS= read -r other; do
  [ -n "$other" ] || continue
  other_numbers=$(added_numbers "$other")
  [ -n "$other_numbers" ] || continue
  collision=$(comm -12 <(printf '%s\n' "$mine") <(printf '%s\n' "$other_numbers"))
  [ -n "$collision" ] || continue
  branch=$(gh api "repos/$REPO/pulls/$other" -q '.head.ref')
  while IFS= read -r n; do
    [ -n "$n" ] || continue
    echo "::error::ADR number $n is claimed by this pull request AND by #$other (branch $branch)" >&2
    rc=1
  done <<< "$collision"
done <<< "$others"

if [ "$rc" -ne 0 ]; then
  echo >&2
  echo "Two open pull requests claim the same ADR number. Renumber one of them --" >&2
  echo "test/adr_numbering_test.sh only ever reads a single tree, so it cannot see" >&2
  echo "this by itself. See CLAUDE.md's ADR pointer for how to pick a number that" >&2
  echo "clears every open branch, not only main." >&2
  exit 1
fi

echo "no other open pull request claims the number(s) above."

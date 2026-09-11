#!/bin/bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

UPSTREAM_URL="https://github.com/YosysHQ/riscv-formal.git"
UPSTREAM_REPO="YosysHQ/riscv-formal"
MIN_AGE_DAYS=${PIN_BUMP_MIN_AGE_DAYS:-7}

emit() {
  printf '%s\n' "$1"
  [ -n "${GITHUB_OUTPUT:-}" ] && printf '%s\n' "$1" >> "$GITHUB_OUTPUT"
  return 0
}

stop() {
  echo "no bump: $1"
  [ -n "${GITHUB_STEP_SUMMARY:-}" ] && printf 'pin bump did not run: %s\n' "$1" >> "$GITHUB_STEP_SUMMARY"
  emit "proceed=no"
  exit 0
}

PIN_SHA=$(python3 -c "
import re, pathlib
text = pathlib.Path('formal/pin.mk').read_text()
m = re.search(r'override RISCV_FORMAL_SHA := ([0-9a-f]{40})', text)
assert m, 'could not find RISCV_FORMAL_SHA in formal/pin.mk'
print(m.group(1))
")

UPSTREAM_SHA=$(git ls-remote "$UPSTREAM_URL" HEAD | cut -f1)
# No `head -1`, and =~ over the whole string: either would accept a first line that looks like a SHA.
if ! [[ $UPSTREAM_SHA =~ ^[0-9a-f]{40}$ ]]; then
  echo "upstream HEAD for $UPSTREAM_URL is not one 40-hex SHA: '$UPSTREAM_SHA'" >&2
  exit 1
fi

echo "pinned:   $PIN_SHA"
echo "upstream: $UPSTREAM_SHA"

[ "$PIN_SHA" = "$UPSTREAM_SHA" ] && stop "the pin is current"

COMMITTED=$(gh api "repos/$UPSTREAM_REPO/commits/$UPSTREAM_SHA" --jq '.commit.committer.date' 2>/dev/null || true)
[ -n "$COMMITTED" ] || {
  echo "could not read the commit date for $UPSTREAM_SHA from $UPSTREAM_REPO" >&2
  exit 1
}
AGE_DAYS=$(python3 -c "
import datetime, sys
c = datetime.datetime.fromisoformat(sys.argv[1].replace('Z', '+00:00'))
print(int((datetime.datetime.now(datetime.timezone.utc) - c).total_seconds() // 86400))
" "$COMMITTED")
echo "committed: $COMMITTED (${AGE_DAYS}d ago)"
[ "$AGE_DAYS" -ge "$MIN_AGE_DAYS" ] ||
  stop "upstream HEAD is ${AGE_DAYS}d old, under the ${MIN_AGE_DAYS}d floor a compromised upstream commit has to outlive"

BRANCH="riscv-formal-pin/bump-${UPSTREAM_SHA:0:12}"
# A branch here needs write access. Issue and PR titles are public, and were a way to switch this off.
if git ls-remote --exit-code --heads origin "$BRANCH" >/dev/null 2>&1; then
  stop "$BRANCH already exists on this repository"
fi

emit "pin_sha=$PIN_SHA"
emit "upstream_sha=$UPSTREAM_SHA"
emit "branch=$BRANCH"
emit "title=Bump riscv-formal pin to ${UPSTREAM_SHA:0:12}"
emit "proceed=yes"

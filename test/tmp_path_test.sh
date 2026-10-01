#!/bin/bash
# Refuses a literal /tmp/ path in a tracked Makefile or shell script: scratch belongs under
# the gitignored build/. mktemp under ${TMPDIR:-/tmp} is for files nobody reads later.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=${1:-$(cd "$HERE/.." && pwd)}

if [ ! -d "$REPO" ]; then
  echo "error: '$REPO' is not a directory, so there is nothing to scan." >&2
  exit 1
fi

allow_paths() {
  sed -e 's/#.*//' -e 's/[[:space:]]*$//' -e '/^$/d' <<'PATHS'
# Probe fixtures name /tmp/ because it is the string under test.
test/probe_gates.sh

test/tmp_path_test.sh
PATHS
}

tmp=$(mktemp -d "${TMPDIR:-/tmp}/littlecpu-tmppath.XXXXXX") || {
  echo "error: could not create a temporary directory under ${TMPDIR:-/tmp}." >&2
  exit 1
}
trap 'rm -rf "$tmp"' EXIT

allow_paths > "$tmp/allow"
if [ ! -s "$tmp/allow" ]; then
  echo "error: the allow-list is empty, so the probe fixtures would go red." >&2
  exit 1
fi

if ! git -C "$REPO" ls-files -z > "$tmp/files" 2>/dev/null || [ ! -s "$tmp/files" ]; then
  echo "error: cannot enumerate any tracked files under $REPO. This check reads" >&2
  echo "git's index, because what it guards is a path arriving in a commit; a" >&2
  echo "tree git cannot list is a scan of nothing reporting green." >&2
  exit 1
fi

tr '\0' '\n' < "$tmp/files" | { grep -E '(^|/)Makefile$|\.(mk|sh)$' || true; } | tr '\n' '\0' > "$tmp/scripts"
if [ ! -s "$tmp/scripts" ]; then
  echo "error: no tracked Makefile or shell script under $REPO, so there is nothing to scan." >&2
  exit 1
fi

hits=$( (cd "$REPO" && xargs -0 grep -nIF -e '/tmp/' -- /dev/null < "$tmp/scripts") || true)

rc=0
: > "$tmp/unexpected"
: > "$tmp/covered"
while IFS= read -r hit; do
  [ -n "$hit" ] || continue
  path=${hit%%:*}
  if grep -qxF -- "$path" "$tmp/allow"; then
    printf '%s\n' "$path" >> "$tmp/covered"
  else
    printf '%s\n' "$hit" >> "$tmp/unexpected"
  fi
done <<< "$hits"

if [ -s "$tmp/unexpected" ]; then
  rc=1
  echo "error: a literal /tmp/ path appears in a tracked build script:" >&2
  sed -e 's|^|  |' "$tmp/unexpected" >&2
  echo >&2
  echo "A fixed /tmp name collides across worktrees and vanishes when read later." >&2
  echo "Write scratch under \$(BUILD)/ (build/ in a script), which is gitignored and" >&2
  echo "per-worktree; use mktemp only for a file nothing reads after the script ends." >&2
  echo "If the string is data a checker must see, add the file to the allow-list in" >&2
  echo "test/tmp_path_test.sh with its reason." >&2
fi

while IFS= read -r entry; do
  if ! grep -qxF -- "$entry" "$tmp/covered"; then
    rc=1
    echo >&2
    echo "error: the allow-list exempts $entry, and no literal /tmp/ appears there" >&2
    echo "any more. Delete the entry; an exemption kept past its reason waves the next" >&2
    echo "one through, which is why this comparison runs both ways." >&2
  fi
done < "$tmp/allow"

[ "$rc" -eq 0 ] || exit 1

entries=$(wc -l < "$tmp/allow" | tr -d ' ')
scripts=$(tr -cd '\0' < "$tmp/scripts" | wc -c | tr -d ' ')
echo "no literal /tmp/ path in $scripts tracked Makefiles and scripts outside $entries named exceptions"

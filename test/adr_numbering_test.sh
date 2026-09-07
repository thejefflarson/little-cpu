#!/bin/bash
# Asserts that every ADR file has a unique number and exactly one row in the index, both
# ways round.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=${1:-$(cd "$HERE/.." && pwd)}
ADR="$REPO/docs/adr"
README="$ADR/README.md"

if [ ! -d "$ADR" ] || [ ! -f "$README" ]; then
  echo "error: $ADR or its README.md is missing, so there is nothing to grade." >&2
  exit 1
fi

rc=0

files=$(cd "$ADR" && ls -1 | grep -E '^[0-9]{4}-.*\.md$' | sort)
[ -n "$files" ] || { echo "error: no NNNN-*.md files found under $ADR." >&2; exit 1; }

dupes=$(sed -E 's/^([0-9]{4})-.*/\1/' <<< "$files" | uniq -d)
if [ -n "$dupes" ]; then
  rc=1
  while IFS= read -r n; do
    echo "error: ADR number $n is claimed by more than one file:" >&2
    grep -E "^$n-" <<< "$files" | sed -e 's|^|  |' >&2
  done <<< "$dupes"
fi

rows=$(grep -oE '^\| \[[0-9]{4}\]\([0-9]{4}-[a-z0-9-]+\.md\)' "$README" \
         | sed -E 's/.*\(([0-9]{4}-[a-z0-9-]+\.md)\)/\1/' | sort)

rows_unique=$(sort -u <<< "$rows")

while IFS= read -r f; do
  [ -n "$f" ] || continue
  rc=1
  echo "error: $f has no row in docs/adr/README.md." >&2
done < <(comm -23 <(printf '%s\n' "$files") <(printf '%s\n' "$rows_unique"))

while IFS= read -r f; do
  [ -n "$f" ] || continue
  rc=1
  echo "error: $f has more than one row in docs/adr/README.md, not one." >&2
done < <(comm -12 <(printf '%s\n' "$files") <(uniq -d <<< "$rows"))

while IFS= read -r f; do
  [ -n "$f" ] || continue
  rc=1
  echo "error: docs/adr/README.md has a row naming $f, and no such file exists." >&2
done < <(comm -23 <(printf '%s\n' "$rows_unique") <(printf '%s\n' "$files"))

if [ "$rc" -ne 0 ]; then
  echo >&2
  echo "ADR numbering is inconsistent. A gap in the sequence is fine -- work" >&2
  echo "merges around a reserved number sometimes -- but a collision or an" >&2
  echo "orphaned row is not: it is how two PRs land on one ADR number with git" >&2
  echo "seeing no conflict at all." >&2
  exit 1
fi

echo "$(wc -l <<< "$files" | tr -d ' ') ADR files, each with exactly one README row, no number claimed twice."

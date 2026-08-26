#!/bin/bash
# Asserts that every ADR file has a unique number and exactly one row in the
# index, both ways round.
#
# Usage: adr_numbering_test.sh [repo-root]     # defaults to this script's parent
#
# WHY THIS EXISTS. Two PRs once claimed the same ADR number under two
# different filenames. The README row each one added is a full line in the
# same region of one table, so the two insertions collided and git refused to
# merge them -- that half caught itself. The filename did not: `NNNN-a.md`
# and `NNNN-b.md` are two different paths git has never seen conflict, and
# only one README row need exist for the pair to slip through undetected. An
# architect caught it by reading the table; this script is the mechanism.
#
# A gap in the sequence is not a defect -- work merges around a reserved
# number sometimes and leaves it unused -- so this checks for a COLLISION
# (two files claiming one number) and an ORPHAN (a row with no file, or a
# file with no row), never for a number the sequence skipped.
#
# Hermetic: ls, grep and sed. No git, no toolchain, so this runs inside
# `make test` anywhere, and a fixture directory that is not a checkout can be
# graded the same way the shipping tree is.
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

dupes=$(sed -E 's/^([0-9]{4})-.*/\1/' <<< "$files" | sort | uniq -d)
if [ -n "$dupes" ]; then
  rc=1
  while IFS= read -r n; do
    echo "error: ADR number $n is claimed by more than one file:" >&2
    grep -E "^$n-" <<< "$files" | sed -e 's|^|  |' >&2
  done <<< "$dupes"
fi

# The linked filename, not the displayed number in brackets -- the filename is
# what a second insertion at the same number would fail to collide on.
rows=$(grep -oE '^\| \[[0-9]{4}\]\([0-9]{4}-[a-z0-9-]+\.md\)' "$README" \
         | sed -E 's/.*\(([0-9]{4}-[a-z0-9-]+\.md)\)/\1/' | sort)

while IFS= read -r f; do
  n=$(grep -cxF -- "$f" <<< "$rows" || true)
  if [ "$n" -eq 0 ]; then
    rc=1
    echo "error: $f has no row in docs/adr/README.md." >&2
  elif [ "$n" -gt 1 ]; then
    rc=1
    echo "error: $f has $n rows in docs/adr/README.md, not one." >&2
  fi
done <<< "$files"

while IFS= read -r f; do
  [ -n "$f" ] || continue
  if ! grep -qxF -- "$f" <<< "$files"; then
    rc=1
    echo "error: docs/adr/README.md has a row naming $f, and no such file exists." >&2
  fi
done <<< "$rows"

if [ "$rc" -ne 0 ]; then
  echo >&2
  echo "ADR numbering is inconsistent. A gap in the sequence is fine -- work" >&2
  echo "merges around a reserved number sometimes -- but a collision or an" >&2
  echo "orphaned row is not: it is how two PRs land on one ADR number with git" >&2
  echo "seeing no conflict at all." >&2
  exit 1
fi

echo "$(wc -l <<< "$files" | tr -d ' ') ADR files, each with exactly one README row, no number claimed twice."

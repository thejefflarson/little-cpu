#!/bin/bash
# Asserts that a word this repo retired has not come back anywhere it was
# retired from.
#
# Usage: retired_term_test.sh [repo-root]     # defaults to this script's parent
#
# WHY THIS EXISTS. "Ladder" was this repo's name for the generated riscv-formal
# check set, and it was retired: one sweep took out 59 occurrences across 23
# files, including probe labels and the diagnostics a gate prints when it refuses
# to grade. The sweep added no guard, and the word was back on main within hours
# -- in a comment written on a branch that predated the sweep and merged after
# it, with nothing anywhere to object. That merge order is the reintroduction
# path, and no amount of care during a sweep closes it.
#
# WHY THE ALLOW-LIST IS PROSE. A blind grep for this word would have been wrong
# three times in this repo already: test/probe_gates.sh used it for a hierarchy
# of exit codes, test/check_suite_shape.sh for a set-equality property, and
# docs/THREAT_MODEL.md for a stepwise sequence of milestones. All three are a
# different word that reads the same, and the sweep told them apart only because
# a person read them. So each entry below states WHICH SENSE it carries and why
# it stays; an entry that is only a path teaches the next reader nothing and gets
# deleted by the next person tidying.
#
# The comparison runs BOTH WAYS, like every other table in this repo: an entry
# naming a site the word has left is as red as the word appearing at a site no
# entry names. An exemption that outlives its reason is how the next one gets
# waved through.
#
# Hermetic: git, grep and sed. No toolchain, no simulator, no yosys, so this runs
# inside `make test` anywhere.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=${1:-$(cd "$HERE/.." && pwd)}

if [ ! -d "$REPO" ]; then
  echo "error: '$REPO' is not a directory, so there is nothing to scan." >&2
  exit 1
fi

# The word itself, matched case-insensitively and as a substring rather than a
# whole word, so that a capitalised or pluralised spelling cannot walk past.
RETIRED_TERM='ladder'

# The allow-list. One path per line -- a file, or a directory ending in `/` --
# with the sense the word carries there written above it.
allow_paths() {
  sed -e 's/#.*//' -e 's/[[:space:]]*$//' -e '/^$/d' <<'PATHS'
# The link text of Mozilla's published "code of conduct enforcement ladder",
# inside the URL that cites it. Someone else's document title: rewriting it
# would misquote the source rather than tidy this repo's vocabulary.
CODE_OF_CONDUCT.md

# "The milestone ladder" -- a stepwise sequence of milestones, which is a
# different word that reads the same. The file says so at the sentence itself,
# because a reader who finds only this entry would still have to guess.
docs/THREAT_MODEL.md

# Dated decision records, and dated proposals. Both are history: several of the
# filenames are the word, and the whole value of a record is that it says what
# was decided in the vocabulary of the day. Editing them would rewrite the
# record to agree with a later decision.
docs/adr/
docs/ideas/

# The probes that force this check red spell the word in their fixtures and
# their labels. A probe that cannot name what it is planting is not a probe.
test/probe_gates.sh

# This file: the declaration above, the diagnostic below and this list. The
# check has to be able to say what it is looking for.
test/retired_term_test.sh
PATHS
}

tmp=$(mktemp -d "${TMPDIR:-/tmp}/littlecpu-retired.XXXXXX") || {
  echo "error: could not create a temporary directory under ${TMPDIR:-/tmp}." >&2
  exit 1
}
trap 'rm -rf "$tmp"' EXIT

allow_paths > "$tmp/allow"

# Tracked files only, and the enumeration is git's rather than a `find` with a
# prune list. What this guard defends against is the word arriving in a commit,
# and a checkout carries build artifacts, downloaded tools and -- in the primary
# checkout here -- whole worktrees of the repo under `.claude/`, none of which a
# merge can reintroduce anything through. A prune list would have to grow every
# time one of those appeared.
if ! git -C "$REPO" ls-files -z > "$tmp/files" 2>/dev/null || [ ! -s "$tmp/files" ]; then
  echo "error: cannot enumerate any tracked files under $REPO. This check reads" >&2
  echo "git's index, because what it guards is the word arriving in a commit; a" >&2
  echo "tree git cannot list is a scan of nothing reporting green." >&2
  exit 1
fi

# `/dev/null` as a first argument so grep always prefixes the filename, even when
# xargs hands it a single file. No match at all leaves grep with status 1 and
# xargs with 123, which is not an error here.
hits=$( (cd "$REPO" && xargs -0 grep -inI -e "$RETIRED_TERM" -- /dev/null < "$tmp/files") || true)

# Which allow-list entry covers a path, or nothing. A directory entry matches on
# the `/` it ends with, so `docs/adr/` cannot quietly cover a `docs/adrenaline.md`
# that nobody meant to exempt.
match_entry() {  # $1 = path
  local path=$1 entry
  while IFS= read -r entry; do
    case "$entry" in
      */) case "$path" in "$entry"*) printf '%s' "$entry"; return 0 ;; esac ;;
      *)  if [ "$path" = "$entry" ]; then printf '%s' "$entry"; return 0; fi ;;
    esac
  done < "$tmp/allow"
  return 0
}

: > "$tmp/unexpected"
: > "$tmp/covered"

while IFS= read -r hit; do
  [ -n "$hit" ] || continue
  path=${hit%%:*}
  entry=$(match_entry "$path")
  if [ -n "$entry" ]; then
    printf '%s\n' "$entry" >> "$tmp/covered"
  else
    printf '%s\n' "$hit" >> "$tmp/unexpected"
  fi
done <<< "$hits"

rc=0

if [ -s "$tmp/unexpected" ]; then
  rc=1
  echo "error: the retired term '$RETIRED_TERM' appears where nothing allows it:" >&2
  sed -e 's|^|  |' "$tmp/unexpected" >&2
  echo >&2
  echo "It was this repo's name for the generated riscv-formal check set and is" >&2
  echo 'retired. Write "the generated riscv-formal checks", or "these checks"' >&2
  echo "where the context is already a formal one." >&2
  echo >&2
  echo "If the use above is genuinely a DIFFERENT word -- a hierarchy of exit" >&2
  echo "codes, a set-equality property, a stepwise sequence of milestones, another" >&2
  echo "project's document title -- add its path to the allow-list in" >&2
  echo "test/retired_term_test.sh, with the sense it carries and why it stays. An" >&2
  echo "entry that is only a path is one the next person tidying will delete." >&2
fi

while IFS= read -r entry; do
  if ! grep -qxF -- "$entry" "$tmp/covered"; then
    rc=1
    echo >&2
    echo "error: the allow-list exempts $entry, and '$RETIRED_TERM' does not" >&2
    echo "appear there any more. Delete the entry -- or if that use moved, move" >&2
    echo "the entry with it. An exemption kept past its reason is how the next one" >&2
    echo "gets waved through, and it is the reason this comparison runs both ways." >&2
  fi
done < "$tmp/allow"

if [ "$rc" -ne 0 ]; then
  exit 1
fi

entries=$(wc -l < "$tmp/allow" | tr -d ' ')
files=$(tr -cd '\0' < "$tmp/files" | wc -c | tr -d ' ')
echo "Retired term '$RETIRED_TERM' confined to its $entries allowed sites, over $files tracked files"

#!/bin/bash
# Grades test/MUTATION_COVERAGE: every rtl/*.v file must have a ruling, and
# every ruling must name something real.
#
# Usage: mutation_coverage_test.sh [repo-root]     # defaults to this script's
#                                                   # parent
#
# WHY THIS EXISTS. `make mutation-check` grades test/MUTATION_DETECTORS
# against test/mutations/*.patch, both ways round -- a claim about the eleven
# patches that exist. It says nothing about an rtl/*.v file no patch touches,
# so a new file joins fourteen others in silence by default. This script is
# the other set-equality check: test/MUTATION_COVERAGE's file column against
# `ls rtl/*.v`, both ways round, plus a check that every claim in the second
# column names something real rather than a plausible-looking word.
#
# A bare `unpaired` with no grader would defeat the whole point -- a manifest
# that permits one teaches people to write it, and the coverage ratchet
# becomes the thing it was written to prevent. So every `unpaired` line's
# grader is checked against the real bench list, the real formal component
# tasks, and a short list of other named `make` targets, each validated
# against the Makefiles that define them rather than assumed.
#
# Hermetic: grep, sed, sort, comm. No toolchain, no simulator, no yosys, so
# this runs inside `make test` anywhere.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=${1:-$(cd "$HERE/.." && pwd)}

if [ ! -d "$REPO" ]; then
  echo "error: '$REPO' is not a directory, so there is nothing to grade." >&2
  exit 1
fi

MANIFEST="$REPO/test/MUTATION_COVERAGE"
DETECTORS="$REPO/test/MUTATION_DETECTORS"
COMPONENTS_SBY="$REPO/formal/components.sby"

for f in "$MANIFEST" "$DETECTORS" "$COMPONENTS_SBY" "$REPO/Makefile" "$REPO/formal/Makefile"; do
  if [ ! -f "$f" ] || [ ! -r "$f" ]; then
    echo "error: '$f' does not exist or is not readable. This check reads it" >&2
    echo "as a source of truth; if it moved, move this check with it." >&2
    exit 1
  fi
done

rc=0
fail() {
  echo "error: $*" >&2
  rc=1
}

# Strips comments and blank lines, prints each remaining line's first field.
# Used against both this file and test/MUTATION_DETECTORS, which share the
# convention.
first_fields() {  # $1 = file
  sed -e 's/#.*//' "$1" | awk 'NF { print $1 }'
}

# ---- 1. the manifest names exactly rtl/*.v, both ways round ----------------

listed=$(first_fields "$MANIFEST" | sort)
if [ -z "$listed" ]; then
  echo "error: $MANIFEST names no files. An empty manifest matches an empty" >&2
  echo "rtl/ and reports success." >&2
  exit 1
fi

duplicates=$(printf '%s\n' "$listed" | uniq -d)
if [ -n "$duplicates" ]; then
  fail "$MANIFEST names the same file more than once, which would make the
comparison below non-symmetric:
$(printf '%s\n' "$duplicates" | sed -e 's|^|  |')"
fi

present=$(cd "$REPO" && ls rtl/*.v 2>/dev/null | sort)
if [ -z "$present" ]; then
  echo "error: no rtl/*.v files found under '$REPO'." >&2
  exit 1
fi

missing=$(comm -23 <(printf '%s\n' "$listed") <(printf '%s\n' "$present"))
unlisted=$(comm -13 <(printf '%s\n' "$listed") <(printf '%s\n' "$present"))

if [ -n "$missing" ]; then
  fail "$MANIFEST names files rtl/ does not have:
$(printf '%s\n' "$missing" | sed -e 's|^|  |')
The file was deleted or renamed and its ruling was left behind. Remove the
line in the same commit that removes the file."
fi

if [ -n "$unlisted" ]; then
  fail "rtl/ has files $MANIFEST names no ruling for:
$(printf '%s\n' "$unlisted" | sed -e 's|^|  |')
A new rtl/*.v file joins the ones no mutation touches, silently, unless this
file says so. Add its line -- a mutation name, or \`unpaired\` and the real
grader that covers it -- in the same commit."
fi

# ---- 2. every ruling names something real -----------------------------------

valid_mutations=$(first_fields "$DETECTORS" | sort -u)

# Every test/*_tb.v bench, by the name `make test-unit-<bench>` runs it under.
valid_benches=$(cd "$REPO" && ls test/*_tb.v 2>/dev/null | xargs -n1 basename | sed -e 's/\.v$//' | sort -u)

# Every formal component task declared in formal/components.sby's [tasks]
# section, prefixed the way `make -C formal components_<task>` names it.
# Parsed rather than listed here by hand, so a task added or renamed there is
# picked up without a second edit.
valid_components=$(awk '/^\[tasks\]/ { f=1; next } /^\[/ { f=0 } f && NF { print "components_" $1 }' \
                      "$COMPONENTS_SBY" | sort -u)

# The rest are named `make` targets this file's own graders are, checked
# against the Makefiles that declare them rather than trusted on sight. Kept
# to the ones test/MUTATION_COVERAGE actually names: a grader nothing here
# claims does not need to be enumerable.
other_graders="check memmap-test dual-smoke dual-ecp5-timing"
for g in $other_graders; do
  if ! grep -qE "^${g}:" "$REPO/Makefile" "$REPO/formal/Makefile" 2>/dev/null; then
    echo "error: '$g' is in this script's own other_graders list but neither" >&2
    echo "Makefile nor formal/Makefile declares a target by that name. Fix the" >&2
    echo "list or the target, in the same commit." >&2
    exit 1
  fi
done
# shellcheck disable=SC2086 # deliberate word-splitting, one grader per line
other_graders_lines=$(printf '%s\n' $other_graders)
valid_graders=$(printf '%s\n%s\n%s\n' "$valid_benches" "$valid_components" \
                  "$other_graders_lines" | sort -u)

while IFS= read -r line; do
  line=${line%%#*}
  [ -z "${line// /}" ] && continue
  # shellcheck disable=SC2086 # deliberate word-splitting to count fields
  set -- $line
  file=$1
  case "${2:-}" in
    unpaired)
      if [ "$#" -ne 3 ]; then
        fail "$file: an \`unpaired\` line needs exactly one grader name; got
$(($# - 2)) field(s) after \`unpaired\`."
        continue
      fi
      grader=$3
      if ! printf '%s\n' "$valid_graders" | grep -qxF -- "$grader"; then
        fail "$file: \`unpaired $grader\` names no real bench, formal
component task, or other declared grader. Checked against test/*_tb.v,
formal/components.sby's [tasks] section, and $other_graders."
      fi
      ;;
    "")
      fail "$file: no ruling at all -- neither a mutation name nor \`unpaired\`
and a grader."
      ;;
    *)
      if [ "$#" -ne 2 ]; then
        fail "$file: a mutation-name line takes exactly one field; got $(($# - 1))."
        continue
      fi
      mutation=$2
      if ! printf '%s\n' "$valid_mutations" | grep -qxF -- "$mutation"; then
        fail "$file: \`$mutation\` is not in test/MUTATION_DETECTORS's first
column. Either the mutation was renamed or this line was never a real one."
      fi
      ;;
  esac
done < "$MANIFEST"

if [ "$rc" -ne 0 ]; then
  exit 1
fi

echo "Mutation coverage: $(printf '%s\n' "$present" | wc -l | tr -d ' ') rtl/*.v files, each ruled on."

#!/bin/bash
# The riscv-formal ladder's gate: the generated check set must equal
# formal/EXPECTED_CHECKS, and the not-PASS set must equal formal/EXPECTED_FAIL,
# both by set equality in both directions.
#
# The check set is asserted because checks.cfg's [depth] table is the list of
# checks that EXIST — genchecks silently skips a check with no depth line — so
# a lost line drops a check from the results and from EXPECTED_FAIL at once, and
# the verdict comparison alone calls that a clean match.
#
# The status is compared, not just the name, because ERROR is a broken harness
# where FAIL is a failed property. On a machine without `btorsim` every red
# check flips FAIL -> ERROR and a name-only equality still matches (ADR-0036).
#
# Nothing here is a verdict about the core: every check is `mode bmc` under
# RISCV_FORMAL_ALTOPS, so a passing insn_mul says nothing about the real
# multiplier (ADR-0010).
#
# Usage: check-baseline.sh <checks-dir> <expected-fail-file> [expected-checks-file]
set -u

if [ $# -lt 2 ] || [ $# -gt 3 ]; then
  echo "usage: $0 <checks-dir> <expected-fail-file> [expected-checks-file]" >&2
  exit 2
fi

CHECKS_DIR=$1
EXPECTED_FAIL=$2
EXPECTED_CHECKS=${3:-$(dirname "$0")/EXPECTED_CHECKS}

# READABLE, not merely present. This script sets `set -u` and neither `-e` nor
# `pipefail`, so a `sed` that cannot open its input yields an EMPTY string — and
# an empty expected set matches an all-passing ladder exactly.
for f in "$EXPECTED_FAIL" "$EXPECTED_CHECKS"; do
  if [ ! -f "$f" ]; then
    echo "error: no such file: $f" >&2
    exit 2
  fi
  if [ ! -r "$f" ]; then
    echo "error: $f exists but is not readable." >&2
    echo "An unreadable baseline reads as an EMPTY one here, which matches an" >&2
    echo "all-passing ladder exactly and exits 0. Refusing to grade." >&2
    exit 2
  fi
done

# `NF` must gate the rebuild rather than follow it: awk forces NF to 1 when $1
# is assigned, so `{$1=$1} NF` would resurrect every blank and comment-only line
# as an entry. Same idiom, same reason, as test/run_tests.sh.
strip() {
  sed -e 's/#.*//' "$1" | awk 'NF { $1 = $1; print }' | sort
}

expected_checks=$(strip "$EXPECTED_CHECKS")
expected_fail=$(strip "$EXPECTED_FAIL")

# Everything sby can write, plus this script's own NO-STATUS. A status outside
# the set means sby's output changed under us, and is reported rather than
# bucketed into "not PASS".
known_status() {
  case $1 in
    PASS | FAIL | ERROR | UNKNOWN | TIMEOUT | NO-STATUS) return 0 ;;
    *) return 1 ;;
  esac
}

# The strictly smaller set EXPECTED_FAIL may carry. ERROR and NO-STATUS mean the
# harness broke rather than the property failing, so baselining one would
# re-create the btorsim hole with this gate's blessing on it. TIMEOUT and
# UNKNOWN are budget exhaustion, which is a real verdict, so they are accepted.
baselineable_status() {
  case $1 in
    FAIL | TIMEOUT | UNKNOWN) return 0 ;;
    *) return 1 ;;
  esac
}

# Exit 2 is "the inputs are broken" against exit 1's "the ladder disagrees with
# them", so a rejected line cannot read as a regression in the ladder.
baseline_errors=""
while IFS= read -r line; do
  [ -n "$line" ] || continue
  # shellcheck disable=SC2086 # deliberate word split: the line is normalised.
  set -- $line
  case $# in
    1)
      baseline_errors+="  $line
      -> no status field. The format is '<check>  <STATUS>'.
"
      ;;
    2)
      if ! known_status "$2"; then
        baseline_errors+="  $line
      -> '$2' is not a status sby can produce. Accepted: PASS FAIL ERROR
         UNKNOWN TIMEOUT NO-STATUS.
"
      elif ! baselineable_status "$2"; then
        baseline_errors+="  $line
      -> '$2' must never be baselined. PASS cannot appear in a failure set at
         all; ERROR and NO-STATUS are a broken harness rather than a known-red
         property, and parking one here is what ADR-0036 exists to forbid.
         Baselineable: FAIL TIMEOUT UNKNOWN.
"
      fi
      ;;
    *)
      baseline_errors+="  $line
      -> $# fields. The format is exactly two: '<check>  <STATUS>'. sby's
         trailing engine numbers ('PASS 0 31') are not part of the status.
"
      ;;
  esac
done <<< "$expected_fail"

if [ -n "$baseline_errors" ]; then
  echo "error: $EXPECTED_FAIL is malformed (ADR-0036 format):" >&2
  printf '%s' "$baseline_errors" >&2
  echo "Read that file's header before editing it; the format is the gate." >&2
  exit 2
fi

# The `.sby` FILES, not the run directories: sby creates a directory only for a
# check it actually starts, so globbing directories made a generated-but-never-
# scheduled check fall out of the results and out of the baseline at once. `find`
# rather than a glob so an empty directory yields nothing, not `*.sby`.
generated=$(find "$CHECKS_DIR" -maxdepth 1 -name '*.sby' \
  -exec basename {} .sby \; 2>/dev/null | sort)

if [ -z "$generated" ]; then
  echo "error: no *.sby files under $CHECKS_DIR -- the ladder was never" >&2
  echo "       generated. Run 'make -C formal checks' first." >&2
  exit 2
fi

failed=0

if [ "$generated" != "$expected_checks" ]; then
  echo "Generated check set does NOT match $EXPECTED_CHECKS:"
  diff <(echo "$expected_checks") <(echo "$generated") \
    --label expected --label generated
  echo
  echo "A check that disappeared here lost its formal/checks.cfg [depth] line"
  echo "(ADR-0033). A check that appeared needs a [depth] line, an"
  echo "EXPECTED_CHECKS line, and -- if red -- an $EXPECTED_FAIL entry."
  echo
  failed=1
fi

# The UNION, so a name missing from either side still gets a verdict rather than
# dropping out of the tally.
all_checks=$(printf '%s\n%s\n' "$generated" "$expected_checks" | sort -u)

total=0
declare -a actual_fail=()
declare -a unknown_statuses=()
for name in $all_checks; do
  total=$((total + 1))
  # First word only: sby writes `<VERDICT> <engine> <depth>`, and the numbers
  # move with the engine.
  status=$(awk 'NF { print $1; exit }' "$CHECKS_DIR/$name/status" 2>/dev/null)
  if [ -z "$status" ]; then
    status="NO-STATUS"
  fi
  if ! known_status "$status"; then
    unknown_statuses+=("$name $status")
  fi
  if [ "$status" != "PASS" ]; then
    actual_fail+=("$name $status")
  fi
done

passed=$((total - ${#actual_fail[@]}))
echo "$total checks: $passed pass, ${#actual_fail[@]} fail"

if [ ${#unknown_statuses[@]} -gt 0 ]; then
  echo
  echo "Unrecognised status(es) on disk -- not one of PASS FAIL ERROR UNKNOWN"
  echo "TIMEOUT. sby's output has changed; do not baseline these, fix the"
  echo "vocabulary in $0 after reading what the pin now emits:"
  printf '  %s\n' "${unknown_statuses[@]}"
  failed=1
fi

actual_sorted=$(printf '%s\n' "${actual_fail[@]:-}" | awk 'NF { $1 = $1; print }' | sort)

if [ "$actual_sorted" = "$expected_fail" ]; then
  echo "Failure list matches $EXPECTED_FAIL exactly (name and status)."
else
  echo
  echo "Failure list does NOT match $EXPECTED_FAIL:"
  diff <(echo "$expected_fail") <(echo "$actual_sorted") \
    --label expected --label actual
  echo
  echo "A name on one side only is a check whose verdict moved. A name on BOTH"
  echo "sides with different statuses is the ADR-0036 case: the check is still"
  echo "red, but for a different reason than the one baselined -- ERROR and"
  echo "NO-STATUS mean the harness broke, not that the property failed."
  failed=1
fi

if [ "$failed" -eq 0 ]; then
  echo "Generated check set matches $EXPECTED_CHECKS exactly ($total checks)."
fi

exit "$failed"

#!/bin/bash
# The riscv-formal ladder's gate. Asserts TWO things, and both are set
# equalities in both directions (ADR-0014's contract, applied to the formal
# ladder per ADR-0022 -- an unexpected *pass*, or an unexpected *check*,
# fails this as loudly as a regression):
#
#   1. SHAPE.  The checks genchecks generated (formal/checks/*.sby) are
#      exactly the ones formal/EXPECTED_CHECKS names.
#   2. VERDICT. The checks whose status is not PASS are exactly the ones
#      formal/EXPECTED_FAIL names.
#
# (1) is not decoration. Without it this script could not tell a ladder that
# shrank from one that passed. formal/checks.cfg's [depth] table is the list
# of checks that EXIST -- genchecks skips any check with no depth line,
# silently -- so deleting one line removes a check from the results AND from
# EXPECTED_FAIL at the same time, and (2) alone reports a clean match on
# less coverage than it claims. That is ADR-0033's gap 1, and (1) closes it.
#
# This script deliberately enumerates the generated `*.sby` FILES, not the
# run directories. `sby` creates a directory only for a check it actually
# STARTS, so globbing directories made a generated-but-never-scheduled check
# invisible: it fell out of the actual set, it was never in the baseline,
# and set equality called that a match. Reading the .sby files instead means
# such a check resolves to NO-STATUS and counts as non-PASS, which is what
# this script's header has promised since it was written -- "make stopped
# scheduling it, or it's still running" is a failure, not a quiet pass.
#
# A check named in EXPECTED_CHECKS with no .sby on disk is likewise
# NO-STATUS, so a lost [depth] line trips BOTH assertions rather than
# neither.
#
# Nothing here is a verdict about the core. Every check on this ladder is
# `mode bmc`: PASS means no counterexample was found within that check's
# configured depth, not that the property holds. And the whole ladder runs
# under RISCV_FORMAL_ALTOPS, so a passing insn_mul/insn_div says nothing
# whatever about the real multiplier or divider (ADR-0010).
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

for f in "$EXPECTED_FAIL" "$EXPECTED_CHECKS"; do
  if [ ! -f "$f" ]; then
    echo "error: no such file: $f" >&2
    exit 2
  fi
done

# `#` comments and blank lines out, as both baseline files already allow.
strip() {
  sed -e 's/#.*//' -e 's/[[:space:]]*$//' -e '/^[[:space:]]*$/d' "$1" | sort
}

expected_checks=$(strip "$EXPECTED_CHECKS")
expected_fail=$(strip "$EXPECTED_FAIL")

# The generated set: one line per checks/<name>.sby. `find` rather than a
# glob, so a missing or empty directory yields nothing instead of the
# literal unexpanded pattern -- which would otherwise become a one-element
# set named `*.sby` and produce a confusing diff instead of the explicit
# error below.
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

# Status is read over the UNION of what was generated and what was expected,
# so a name missing from either side still gets a verdict rather than
# dropping out of the tally entirely.
all_checks=$(printf '%s\n%s\n' "$generated" "$expected_checks" | sort -u)

total=0
declare -a actual_fail=()
for name in $all_checks; do
  total=$((total + 1))
  status=$(awk '{print $1; exit}' "$CHECKS_DIR/$name/status" 2>/dev/null)
  if [ -z "$status" ]; then
    status="NO-STATUS"
  fi
  if [ "$status" != "PASS" ]; then
    actual_fail+=("$name")
  fi
done

passed=$((total - ${#actual_fail[@]}))
echo "$total checks: $passed pass, ${#actual_fail[@]} fail"

actual_sorted=$(printf '%s\n' "${actual_fail[@]:-}" | sed '/^$/d' | sort)

if [ "$actual_sorted" = "$expected_fail" ]; then
  echo "Failure list matches $EXPECTED_FAIL exactly."
else
  echo
  echo "Failure list does NOT match $EXPECTED_FAIL:"
  diff <(echo "$expected_fail") <(echo "$actual_sorted") \
    --label expected --label actual
  failed=1
fi

if [ "$failed" -eq 0 ]; then
  echo "Generated check set matches $EXPECTED_CHECKS exactly ($total checks)."
fi

exit "$failed"

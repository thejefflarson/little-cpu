#!/bin/bash
# Tallies the riscv-formal ladder's per-check status files (`make -C formal
# check`, formal/checks/*/status) and checks the resulting non-PASS set
# against formal/EXPECTED_FAIL — same contract as test/run_tests.sh /
# ADR-0014, applied to the formal ladder per ADR-0022: set equality in both
# directions, so an unexpected *pass* trips this as loudly as a new failure.
#
# A check whose directory has no `status` file at all (make stopped
# scheduling it, or it's still running) counts as non-PASS too — that is
# exactly the "ladder didn't actually finish" failure mode ADR-0022 names,
# and it must not read as a quiet, matching baseline.
#
# Usage: check-baseline.sh <checks-dir> <expected-fail-file>
set -u

CHECKS_DIR=$1
EXPECTED_FAIL=$2

declare -a dirs=("$CHECKS_DIR"/*/)
total=${#dirs[@]}
declare -a actual_fail=()

for d in "${dirs[@]}"; do
  name=$(basename "$d")
  status=$(awk '{print $1; exit}' "$d/status" 2>/dev/null)
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
expected_sorted=$(sed -e 's/#.*//' -e '/^[[:space:]]*$/d' "$EXPECTED_FAIL" | sort)

if [ "$actual_sorted" = "$expected_sorted" ]; then
  echo "Failure list matches $EXPECTED_FAIL exactly."
  exit 0
fi

echo
echo "Failure list does NOT match $EXPECTED_FAIL:"
diff <(echo "$expected_sorted") <(echo "$actual_sorted") \
  --label expected --label actual
exit 1

#!/bin/bash
# Grades soc/board_verdict.sh: a verdict read off the UART is data, never code. Hostile
# text must come back PARSE and run nothing; honest verdicts must still grade.
# $1 overrides the library under test (probe_gates.sh passes a mutated copy).
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
LIB=${1:-$HERE/../soc/board_verdict.sh}
[ -f "$LIB" ] || { echo "error: no verdict library at $LIB" >&2; exit 1; }
# shellcheck source=/dev/null
. "$LIB"

WORK=$(mktemp -d "${TMPDIR:-/tmp}/board_verdict.XXXXXX")
trap 'rm -rf "$WORK"' EXIT
fail=0

expect() {
  local v=$1 want=$2 got
  got=$(grade_verdict "$v")
  if [ "$got" != "$want" ]; then
    echo "FAIL: verdict '$v' graded '$got', expected '$want'" >&2
    fail=1
  fi
}

expect 1 PASS
expect 7 "FAIL 3"
expect 08 "FAIL 4"
expect 001 PASS
expect '' PARSE
expect abc PARSE
expect -3 PARSE
expect 1234567890 PARSE

hostile=("x[\$(touch $WORK/pwned)]" "x[\`touch $WORK/pwned\`]" "a[\$(touch $WORK/pwned)]+1")
for v in "${hostile[@]}"; do
  expect "$v" PARSE
done

if [ -e "$WORK/pwned" ]; then
  echo "FAIL: a hostile verdict executed a command" >&2
  fail=1
fi

[ "$fail" -eq 0 ] || exit 1
echo "board verdict parse OK: hostile verdicts rejected, nothing executed"

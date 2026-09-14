#!/bin/bash
# Grades a nano-style suite on two simulator legs and requires them to agree program by program.
set -euo pipefail

if [ "$#" -ne 6 ]; then
  echo "usage: nano_dual_leg_test.sh <cxxrtl-sim> <icarus-sim> <asm-dir> <expected-fail>" \
       "<floor> <cflags>" >&2
  exit 1
fi

CXXRTL_SIM=$1
ICARUS_SIM=$2
ASM_DIR=$3
EXPECTED_FAIL=$4
OBSERVED_FLOOR=$5
CFLAGS=$6
HERE=$(cd "$(dirname "$0")" && pwd)
RUNNER="$HERE/../asm/run_nano_tests.sh"

tmp=$(mktemp -d "${TMPDIR:-/tmp}/nano-dual-leg.XXXXXX") || {
  echo "error: could not create a temporary directory under ${TMPDIR:-/tmp}." >&2
  exit 1
}
trap 'rm -rf "$tmp"' EXIT

# A per-program "<name> ... retires=<n>" line; wrapped so a no-match reaches the check below.
extract_table() {
  { grep -E '^[^[:space:]]+\.S +[^[:space:]]+.*retires=' "$1" || true; } | awk '{ $1=$1; print }'
}

# Independent simulators, so both legs run at once; each writes its own log and status.
run_leg() {  # <sim> <logfile>
  local sim=$1 logfile=$2
  set +e
  "$RUNNER" "$sim" "$ASM_DIR" "$EXPECTED_FAIL" "$OBSERVED_FLOOR" "$CFLAGS" > "$logfile" 2>&1
  echo $?
}

run_leg "$CXXRTL_SIM" "$tmp/cxxrtl.log" > "$tmp/cxxrtl.status" &
cxxrtl_pid=$!
run_leg "$ICARUS_SIM" "$tmp/icarus.log" > "$tmp/icarus.status" &
icarus_pid=$!
wait "$cxxrtl_pid"
wait "$icarus_pid"

echo "== cxxrtl leg =="
cat "$tmp/cxxrtl.log"
if [ "$(cat "$tmp/cxxrtl.status")" != "0" ]; then
  echo "error: the cxxrtl leg did not clear its own baseline; see above." >&2
  exit 1
fi

echo
echo "== iverilog leg =="
cat "$tmp/icarus.log"
if [ "$(cat "$tmp/icarus.status")" != "0" ]; then
  echo "error: the iverilog leg did not clear its own baseline; see above." >&2
  exit 1
fi

extract_table "$tmp/cxxrtl.log" > "$tmp/cxxrtl.table"
extract_table "$tmp/icarus.log" > "$tmp/icarus.table"

if [ ! -s "$tmp/cxxrtl.table" ] || [ ! -s "$tmp/icarus.table" ]; then
  echo "error: could not parse either leg's per-program results table." >&2
  exit 1
fi

echo
if diff --label cxxrtl --label iverilog "$tmp/cxxrtl.table" "$tmp/icarus.table" > "$tmp/diff.out"; then
  echo "Both simulator legs agree, program by program."
  exit 0
fi

echo "The two simulator legs disagree:" >&2
cat "$tmp/diff.out" >&2
exit 1

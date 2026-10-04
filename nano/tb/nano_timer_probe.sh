#!/bin/bash
# Requires nano_timer_tb.v to go red, for the reason each check was written, when the timer posts MTIP early, never posts it, compares one word, drops the carry, ignores byte strobes, lets a tick carry across a write, or decodes a wider window.
# Not hermetic: runs iverilog.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WORKDIR="$HERE/nano-timer-probe"

if ! command -v iverilog >/dev/null 2>&1; then
  echo "error: iverilog is not on PATH." >&2
  exit 2
fi
for name in nano/timer.v nano/tb/nano_timer_tb.v; do
  if [ ! -f "$REPO/$name" ]; then
    echo "error: $name is missing from $REPO." >&2
    exit 2
  fi
done

rm -rf "$WORKDIR"
mkdir -p "$WORKDIR"

run_bench() {  # $1 = timer.v to build, $2 = stem; prints the bench's output
  iverilog -g2012 -o "$WORKDIR/$2.vvp" "$1" "$REPO/nano/tb/nano_timer_tb.v" || return 1
  vvp "$WORKDIR/$2.vvp"
}

echo "control: the shipping timer"
if ! out=$(run_bench "$REPO/nano/timer.v" shipping 2>&1) || ! grep -q '^PASS$' <<< "$out"; then
  echo "$out"
  echo "*** the shipping timer fails its own bench, so a mutant failing it proves nothing." >&2
  exit 1
fi
echo "PASS"

expect_red() {  # $1 = stem, $2 = what the mutant does, $3 = the failure it must draw, $4 = sed program
  sed "$4" "$REPO/nano/timer.v" > "$WORKDIR/$1.v"
  if cmp -s "$REPO/nano/timer.v" "$WORKDIR/$1.v"; then
    echo "error: nano/timer.v no longer spells the line the $2 mutant rewrites. Re-anchor it --" \
         "left alone this builds the shipping timer twice and proves nothing." >&2
    exit 2
  fi
  echo
  echo "mutant: $2"
  if out=$(run_bench "$WORKDIR/$1.v" "$1" 2>&1) && grep -q '^PASS$' <<< "$out"; then
    echo "*** the $2 mutant passed the bench." >&2
    exit 1
  fi
  if ! grep -q "^FAIL .*$3" <<< "$out"; then
    echo "$out" | head -5
    echo "*** the $2 mutant was refused, but not by the check for '$3'." >&2
    exit 1
  fi
  grep -m1 "^FAIL .*$3" <<< "$out"
}

expect_red early "mtip compares mtime + 1, so it posts a cycle early" "posted early" \
  's/mtip <= {time_hi, time_lo} >= {cmp_hi, cmp_lo};/mtip <= time_inc >= {cmp_hi, cmp_lo};/'
expect_red dead "mtip never posts" "mtip late" \
  's/mtip <= {time_hi, time_lo} >= {cmp_hi, cmp_lo};/mtip <= 1'"'"'b0;/'
expect_red low_word "mtip compares the low words only" "posted early" \
  's/mtip <= {time_hi, time_lo} >= {cmp_hi, cmp_lo};/mtip <= time_lo >= cmp_lo;/'
expect_red no_carry "mtime's high word never increments" "mtip late" \
  's/wr_time_lo ? time_hi : time_inc\[63:32\]/wr_time_lo ? time_hi : time_hi/'
expect_red no_strobes "byte strobes are ignored" "after a byte store" \
  's/assign wmask = .*/assign wmask = 32'"'"'hffff_ffff;/'
expect_red no_suspend "a tick carries across a write to the low word" "carry edge" \
  's/wr_time_lo ? time_hi : time_inc\[63:32\]/time_inc[63:32]/'
expect_red wide_window "the decode ignores address bit 4" "stray stores" \
  's/mem_addr\[31:4\] == BASE\[31:4\]/mem_addr[31:5] == BASE[31:5]/'

echo
echo "nano_timer_tb.v passes the shipping timer and fails seven mutants for their own reasons."

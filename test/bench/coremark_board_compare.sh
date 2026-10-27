#!/bin/bash
# Requires the board's CoreMark report to equal the cxxrtl run's of the same ELF.
# The board repeats its report and a reader may attach mid-block, so only complete blocks count.
# usage: coremark_board_compare.sh <sim-log> <board-log>
set -euo pipefail

if [ "$#" -ne 2 ]; then
  echo "usage: coremark_board_compare.sh <sim-log> <board-log>" >&2
  exit 1
fi

blocks() {
  tr -d '\r' < "$1" | awk '
    /^Cycles +: [0-9]+$/ { c = $3; i = ""; next }
    /^Iterations +: [0-9]+$/ { if (c != "") i = $3; next }
    /^Self-check +: [A-Z]+$/ { if (c != "" && i != "") print c, i, $3; c = ""; i = ""; next }'
}

summarize() {  # $1 = label, $2 = log
  local found distinct
  if [ ! -s "$2" ]; then
    echo "error: $1 log '$2' is missing or empty." >&2
    return 1
  fi
  found=$(blocks "$2")
  if [ -z "$found" ]; then
    echo "error: $1 log '$2' holds no complete CoreMark report" \
         "(Cycles, Iterations and Self-check lines)." >&2
    return 1
  fi
  distinct=$(sort -u <<< "$found")
  if [ "$(wc -l <<< "$distinct")" -ne 1 ]; then
    echo "error: $1 log '$2' holds reports that disagree with each other:" >&2
    echo "$distinct" >&2
    return 1
  fi
  printf '%s\n' "$distinct"
}

sim=$(summarize "sim" "$1")
board=$(summarize "board" "$2")

read -r sim_cycles sim_iters sim_verdict <<< "$sim"
read -r board_cycles board_iters board_verdict <<< "$board"

fail=0
if [ "$sim_verdict" != "PASS" ]; then
  echo "error: the sim run's self-check read $sim_verdict, not PASS." >&2
  fail=1
fi
if [ "$board_verdict" != "PASS" ]; then
  echo "error: the board run's self-check read $board_verdict, not PASS." >&2
  fail=1
fi
if [ "$sim_iters" != "$board_iters" ]; then
  echo "error: iteration counts differ: sim $sim_iters, board $board_iters." >&2
  fail=1
fi
if [ "$sim_cycles" != "$board_cycles" ]; then
  echo "error: timed cycles differ: sim $sim_cycles, board $board_cycles" \
       "(board minus sim: $((board_cycles - sim_cycles)))." >&2
  fail=1
fi
if [ "$fail" -ne 0 ]; then
  exit 1
fi

echo "coremark board matches cxxrtl: $sim_cycles cycles over $sim_iters iterations, self-check PASS in both"

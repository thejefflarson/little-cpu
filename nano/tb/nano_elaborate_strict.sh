#!/bin/bash
# Fails a yosys elaboration on any warning but its deep-recursion notice, in both the file:line and
# bare forms yosys prints; the repo treats elaboration warnings as errors.
set -euo pipefail

if [ "$#" -ne 2 ]; then
  echo "usage: nano_elaborate_strict.sh <log-file> <yosys -p script>" >&2
  exit 2
fi
log=$1
script=$2

if ! yosys -p "$script" > "$log" 2>&1; then
  cat "$log" >&2
  echo "error: yosys failed before its warnings were graded." >&2
  exit 1
fi

if promoted=$(grep -E '(^|: )Warning: ' "$log" | grep -vE 'Deep recursion in AST simplifier'); then
  printf '%s\n' "$promoted" >&2
  echo "error: yosys reported the warning(s) above; elaboration warnings are errors." >&2
  exit 1
fi

echo "yosys: zero promoted warnings ($log)"

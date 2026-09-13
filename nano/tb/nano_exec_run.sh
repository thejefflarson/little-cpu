#!/bin/bash
# Builds and runs the mul/div oracle against a given nano.v -- the shipping one, or a mutated copy nano_exec_probe.sh hands it.
set -euo pipefail

if [ "$#" -ne 1 ]; then
  echo "usage: nano_exec_run.sh <path-to-nano.v>" >&2
  exit 2
fi

NANO_V=$1
HERE=$(cd "$(dirname "$0")" && pwd)

tmp=$(mktemp -d "${TMPDIR:-/tmp}/nano-exec-run.XXXXXX")
trap 'rm -rf "$tmp"' EXIT

cp "$HERE/nano_exec_cxxrtl.cc" "$tmp/nano_exec_cxxrtl.cc" # its #include resolves next to itself, not $HERE

yosys -p "read_verilog -sv \"$NANO_V\" \"$HERE/nano_exec_tb.v\"; hierarchy -top nano_exec_tb; write_cxxrtl \"$tmp/nano_exec_rtl.cc\"" \
  > "$tmp/synth.log" 2>&1 || { echo "error: yosys could not elaborate $NANO_V" >&2; tail -40 "$tmp/synth.log" >&2; exit 1; }

clang++ -O2 -DNDEBUG -std=c++17 -Wall -Wextra -Werror \
  -isystem "$(yosys-config --datdir)/include/backends/cxxrtl/runtime" \
  "$tmp/nano_exec_cxxrtl.cc" -o "$tmp/nano-exec-sim" \
  > "$tmp/build.log" 2>&1 || { echo "error: clang++ could not build the oracle against $NANO_V" >&2; cat "$tmp/build.log" >&2; exit 1; }

"$tmp/nano-exec-sim"

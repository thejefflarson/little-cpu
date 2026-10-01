#!/bin/bash
# Requires nano's iverilog leg to go red when nano stops honouring the macro's registered reads: operand addresses presented a cycle late, and the two read ports' addresses swapped.
# Not hermetic: runs the real cross compiler and iverilog.
set -euo pipefail

if [ "$#" -ne 3 ]; then
  echo "usage: nano_rf_timing_probe.sh <cflags> <rtl-srcs> <riscv-formal-macros>" >&2
  exit 2
fi
CFLAGS=$1
# shellcheck disable=SC2206
RTL_SRCS=($2)
# shellcheck disable=SC2206
FORMAL_MACROS=($3)
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WORKDIR="$HERE/nano-rf-timing-probe"

for name in "${RTL_SRCS[@]}" nano/tb/nano_testbench.v rvfi_macros.vh test/monitor.sim.v; do
  if [ ! -f "$REPO/$name" ]; then
    echo "error: $name is missing from $REPO. rvfi_macros.vh and test/monitor.sim.v are" \
      "make targets of their own -- run 'make rvfi_macros.vh test/monitor.sim.v' first." >&2
    exit 2
  fi
done
if ! command -v iverilog >/dev/null 2>&1; then
  echo "error: iverilog is not on PATH." >&2
  exit 2
fi

rm -rf "$WORKDIR"
mkdir -p "$WORKDIR"

build_vvp() {  # $1 = nano.v to use, $2 = vvp output path
  local define src
  local -a defines=(-DICARUS) sources=("$REPO/rvfi_macros.vh")
  for define in "${FORMAL_MACROS[@]}"; do
    defines+=(-D "$define")
  done
  for src in "${RTL_SRCS[@]}"; do
    if [ "$src" = "nano/nano.v" ]; then sources+=("$1"); else sources+=("$REPO/$src"); fi
  done
  iverilog -g2012 "${defines[@]}" -o "$2" \
    "${sources[@]}" "$REPO/nano/tb/nano_testbench.v" "$REPO/test/monitor.sim.v"
}

run_suite() {  # $1 = vvp image; prints the suite and returns its status
  NANO_VVP_IMAGE=$1 "$REPO/nano/asm/run_nano_tests.sh" "$REPO/nano/tb/nano_sim_icarus.sh" \
    "$REPO/nano/asm" "$REPO/nano/asm/EXPECTED_FAIL" "$REPO/nano/asm/OBSERVED_FLOOR" "$CFLAGS"
}

echo "control: the shipping nano.v"
build_vvp "$REPO/nano/nano.v" "$WORKDIR/shipping.vvp"
if ! out=$(run_suite "$WORKDIR/shipping.vvp" 2>&1); then
  echo "$out"
  echo "*** the shipping build fails its own suite, so a mutant failing it proves nothing." >&2
  exit 1
fi
echo "$out" | tail -3

expect_red() {  # $1 = file name stem, $2 = what the mutant does, $3 = sed program
  sed "$3" "$REPO/nano/nano.v" > "$WORKDIR/$1.v"
  if cmp -s "$REPO/nano/nano.v" "$WORKDIR/$1.v"; then
    echo "error: nano.v no longer spells the line the $2 mutant rewrites. Re-anchor it --" \
         "left alone this builds the shipping core twice and proves nothing." >&2
    exit 2
  fi
  build_vvp "$WORKDIR/$1.v" "$WORKDIR/$1.vvp"
  echo
  echo "mutant: $2"
  if out=$(run_suite "$WORKDIR/$1.vvp" 2>&1); then
    echo "$out" | tail -5
    echo "*** the $2 mutant passed the suite." >&2
    exit 1
  fi
  echo "$out" | tail -5
  if ! grep -qE '\.S +(FAIL|X-REACHED|MONITOR-ERROR|TIMEOUT|TRAP)' <<< "$out"; then
    echo "*** the $2 mutant was refused, but not by a program failing." >&2
    exit 1
  fi
}

expect_red late_addresses "operand addresses presented a cycle late" \
  's/cpu_state <= fetch_rs1;/cpu_state <= execute_instr;/'
expect_red swapped_ports "read ports' addresses swapped" \
  's/\.ra_addr({1.b0, rs1\[3:0\]}),/.ra_addr({1'"'"'b0, rs2[3:0]}),/; s/\.rb_addr({1.b0, rs2\[3:0\]}),/.rb_addr({1'"'"'b0, rs1[3:0]}),/'

echo
echo "Both mutants of nano's use of the register file's registered reads fail the suite."

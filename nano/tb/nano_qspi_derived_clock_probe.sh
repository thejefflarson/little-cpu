#!/bin/bash
# Forces the two-leg agreement check below to catch a design-internal derived clock:
# mutates the flash model to clock its always_ff off `sck` (a wire) instead of `clk`.
# cxxrtl never fires such an edge, so the cxxrtl leg traps almost immediately while
# iverilog runs the program through, and the retire counts must disagree. NOT HERMETIC --
# runs the real cross compiler, yosys, clang++ and iverilog; prerequisite of the test below.
set -euo pipefail

if [ "$#" -ne 3 ]; then
  echo "usage: nano_qspi_derived_clock_probe.sh <cflags> <rtl-srcs> <riscv-formal-macros>" >&2
  exit 2
fi
CFLAGS=$1
# shellcheck disable=SC2206
RTL_SRCS=($2)
# shellcheck disable=SC2206
FORMAL_MACROS=($3)
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WORKDIR="$HERE/nano-qspi-derived-clock-probe"

for name in "${RTL_SRCS[@]}" nano/tb/nano_testbench.v nano/tb/nano_cxxrtl.cc nano/tb/nano_dual_leg_test.sh \
            rvfi_macros.vh test/monitor.sim.v; do
  if [ ! -f "$REPO/$name" ]; then
    echo "error: $name is missing from $REPO. rvfi_macros.vh and test/monitor.sim.v are" \
      "make targets of their own -- run 'make rvfi_macros.vh test/monitor.sim.v' first." >&2
    exit 2
  fi
done
for tool in yosys clang++ iverilog; do
  if ! command -v "$tool" >/dev/null 2>&1; then
    echo "error: $tool is not on PATH." >&2
    exit 2
  fi
done

rm -rf "$WORKDIR"
mkdir -p "$WORKDIR"

mutant="$WORKDIR/nano_qspi_flash_model.mutant.v"
sed "s/always_ff @(posedge clk) begin/always_ff @(posedge sck) begin/" \
  "$REPO/nano/tb/nano_qspi_flash_model.v" > "$mutant"
if cmp -s "$REPO/nano/tb/nano_qspi_flash_model.v" "$mutant"; then
  echo "error: nano_qspi_flash_model.v no longer spells its top-level clocking the way" \
    "this probe mutates. Re-anchor the sed pattern on the new spelling." >&2
  exit 1
fi

sources_for() {  # $1 = flash model source to substitute in RTL_SRCS
  local flash_v=$1 src
  for src in "${RTL_SRCS[@]}"; do
    if [ "$src" = "nano/tb/nano_qspi_flash_model.v" ]; then
      echo "$flash_v"
    else
      echo "$REPO/$src"
    fi
  done
}

build_legs() {  # $1 = flash model source, $2 = tag -> $WORKDIR/<tag>-sim, $WORKDIR/<tag>-icarus.vvp
  local flash_v=$1 tag=$2
  local -a sources=()
  while IFS= read -r line; do sources+=("$line"); done < <(sources_for "$flash_v")
  local -a defines=()
  local define
  for define in "${FORMAL_MACROS[@]}"; do
    defines+=(-D "$define")
  done

  yosys -p "read_verilog -sv ${defines[*]} -D NANO_QSPI_PINS $REPO/rvfi_macros.vh ${sources[*]} $REPO/nano/tb/nano_testbench.v $REPO/test/monitor.sim.v; hierarchy -top nano_testbench; write_cxxrtl $WORKDIR/$tag.rtl.cc" \
    > "$WORKDIR/$tag.yosys.log" 2>&1 || { tail -60 "$WORKDIR/$tag.yosys.log" >&2; exit 1; }
  clang++ -O2 -DNDEBUG -std=c++17 -Wall -Wextra -Werror \
    -DNANO_RTL_INCLUDE="\"$WORKDIR/$tag.rtl.cc\"" \
    -isystem "$(yosys-config --datdir)/include/backends/cxxrtl/runtime" -I "$REPO/nano/tb" \
    "$REPO/nano/tb/nano_cxxrtl.cc" -o "$WORKDIR/$tag-sim" \
    > "$WORKDIR/$tag.build.log" 2>&1 || { tail -60 "$WORKDIR/$tag.build.log" >&2; exit 1; }
  iverilog -I./rtl/ -DICARUS -DNANO_QSPI_PINS "${defines[@]}" -g2012 \
    -o "$WORKDIR/$tag-icarus.vvp" "$REPO/rvfi_macros.vh" "${sources[@]}" \
    "$REPO/nano/tb/nano_testbench.v" "$REPO/test/monitor.sim.v"
}

run_dual_leg() {  # $1 = tag -> stdout, return code is nano_dual_leg_test.sh's
  local tag=$1
  (cd "$REPO" && NANO_VVP_IMAGE="$WORKDIR/$tag-icarus.vvp" \
    "$REPO/nano/tb/nano_dual_leg_test.sh" "$WORKDIR/$tag-sim" \
    "$REPO/nano/tb/nano_sim_icarus.sh" nano/asm nano/asm/EXPECTED_FAIL nano/asm/OBSERVED_FLOOR "$CFLAGS")
}

echo "shipping (single-clock models):"
build_legs "$REPO/nano/tb/nano_qspi_flash_model.v" shipping
set +e
shipping_out=$(run_dual_leg shipping 2>&1)
shipping_rc=$?
set -e
echo "$shipping_out"
if [ "$shipping_rc" -ne 0 ]; then
  echo "*** the shipping models do not agree leg to leg -- the control this probe relies" \
    "on, so a mutant disagreeing the same way would prove nothing." >&2
  exit 1
fi

echo
echo "mutant (flash model clocked off sck, a design-internal derived clock):"
build_legs "$mutant" mutant
set +e
mutant_out=$(run_dual_leg mutant 2>&1)
mutant_rc=$?
set -e
echo "$mutant_out"
if [ "$mutant_rc" -eq 0 ]; then
  echo "*** RED PROBE FAILED: the two legs still agreed with a design-internal derived" \
    "clock in the flash model -- nothing would catch cxxrtl going structurally blind." >&2
  exit 1
fi

echo
echo "shipping's two legs agree; the mutant's disagree: nano-qspi-pins-test is a real" \
  "grader against a structurally blind cxxrtl leg."

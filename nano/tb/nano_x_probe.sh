#!/bin/bash
# Forces nano's iverilog leg (the only four-state one) to catch an X on the load path
# (a never-written word) and the store path (a never-written register), and requires the
# shipping harness to stay clean first. NOT HERMETIC -- runs the real cross compiler and
# iverilog; prerequisite of `make nano-test`. test/probe_gates.sh covers its own logic.
set -euo pipefail

if [ "$#" -ne 3 ]; then
  echo "usage: nano_x_probe.sh <cflags> <rtl-srcs> <riscv-formal-macros>" >&2
  exit 2
fi
CFLAGS=$1
# shellcheck disable=SC2206
RTL_SRCS=($2)
# shellcheck disable=SC2206
FORMAL_MACROS=($3)
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WORKDIR="$HERE/nano-x-probe"

for name in "${RTL_SRCS[@]}" nano/tb/nano_testbench.v rvfi_macros.vh test/monitor.sim.v; do
  if [ ! -f "$REPO/$name" ]; then
    echo "error: $name is missing from $REPO. rvfi_macros.vh and test/monitor.sim.v are" \
      "make targets of their own -- run 'make rvfi_macros.vh test/monitor.sim.v' first." >&2
    exit 2
  fi
done

CC=""
if command -v riscv-none-elf-gcc >/dev/null 2>&1; then
  CC=riscv-none-elf-gcc
fi
if [ -z "$CC" ]; then
  echo "error: no RISC-V cross compiler found (want riscv-none-elf-gcc)." >&2
  exit 2
fi
OBJCOPY=${CC%gcc}objcopy
if ! command -v "$OBJCOPY" >/dev/null 2>&1; then
  echo "error: found $CC but not its matching $OBJCOPY." >&2
  exit 2
fi
if ! command -v iverilog >/dev/null 2>&1; then
  echo "error: iverilog is not on PATH." >&2
  exit 2
fi

rm -rf "$WORKDIR"
mkdir -p "$WORKDIR"

# An address neither this ELF's .text nor its .data/.bss reaches, so the word behind it
# stays whatever the simulated memory reset it to.
cat > "$WORKDIR/xprobe.S" <<'ASM'
// nano-local: every register below is x0-x15.
#include "riscv_test.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  li x1, 0x00013000
  lw x6, 0(x1)

  RVTEST_PASS

fail:
  RVTEST_FAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN
RVTEST_DATA_END
ASM

# x1 is never written by RVTEST_CODE_BEGIN (gp) or RVTEST_PASS/FAIL (t0), so it carries
# whatever the register file reset it to; storing it exercises the write-mask-masked
# mem_wdata term rather than the load-path terms the probe above already covers.
cat > "$WORKDIR/store_x.S" <<'ASM'
#include "riscv_test.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  li x2, 0x00013000
  sw x1, 0(x2)

  RVTEST_PASS

fail:
  RVTEST_FAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN
RVTEST_DATA_END
ASM

cat > "$WORKDIR/store_clean.S" <<'ASM'
#include "riscv_test.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  li x2, 0x00013000
  li x1, 0x12345678
  sw x1, 0(x2)

  RVTEST_PASS

fail:
  RVTEST_FAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN
RVTEST_DATA_END
ASM

assemble() {  # $1 = source basename without .S
  # shellcheck disable=SC2086
  "$CC" $CFLAGS -nostdlib -I "$REPO/nano/asm" -I "$REPO/test/asm" \
    -T "$REPO/nano/asm/nano.lds" "$WORKDIR/$1.S" -o "$WORKDIR/$1.elf"
  "$OBJCOPY" -O verilog --verilog-data-width=4 --only-section=.text \
    "$WORKDIR/$1.elf" "$WORKDIR/$1.rom.hex"
  "$OBJCOPY" -O verilog --verilog-data-width=4 --remove-section=.text \
    "$WORKDIR/$1.elf" "$WORKDIR/$1.ram.hex"
}

assemble xprobe
assemble store_x
assemble store_clean

build_vvp() {  # $1 = nano_testbench.v to use, $2 = vvp output path
  local tb=$1 vvp_out=$2 define src
  local -a defines=(-DICARUS) sources=("$REPO/rvfi_macros.vh")
  for define in "${FORMAL_MACROS[@]}"; do
    defines+=(-D "$define")
  done
  for src in "${RTL_SRCS[@]}"; do
    sources+=("$REPO/$src")
  done
  iverilog -g2012 "${defines[@]}" -o "$vvp_out" \
    "${sources[@]}" "$tb" "$REPO/test/monitor.sim.v"
}

run_image() {  # $1 = vvp path, $2 = program basename (its .rom.hex/.ram.hex)
  (cd "$WORKDIR" && vvp "$1" "+ROM=$WORKDIR/$2.rom.hex" \
    "+RAM=$WORKDIR/$2.ram.hex" +CYCLES=200) 2>&1
}

red=()

build_vvp "$REPO/nano/tb/nano_testbench.v" "$WORKDIR/shipping.vvp"

shipping_out=$(run_image "$WORKDIR/shipping.vvp" xprobe)
echo "shipping, load path:"
echo "$shipping_out"
if grep -q '^X reached a retiring instruction' <<< "$shipping_out"; then
  red+=("the shipping harness itself reports an X reading an unzeroed word. That is what
nano_testbench.v's zeroing loop is meant to prevent, so a control that starts
red proves nothing about the mutant below.")
fi

store_clean_out=$(run_image "$WORKDIR/shipping.vvp" store_clean)
echo
echo "shipping, store path, x1 written first:"
echo "$store_clean_out"
if grep -q '^X reached a retiring instruction' <<< "$store_clean_out"; then
  red+=("storing a register this program itself wrote first still reports an X. That
is not evidence the store-path term works -- it is evidence it fires on
anything, which is not a control at all.")
fi

store_x_out=$(run_image "$WORKDIR/shipping.vvp" store_x)
echo
echo "shipping, store path, x1 never written:"
echo "$store_x_out"
if ! grep -q '^X reached a retiring instruction' <<< "$store_x_out"; then
  red+=("storing x1 before this program ever writes it does not report an X. nano.v
never resets regs[1]-regs[15], so this is a real X reaching mem_wdata; the
store-path (write-mask-masked mem_wdata) term of the check is not catching it.")
fi

mutant_tb="$WORKDIR/nano_testbench.mutant.v"
if ! sed 's/for (int unsigned i = 0; i < MEM_WORDS; i = i + 1) mem\.mem\[i\] = 32.b0;/if (0) mem.mem[0] = 32'"'"'b0;/' \
  "$REPO/nano/tb/nano_testbench.v" > "$mutant_tb"; then
  echo "error: could not write the mutant testbench." >&2
  exit 2
fi
if cmp -s "$REPO/nano/tb/nano_testbench.v" "$mutant_tb"; then
  echo "error: nano_testbench.v no longer spells its zeroing loop the way this probe
mutates. Re-anchor the sed pattern on the new spelling -- left alone this
would build the shipping harness twice and prove nothing about skipped
zeroing." >&2
  exit 2
fi

build_vvp "$mutant_tb" "$WORKDIR/mutant.vvp"
mutant_out=$(run_image "$WORKDIR/mutant.vvp" xprobe)
echo
echo "mutant (zeroing loop skipped), load path:"
echo "$mutant_out"
if ! grep -q '^X reached a retiring instruction' <<< "$mutant_out"; then
  red+=("the mutant with no zeroing loop does not report an X. Either the probe's own
read at 0x00013000 no longer lands on an untouched word, or the X-detection
check in nano_testbench.v's ICARUS block stopped catching one.")
fi

if [ "${#red[@]}" -ne 0 ]; then
  echo
  for why in "${red[@]}"; do
    echo "*** $why" >&2
  done
  exit 1
fi

echo
echo "The load and store paths both report an X exactly when one is really there."

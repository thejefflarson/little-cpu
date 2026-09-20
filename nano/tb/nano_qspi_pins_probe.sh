#!/bin/bash
# Forces the pin-level QSPI harness to catch a broken partial-word store: nano/qspi.v's
# read-modify-write merge picks the wrong bytes, and nano/asm/loadstore.S -- which
# stores and reads back individual bytes and halfwords through this same PSRAM model --
# must fail against it. Requires the shipping harness to pass loadstore.S first.
# NOT HERMETIC -- runs the real cross compiler and iverilog; prerequisite of
# `make nano-qspi-pins-test`. test/probe_gates.sh covers its own logic.
set -euo pipefail

if [ "$#" -ne 3 ]; then
  echo "usage: nano_qspi_pins_probe.sh <cflags> <rtl-srcs> <riscv-formal-macros>" >&2
  exit 2
fi
CFLAGS=$1
# shellcheck disable=SC2206
RTL_SRCS=($2)
# shellcheck disable=SC2206
FORMAL_MACROS=($3)
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WORKDIR="$HERE/nano-qspi-pins-probe"

for name in "${RTL_SRCS[@]}" nano/tb/nano_testbench.v nano/asm/loadstore.S nano/asm/nano.lds \
            rvfi_macros.vh test/monitor.sim.v; do
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

# shellcheck disable=SC2086
"$CC" $CFLAGS -nostdlib -I "$REPO/nano/asm" -I "$REPO/test/asm" \
  -T "$REPO/nano/asm/nano.lds" "$REPO/nano/asm/loadstore.S" -o "$WORKDIR/loadstore.elf"
"$OBJCOPY" -O verilog --verilog-data-width=4 --only-section=.text \
  "$WORKDIR/loadstore.elf" "$WORKDIR/loadstore.rom.hex"
"$OBJCOPY" -O verilog --verilog-data-width=4 --remove-section=.text \
  "$WORKDIR/loadstore.elf" "$WORKDIR/loadstore.ram.hex"

build_vvp() {  # $1 = qspi.v to use, $2 = vvp output path
  local qspi_v=$1 vvp_out=$2 define src
  local -a defines=(-DICARUS -DNANO_QSPI_PINS) sources=("$REPO/rvfi_macros.vh")
  for define in "${FORMAL_MACROS[@]}"; do
    defines+=(-D "$define")
  done
  for src in "${RTL_SRCS[@]}"; do
    if [ "$src" = "nano/qspi.v" ]; then
      sources+=("$qspi_v")
    else
      sources+=("$REPO/$src")
    fi
  done
  iverilog -I./rtl/ -g2012 "${defines[@]}" -o "$vvp_out" \
    "${sources[@]}" "$REPO/nano/tb/nano_testbench.v" "$REPO/test/monitor.sim.v"
}

run_image() {  # $1 = vvp path
  # A redirect over QSPI costs tens of cycles rather than one, so this program's own
  # 44-retire floor needs far more than the zero-wait suite's usual few thousand.
  (cd "$WORKDIR" && vvp "$1" "+ROM=$WORKDIR/loadstore.rom.hex" \
    "+RAM=$WORKDIR/loadstore.ram.hex" +CYCLES=200000) 2>&1
}

red=()

build_vvp "$REPO/nano/qspi.v" "$WORKDIR/shipping.vvp"
shipping_out=$(run_image "$WORKDIR/shipping.vvp")
echo "shipping:"
echo "$shipping_out"
if ! grep -q '^PASS$' <<< "$shipping_out"; then
  red+=("the shipping controller does not pass loadstore.S over the pin-level models.
That is the control 'make nano-qspi-pins-test' relies on, so a mutant failing
the same way would prove nothing.")
fi

# Invert which bytes the read-modify-write merge takes from the store versus the word
# it just read back, so a partial-word store silently keeps the OLD byte and discards
# the new one.
mutant_qspi="$WORKDIR/qspi.mutant.v"
if ! sed "s/(psram_wdata_pending & psram_byte_mask) |/(psram_wdata_pending \& ~psram_byte_mask) |/;
          s/(rx_shift & ~psram_byte_mask);/(rx_shift \& psram_byte_mask);/" \
  "$REPO/nano/qspi.v" > "$mutant_qspi"; then
  echo "error: could not write the mutant qspi.v." >&2
  exit 2
fi
if cmp -s "$REPO/nano/qspi.v" "$mutant_qspi"; then
  echo "error: nano/qspi.v no longer spells its read-modify-write merge the way this
probe mutates. Re-anchor the sed pattern on the new spelling -- left alone
this would build the shipping controller twice and prove nothing about a
swapped merge." >&2
  exit 2
fi

build_vvp "$mutant_qspi" "$WORKDIR/mutant.vvp"
mutant_out=$(run_image "$WORKDIR/mutant.vvp")
echo
echo "mutant (read-modify-write merge inverted):"
echo "$mutant_out"
if grep -q '^PASS$' <<< "$mutant_out"; then
  red+=("the mutant with an inverted read-modify-write merge still passes loadstore.S.
Either the program no longer exercises a partial-word store through PSRAM, or
the merge bug is not reaching an observable result.")
fi

if [ "${#red[@]}" -ne 0 ]; then
  echo
  for why in "${red[@]}"; do
    echo "*** $why" >&2
  done
  exit 1
fi

echo
echo "loadstore.S passes the shipping controller and fails the inverted-merge mutant."

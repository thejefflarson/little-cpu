#!/bin/bash
# Requires meip.S's retire floor to catch a program that reports PASS while skipping its
# checks. Not hermetic: runs the real cross compiler and nano-sim.
set -euo pipefail

if [ "$#" -ne 2 ]; then
  echo "usage: nano_meip_floor_probe.sh <nano-sim> <cflags>" >&2
  exit 2
fi
SIM=$1
CFLAGS=$2
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WORKDIR="$HERE/nano-meip-floor-probe"

if [ ! -x "$SIM" ]; then
  echo "error: '$SIM' is not an executable runner; build it with 'make nano-sim'." >&2
  exit 2
fi

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

floor=$(awk '$1 == "meip.S" { print $2; exit }' "$REPO/nano/asm/OBSERVED_FLOOR")
if [ -z "$floor" ]; then
  echo "error: meip.S has no line in $REPO/nano/asm/OBSERVED_FLOOR." >&2
  exit 2
fi

rm -rf "$WORKDIR"
mkdir -p "$WORKDIR/asm"

# Arms the interrupt, then reports PASS at once, skipping the wait and both checks: it
# still says PASS, so only the retire count can tell it from the shipping test.
cat > "$WORKDIR/asm/meip.S" <<'ASM'
// nano-local, x0-x15. Probe-only mutant of meip.S: arms MEIP then reports PASS with
// none of the shipping test's own verification, to show that retires fewer
// instructions than its floor.
#include "riscv_test.h"

#define MIE_MEIE 0x800
#define MSTATUS_MIE 0x8

RVTEST_RV64U
RVTEST_CODE_BEGIN

  la t0, trap_handler
  csrw mtvec, t0

  la t2, irq_count
  sw x0, 0(t2)

  li t0, MIE_MEIE
  csrs mie, t0
  li t0, MSTATUS_MIE
  csrs mstatus, t0

  la a2, __irqctl
  li a3, 1
  sw a3, 0(a2)

  RVTEST_PASS

fail:
  RVTEST_FAIL

  .align 2
trap_handler:
  .option push
  .option norvc
  li t0, MIE_MEIE
  csrc mie, t0
  mret
  .option pop

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN
  .align 2
  .global irq_count
irq_count:
  .word 0
RVTEST_DATA_END
ASM

# shellcheck disable=SC2086
"$CC" $CFLAGS -nostdlib -I "$WORKDIR/asm" -I "$REPO/test/asm" \
  -T "$REPO/nano/asm/nano.lds" "$WORKDIR/asm/meip.S" -o "$WORKDIR/meip.elf"
"$OBJCOPY" -O verilog --verilog-data-width=4 --only-section=.text \
  "$WORKDIR/meip.elf" "$WORKDIR/meip.rom.hex"
"$OBJCOPY" -O verilog --verilog-data-width=4 --remove-section=.text \
  "$WORKDIR/meip.elf" "$WORKDIR/meip.ram.hex"

out=$("$SIM" --rom "$WORKDIR/meip.rom.hex" --ram "$WORKDIR/meip.ram.hex" --cycles 5000)
echo "$out"

if ! grep -q '^PASS$' <<< "$out"; then
  echo "error: the probe program did not PASS, so it cannot show what a false PASS" >&2
  echo "would retire. Its own arming sequence may have diverged from meip.S's." >&2
  exit 2
fi

retires=$(awk '/^RETIRES /{print $2; exit}' <<< "$out")
if [ -z "$retires" ]; then
  echo "error: the probe run printed no RETIRES line." >&2
  exit 2
fi

if [ "$retires" -ge "$floor" ]; then
  echo "*** the probe retires $retires, at or above meip.S's floor of $floor -- a" >&2
  echo "*** regression that reports PASS while skipping the shipping test's own" >&2
  echo "*** verification would NOT be caught by that floor." >&2
  exit 1
fi

echo
echo "the probe retires $retires against a floor of $floor: a regression that reports" \
     "PASS while skipping meip.S's own verification is BELOW-FLOOR."

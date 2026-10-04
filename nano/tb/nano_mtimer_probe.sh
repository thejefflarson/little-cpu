#!/bin/bash
# Requires nano's iverilog leg to fail mtimer.S or mtimerorder.S when the timer interrupt breaks one way at a time: the cause code, external-over-timer priority, mie.MTIE gating or its write path, mip.MTIP, a dead comparator and a low-word-only one.
# Not hermetic: runs the real cross compiler and iverilog.
set -euo pipefail

if [ "$#" -ne 3 ]; then
  echo "usage: nano_mtimer_probe.sh <cflags> <rtl-srcs> <riscv-formal-macros>" >&2
  exit 2
fi
CFLAGS=$1
# shellcheck disable=SC2206
RTL_SRCS=($2)
# shellcheck disable=SC2206
FORMAL_MACROS=($3)
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WORKDIR="$HERE/nano-mtimer-probe"

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

build_vvp() {  # $1 = nano.v to use, $2 = timer.v to use, $3 = vvp output path
  local define src
  local -a defines=(-DICARUS) sources=("$REPO/rvfi_macros.vh")
  for define in "${FORMAL_MACROS[@]}"; do
    defines+=(-D "$define")
  done
  for src in "${RTL_SRCS[@]}"; do
    case "$src" in
      nano/nano.v)  sources+=("$1") ;;
      nano/timer.v) sources+=("$2") ;;
      *)            sources+=("$REPO/$src") ;;
    esac
  done
  iverilog -g2012 "${defines[@]}" -o "$3" \
    "${sources[@]}" "$REPO/nano/tb/nano_testbench.v" "$REPO/test/monitor.sim.v"
}

run_suite() {  # $1 = vvp image; prints the suite and returns its status
  NANO_VVP_IMAGE=$1 "$REPO/nano/asm/run_nano_tests.sh" "$REPO/nano/tb/nano_sim_icarus.sh" \
    "$REPO/nano/asm" "$REPO/nano/asm/EXPECTED_FAIL" "$REPO/nano/asm/OBSERVED_FLOOR" "$CFLAGS" \
    "$REPO/nano/asm/nano.lds" 10000
}

echo "control: the shipping nano.v and timer.v"
build_vvp "$REPO/nano/nano.v" "$REPO/nano/timer.v" "$WORKDIR/shipping.vvp"
if ! out=$(run_suite "$WORKDIR/shipping.vvp" 2>&1); then
  echo "$out"
  echo "*** the shipping build fails its own suite, so a mutant failing it proves nothing." >&2
  exit 1
fi
echo "$out" | tail -3

expect_red() {  # $1 = stem, $2 = what the mutant does, $3 = nano.v or timer.v, $4 = sed program
  local mutant_nano="$REPO/nano/nano.v" mutant_timer="$REPO/nano/timer.v"
  sed "$4" "$REPO/nano/$3" > "$WORKDIR/$1.v"
  if cmp -s "$REPO/nano/$3" "$WORKDIR/$1.v"; then
    echo "error: nano/$3 no longer spells the line the $2 mutant rewrites. Re-anchor it --" \
         "left alone this builds the shipping core twice and proves nothing." >&2
    exit 2
  fi
  if [ "$3" = nano.v ]; then mutant_nano="$WORKDIR/$1.v"; else mutant_timer="$WORKDIR/$1.v"; fi
  build_vvp "$mutant_nano" "$mutant_timer" "$WORKDIR/$1.vvp"
  echo
  echo "mutant: $2"
  if out=$(run_suite "$WORKDIR/$1.vvp" 2>&1); then
    echo "$out" | tail -5
    echo "*** the $2 mutant passed the suite." >&2
    exit 1
  fi
  if ! grep -qE '^mtimer(order)?\.S +(FAIL|X-REACHED|MONITOR-ERROR|TIMEOUT|TRAP)' <<< "$out"; then
    echo "$out" | tail -12
    echo "*** the $2 mutant was refused, but not by mtimer.S or mtimerorder.S failing." >&2
    exit 1
  fi
  grep -E '^mtimer(order)?\.S ' <<< "$out"
}

expect_red cause "the timer interrupt reports cause 3" nano.v \
  's/CAUSE_MACHINE_TIMER       = 32.h8000_0007/CAUSE_MACHINE_TIMER       = 32'"'"'h8000_0003/'
expect_red priority "the timer outranks the external interrupt" nano.v \
  's/external_pending ? CAUSE_MACHINE_EXTERNAL\[3:0\] : CAUSE_MACHINE_TIMER\[3:0\]/external_pending ? CAUSE_MACHINE_TIMER[3:0] : CAUSE_MACHINE_EXTERNAL[3:0]/'
expect_red ungated "mie.MTIE no longer gates the timer interrupt" nano.v \
  's/(irq_mtip && mie_mtie)/irq_mtip/'
expect_red mie_write "mie.MTIE is written from the wrong bit" nano.v \
  's/mie_mtie <= csr_new_value\[7\];/mie_mtie <= csr_new_value[3];/'
expect_red mip_read "mip.MTIP reads zero" nano.v \
  's/irq_meip_sync2, 3.b0, irq_mtip, 7.b0/irq_meip_sync2, 3'"'"'b0, 1'"'"'b0, 7'"'"'b0/'
expect_red dead "mtip never posts" timer.v \
  's/mtip <= {time_hi, time_lo} >= {cmp_hi, cmp_lo};/mtip <= 1'"'"'b0;/'
expect_red low_word "mtip compares the low words only" timer.v \
  's/mtip <= {time_hi, time_lo} >= {cmp_hi, cmp_lo};/mtip <= time_lo >= cmp_lo;/'

echo
echo "All seven timer-interrupt mutants fail mtimer.S or mtimerorder.S."

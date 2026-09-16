#!/bin/bash
# Grades DHRY_BOARD_EXTRA_DEFINES as `dhrystone-rom`'s only allowed flag difference from
# `make dhrystone`: dhry_1.o/dhry_2.o, which read no DHRY_UART, must come out identical.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=${1:-$(cd "$HERE/.." && pwd)}
BENCH="$REPO/test/bench"

if [ ! -d "$REPO" ]; then
  echo "error: '$REPO' is not a directory, so there is nothing to build." >&2
  exit 1
fi

CC=""
if command -v riscv-none-elf-gcc >/dev/null 2>&1; then
  CC=riscv-none-elf-gcc
fi
if [ -z "$CC" ]; then
  echo "error: no RISC-V cross compiler found (want riscv-none-elf-gcc)." >&2
  echo "Run 'make riscv-gcc-setup' to install the pinned one." >&2
  exit 1
fi

dhry_vars=$(cd "$REPO" && make -s \
  print-DHRY_CFLAGS print-DHRY_BOARD_CFLAGS print-DHRY_BOARD_EXTRA_DEFINES)
sim_cflags=$(sed -n '1p' <<< "$dhry_vars")
board_cflags=$(sed -n '2p' <<< "$dhry_vars")
extra_defines=$(sed -n '3p' <<< "$dhry_vars")

if [ -z "$sim_cflags" ] || [ -z "$board_cflags" ] || [ -z "$extra_defines" ]; then
  echo "error: could not read DHRY_CFLAGS, DHRY_BOARD_CFLAGS or" >&2
  echo "DHRY_BOARD_EXTRA_DEFINES out of $REPO/Makefile via 'make print-<VAR>'." >&2
  exit 1
fi

if [ "$sim_cflags" != "$board_cflags" ]; then
  echo "error: DHRY_BOARD_CFLAGS no longer matches DHRY_CFLAGS." >&2
  echo "  DHRY_CFLAGS      : $sim_cflags" >&2
  echo "  DHRY_BOARD_CFLAGS: $board_cflags" >&2
  echo "DHRY_BOARD_EXTRA_DEFINES is meant to be the only difference between the" >&2
  echo "board build and the simulated one; a second one here would move the" >&2
  echo "board's number for a reason it does not carry." >&2
  exit 1
fi

tmp=$(mktemp -d "${TMPDIR:-/tmp}/dhry-parity.XXXXXX") || {
  echo "error: could not create a temporary directory under ${TMPDIR:-/tmp}." >&2
  exit 1
}
trap 'rm -rf "$tmp"' EXIT

# shellcheck disable=SC2089
common_defines='-DDHRY_RUNS=5 -DDHRY_FLAGS="dhry-board-parity-check"'

build() {
  local outdir=$1 extra=$2 unit
  mkdir -p "$outdir"
  for unit in dhry_1 dhry_2 dhry_port; do
    # shellcheck disable=SC2086,SC2090
    "$CC" $sim_cflags $common_defines $extra -c "$BENCH/$unit.c" -o "$outdir/$unit.o"
  done
}

build "$tmp/sim" ""
build "$tmp/board" "$extra_defines"

fail=0
for unit in dhry_1 dhry_2; do
  if ! cmp -s "$tmp/sim/$unit.o" "$tmp/board/$unit.o"; then
    fail=1
    echo "error: $unit.o differs between the sim and board builds, and $unit.c" >&2
    echo "reads no DHRY_UART -- something besides DHRY_BOARD_EXTRA_DEFINES is" >&2
    echo "diverging between the two builds' flags." >&2
  fi
done

if cmp -s "$tmp/sim/dhry_port.o" "$tmp/board/dhry_port.o"; then
  fail=1
  echo "error: dhry_port.o came out byte-identical with and without" >&2
  echo "DHRY_BOARD_EXTRA_DEFINES. That define is supposed to gate real code --" >&2
  echo "uart_putc's busy-wait and the report's repeat loop -- so if it no longer" >&2
  echo "changes anything, the board build differs from the simulated one for a" >&2
  echo "reason this check can no longer see, which is worse than the reason it" >&2
  echo "used to have." >&2
fi

if [ "$fail" -ne 0 ]; then
  exit 1
fi

echo "dhry_1.o and dhry_2.o are byte-identical between the sim and board builds;" \
     "dhry_port.o differs only through DHRY_BOARD_EXTRA_DEFINES."

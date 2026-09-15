#!/bin/bash
# Grades nano/bench/start.S's gp initialisation by tohost: a few-hundred-cycle PASS/FAIL on
# `make test`'s path, not a benchmark. An unset gp once trapped both benchmarks mid-.bss-zero.
set -euo pipefail

if [ "$#" -ne 2 ]; then
  echo "usage: run_startup_test.sh <sim-binary> <nano-march-cflags>" >&2
  exit 1
fi

SIM=$1
CFLAGS="$2 -O2 -std=c11 -ffreestanding -Wall -Wextra -Werror"
HERE=$(cd "$(dirname "$0")" && pwd)

if [ ! -x "$SIM" ]; then
  echo "error: '$SIM' is not an executable runner; build it with 'make nano-sim'." >&2
  exit 1
fi

CC=""
if command -v riscv-none-elf-gcc >/dev/null 2>&1; then
  CC=riscv-none-elf-gcc
fi
if [ -z "$CC" ]; then
  echo "error: no RISC-V cross compiler found; see 'make riscv-gcc-setup'." >&2
  exit 1
fi
OBJCOPY=${CC%gcc}objcopy

tmp=$(mktemp -d "${TMPDIR:-/tmp}/nano-startup-test.XXXXXX")
test -n "$tmp" -a -d "$tmp"
trap 'rm -rf "$tmp"' EXIT

elf="$tmp/startup_test.elf"
# shellcheck disable=SC2086
if ! $CC $CFLAGS -nostdlib -T "$HERE/startup_test.lds" -o "$elf" \
     "$HERE/start.S" "$HERE/startup_test.c" 2> "$tmp/build.log"; then
  cat "$tmp/build.log" >&2
  exit 1
fi
if [ -s "$tmp/build.log" ]; then
  cat "$tmp/build.log" >&2
  echo "error: the build produced diagnostics; warnings are errors here." >&2
  exit 1
fi

$OBJCOPY -O verilog --verilog-data-width=4 --only-section=.text "$elf" "$tmp/rom.hex"
$OBJCOPY -O verilog --verilog-data-width=4 --remove-section=.text "$elf" "$tmp/ram.hex"

set +e
"$SIM" --rom "$tmp/rom.hex" --ram "$tmp/ram.hex" --cycles 2000 > "$tmp/run.log" 2>&1
sim_status=$?
set -e
cat "$tmp/run.log"
if [ "$sim_status" -ne 0 ]; then
  echo "*** gp is not initialized before nano/bench/start.S's first gp-relative reference" \
    "(runner exit $sim_status)." >&2
  exit "$sim_status"
fi
echo "nano/bench/start.S initializes gp before any gp-relative reference runs."

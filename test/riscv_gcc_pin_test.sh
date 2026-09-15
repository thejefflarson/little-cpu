#!/bin/bash
# Confirms a build resolves the pinned riscv-none-elf-gcc, not some other one earlier on
# PATH, which is what a real build actually obeys.
set -euo pipefail

if [ "$#" -ne 1 ]; then
  echo "usage: riscv_gcc_pin_test.sh <pinned-riscv-gcc-bin-dir>" >&2
  exit 1
fi

PINNED_BIN=$1

resolved=$(command -v riscv-none-elf-gcc) || {
  echo "error: riscv-none-elf-gcc is not on PATH; run \`make riscv-gcc-setup\`." >&2
  exit 1
}

want="$PINNED_BIN/riscv-none-elf-gcc"
if [ "$resolved" != "$want" ]; then
  echo "error: PATH resolves riscv-none-elf-gcc to a different install:" >&2
  echo "  resolved : $resolved" >&2
  echo "  pinned   : $want" >&2
  echo "A build made with this PATH would compile the suite and the" >&2
  echo "benchmarks with an unpinned compiler." >&2
  exit 1
fi

echo "riscv-none-elf-gcc resolves to the pinned install: $resolved"

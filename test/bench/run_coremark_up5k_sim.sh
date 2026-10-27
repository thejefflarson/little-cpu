#!/bin/bash
# Runs the up5k CoreMark ELF under cxxrtl, dropping the per-access trace (gigabytes at 800 iterations).
set -euo pipefail

if [ "$#" -ne 4 ]; then
  echo "usage: run_coremark_up5k_sim.sh <sim> <elf> <cycle-limit> <out-log>" >&2
  exit 1
fi
SIM=$1 ELF=$2 CYCLES=$3 OUT=$4

dir=$(dirname "$OUT")
base="$dir/$(basename "$ELF" .elf)"
riscv-none-elf-objcopy -O verilog --verilog-data-width=4 -j .text -j .data "$ELF" "$base.rom.hex"
riscv-none-elf-objcopy -O verilog --verilog-data-width=4 -j .tohost "$ELF" "$base.ram.hex"
console=$(riscv-none-elf-nm "$ELF" | awk '$3 == "coremark_console" { print "0x" $1 }')
if [ -z "$console" ]; then
  echo "error: coremark_console is not in $ELF." >&2
  exit 1
fi

set +e
"$SIM" --rom "$base.rom.hex" --ram "$base.ram.hex" --cycles "$CYCLES" --console "$console" \
  | awk '!/^(ifetch |write  |read   |trap!)/' > "$OUT"
status=("${PIPESTATUS[@]}")
set -e
if [ "${status[0]}" -ne 0 ] || [ "${status[1]}" -ne 0 ]; then
  cat "$OUT" >&2
  echo "error: the run did not reach a PASS verdict (runner exit ${status[0]}; 2 is the cycle limit)." >&2
  exit 1
fi
cat "$OUT"

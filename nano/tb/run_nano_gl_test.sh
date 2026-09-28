#!/bin/sh
# Runs tt_gpio_uart.S through a hardened netlist under iverilog and the pinned sky130_fd_sc_hd models; single-clock nano_tt_tb.v runs a gated netlist cxxrtl never could.
set -eu

CFLAGS=$1
NETLIST=$2
CELL_DIR=$3

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WORKDIR="$HERE/nano-gl-test"

if [ ! -f "$NETLIST" ]; then
  echo "error: no netlist at $NETLIST." >&2
  exit 2
fi
if [ ! -d "$CELL_DIR" ] || [ -z "$(ls -A "$CELL_DIR" 2>/dev/null)" ]; then
  echo "error: $CELL_DIR has no cell models -- run 'make nano-sky130-verilog-setup' first." >&2
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
if ! command -v iverilog >/dev/null 2>&1; then
  echo "error: iverilog is not on PATH." >&2
  exit 2
fi

python3 "$REPO/nano/gl_census.py" "$NETLIST" --require dlclkp

rm -rf "$WORKDIR"
mkdir -p "$WORKDIR"

# shellcheck disable=SC2086
"$CC" $CFLAGS -nostdlib -I "$REPO/test/asm" -T "$REPO/nano/tb/asm/nano_tt.lds" \
  "$REPO/nano/tb/asm/tt_gpio_uart.S" -o "$WORKDIR/tt_gpio_uart.elf"
"$OBJCOPY" -O verilog --verilog-data-width=4 --only-section=.text \
  "$WORKDIR/tt_gpio_uart.elf" "$WORKDIR/tt_gpio_uart.rom.hex"
"$OBJCOPY" -O verilog --verilog-data-width=4 --remove-section=.text \
  --adjust-vma=-0x10000000 "$WORKDIR/tt_gpio_uart.elf" "$WORKDIR/tt_gpio_uart.ram.hex"

iverilog -g2012 -D FUNCTIONAL -I "$CELL_DIR" -o "$WORKDIR/nano_gl.vvp" \
  "$NETLIST" \
  "$REPO/nano/tb/nano_qspi_flash_model.v" "$REPO/nano/tb/nano_qspi_psram_model.v" \
  "$REPO/nano/tb/nano_tt_tb.v"

cd "$WORKDIR"
OUT_LOG="$WORKDIR/gl_test.out"
set +e
if command -v /usr/bin/time >/dev/null 2>&1 && /usr/bin/time -v true >/dev/null 2>&1; then
  /usr/bin/time -v vvp nano_gl.vvp "+ROM=$WORKDIR/tt_gpio_uart.rom.hex" \
    "+RAM=$WORKDIR/tt_gpio_uart.ram.hex" > "$OUT_LOG" 2> "$WORKDIR/gl_test.time.log"
  rc=$?
  peak=$(grep 'Maximum resident set size' "$WORKDIR/gl_test.time.log" | awk '{print $NF}')
  [ -n "$peak" ] && echo "peak RSS: ${peak} KB"
else
  echo "note: GNU time -v is not on PATH, so this run does not report peak RSS." >&2
  vvp nano_gl.vvp "+ROM=$WORKDIR/tt_gpio_uart.rom.hex" "+RAM=$WORKDIR/tt_gpio_uart.ram.hex" > "$OUT_LOG"
  rc=$?
fi
set -e
cat "$OUT_LOG"
[ "$rc" -eq 0 ] || exit "$rc"
grep -q '^PASS$' "$OUT_LOG"

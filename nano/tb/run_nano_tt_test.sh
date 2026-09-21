#!/bin/sh
# Builds nano/tb/asm/tt_gpio_uart.S and runs it against the tt_um top through
# nano/tb/nano_tt_tb.v -- pins only, no peek at the core's internal bus.
set -eu

CFLAGS=$1
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WORKDIR="$HERE/nano-tt-test"

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
"$CC" $CFLAGS -nostdlib -I "$REPO/test/asm" -T "$REPO/nano/tb/asm/nano_tt.lds" \
  "$REPO/nano/tb/asm/tt_gpio_uart.S" -o "$WORKDIR/tt_gpio_uart.elf"
"$OBJCOPY" -O verilog --verilog-data-width=4 --only-section=.text \
  "$WORKDIR/tt_gpio_uart.elf" "$WORKDIR/tt_gpio_uart.rom.hex"
# nano_bus subtracts PSRAM_BASE before addressing the PSRAM model, so the model's own
# array is indexed from zero; --adjust-vma rebases the RAM image to match.
"$OBJCOPY" -O verilog --verilog-data-width=4 --remove-section=.text \
  --adjust-vma=-0x10000000 "$WORKDIR/tt_gpio_uart.elf" "$WORKDIR/tt_gpio_uart.ram.hex"

TT_TOP=${NANO_TT_TOP:-"$REPO/nano/tt/src/tt_um_thejefflarson_nanocpu.v"}

iverilog -g2012 -o "$WORKDIR/nano_tt.vvp" \
  "$REPO/nano/nano.v" "$REPO/nano/qspi.v" "$REPO/nano/uart.v" "$REPO/nano/gpio.v" \
  "$REPO/nano/bus.v" "$TT_TOP" \
  "$REPO/nano/tb/nano_qspi_flash_model.v" "$REPO/nano/tb/nano_qspi_psram_model.v" \
  "$REPO/nano/tb/nano_tt_tb.v"

cd "$WORKDIR"
out=$(vvp nano_tt.vvp "+ROM=$WORKDIR/tt_gpio_uart.rom.hex" "+RAM=$WORKDIR/tt_gpio_uart.ram.hex")
echo "$out"
printf '%s\n' "$out" | grep -q '^PASS$'

#!/bin/bash
# Asserts that the range decodes refuse to elaborate at a parameter shape they are not
# valid for -- in both frontends, and for the reason each is written for.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/.." && pwd)

for tool in iverilog yosys; do
  if ! command -v "$tool" >/dev/null 2>&1; then
    echo "error: $tool is not on PATH, so the elaboration checks in rtl/ cannot" >&2
    echo "be forced to fire. Skipping would report a green run for checks that" >&2
    echo "were never executed, which is what this file exists to prevent." >&2
    exit 1
  fi
done

tmp=$(mktemp -d "${TMPDIR:-/tmp}/littlecpu-window.XXXXXX")
test -n "$tmp" -a -d "$tmp"
trap 'rm -rf "$tmp"' EXIT

cases=0
failed=0

# Run from inside the fixture directory so the two frontends are handed bare filenames.
cd "$tmp"

# What each frontend is handed, set by the two case runners below.
SRCS=dut.v

# elaborate <frontend> -> prints the diagnostic, returns the frontend's status
elaborate() {
  case $1 in
    iverilog) iverilog -g2012 -o /dev/null $SRCS probe.v 2>&1 ;;
    yosys)    yosys -p "read_verilog -sv $SRCS probe.v; hierarchy -top window_probe" 2>&1 ;;
  esac
}

run_case() {
  local label=$1 file=$2 module=$3 params=$4 expect=$5 want=$6
  printf '`default_nettype none\nmodule window_probe;\n  %s #(%s) dut ();\nendmodule\n' \
    "$module" "$params" > probe.v
  if grep -q '^`include' "$REPO/$file"; then
    cp "$REPO"/rtl/*.v .
    SRCS=$(cd "$REPO/rtl" && printf '%s ' *.v)
  else
    cp "$REPO/$file" dut.v
    SRCS=dut.v
  fi
  local frontend out rc
  for frontend in iverilog yosys; do
    cases=$((cases + 1))
    set +e
    out=$(elaborate "$frontend")
    rc=$?
    set -e
    if [ "$expect" = reject ]; then
      if [ "$rc" -eq 0 ]; then
        echo "FAIL [$frontend] $label: elaborated cleanly; the check did not fire" >&2
        failed=$((failed + 1))
        continue
      fi
      local pin=$want
      [ "$frontend" = yosys ] && pin='FATAL'
      case $out in
        *"$pin"*) ;;
        *)
          echo "FAIL [$frontend] $label: failed without \"$pin\":" >&2
          printf '%s\n' "$out" | sed 's/^/    /' >&2
          failed=$((failed + 1))
          continue ;;
      esac
      echo "ok   [$frontend] $label rejected"
    else
      if [ "$rc" -ne 0 ]; then
        echo "FAIL [$frontend] $label: the shipping parameters were rejected:" >&2
        printf '%s\n' "$out" | sed 's/^/    /' >&2
        failed=$((failed + 1))
        continue
      fi
      echo "ok   [$frontend] $label accepted"
    fi
  done
}

echo "== rtl/imemory.v: ROM_WORDS must be a power of two"
run_case "ROM_WORDS = 1536" rtl/imemory.v imemory ".ROM_WORDS(1536)" \
  reject "ROM_WORDS must be a power of two"
run_case "ROM_WORDS = 2048" rtl/imemory.v imemory ".ROM_WORDS(2048)" accept ""

echo
echo "== rtl/memory.v: an aligned power-of-two window"
run_case "RAM_WORDS = 12288" rtl/memory.v memory \
  ".BASE(32'h0001_0000), .RAM_WORDS(12288)" reject "RAM_WORDS must be a power of two"
run_case "BASE off the window" rtl/memory.v memory \
  ".BASE(32'h0001_0004), .RAM_WORDS(16384)" reject "BASE must be aligned"
run_case "the SoC's own" rtl/memory.v memory \
  ".BASE(32'h0001_0000), .RAM_WORDS(16384)" accept ""

echo
echo "== rtl/timer.v: a BASE aligned to the whole window, which NHARTS sizes"
run_case "BASE = 0x0002_0008" rtl/timer.v timer ".BASE(32'h0002_0008)" \
  reject "BASE must be aligned"
run_case "the SoC's own" rtl/timer.v timer ".BASE(32'h0002_0000)" accept ""
run_case "a 16-byte aligned BASE at one hart" rtl/timer.v timer \
  ".BASE(32'h0002_0010)" accept ""
run_case "...the same BASE at two harts" rtl/timer.v timer \
  ".BASE(32'h0002_0010), .NHARTS(2)" reject "BASE must be aligned"
run_case "two harts at the SoC's own" rtl/timer.v timer \
  ".BASE(32'h0002_0000), .NHARTS(2)" accept ""

echo
echo "== rtl/uart.v: an 8-byte aligned BASE, and a divisor with bits in it"
run_case "BASE = 0x0002_0014" rtl/uart.v uart ".BASE(32'h0002_0014)" \
  reject "BASE must be 8-byte aligned"
run_case "BAUD = the clock" rtl/uart.v uart ".BAUD(12_000_000)" \
  reject "CLOCK_HZ / BAUD must be at least 2"
run_case "the SoC's own" rtl/uart.v uart \
  ".BASE(32'h0002_0020), .CLOCK_HZ(12_000_000), .BAUD(115_200)" accept ""

echo
echo "== rtl/spiflash.v: an 8-byte aligned BASE"
run_case "BASE = 0x0002_002c" rtl/spiflash.v spiflash ".BASE(32'h0002_002c)" \
  reject "BASE must be 8-byte aligned"
run_case "the SoC's own" rtl/spiflash.v spiflash ".BASE(32'h0002_0028)" accept ""

echo
echo "== rtl/littlecpu.v: the copied map has the shape the memories demand"
run_case "LS_TEXT_WORDS = 3072" rtl/littlecpu.v littlecpu ".LS_TEXT_WORDS(3072)" \
  reject "LS_TEXT_WORDS must be a power of two"
run_case "LS_RAM_WORDS = 12288" rtl/littlecpu.v littlecpu ".LS_RAM_WORDS(12288)" \
  reject "LS_RAM_WORDS must be a power of two"
run_case "LS_RAM_BASE off the window" rtl/littlecpu.v littlecpu \
  ".LS_RAM_BASE(32'h0001_0004)" reject "LS_RAM_BASE must be aligned"
run_case "LS_TIMER_BASE = 0x0002_0008" rtl/littlecpu.v littlecpu \
  ".LS_TIMER_BASE(32'h0002_0008)" reject "LS_TIMER_BASE must be 16-byte aligned"
run_case "LS_UART_BASE = 0x0002_0014" rtl/littlecpu.v littlecpu \
  ".LS_UART_BASE(32'h0002_0014)" reject "LS_UART_BASE must be 8-byte aligned"
run_case "LS_FLASH_BASE = 0x0002_002c" rtl/littlecpu.v littlecpu \
  ".LS_FLASH_BASE(32'h0002_002c)" reject "LS_FLASH_BASE must be 8-byte aligned"
run_case "the SoC's own" rtl/littlecpu.v littlecpu \
  ".LS_TEXT_WORDS(2048), .LS_RAM_BASE(32'h0001_0000), .LS_RAM_WORDS(16384), .LS_TIMER_BASE(32'h0002_0000), .LS_UART_BASE(32'h0002_0020), .LS_FLASH_BASE(32'h0002_0028)" \
  accept ""

echo
if [ "$failed" -ne 0 ]; then
  echo "$failed of $cases elaborations did not behave as required." >&2
  exit 1
fi
echo "$cases elaborations, each rejected or accepted as required."

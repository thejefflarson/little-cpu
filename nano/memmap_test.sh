#!/bin/sh
# Reads the map out of the RTL that declares it; refuses one region inside another's span.
set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=${1:-$(cd "$HERE/.." && pwd)}

if [ ! -d "$REPO" ]; then
  echo "error: '$REPO' is not a directory, so there is nothing to compare." >&2
  exit 1
fi

rc=0
fail() {
  echo "error: $*" >&2
  rc=1
}

for f in nano/tt/src/tt_um_thejefflarson_nanocpu.v nano/bus.v nano/uart.v nano/gpio.v nano/nano.v; do
  if [ ! -f "$REPO/$f" ]; then
    echo "error: $f is missing, so its copy of the memory map cannot be compared." >&2
    exit 1
  fi
done

no_param() {  # $1 = file, $2 = parameter name
  echo "error: no \`$2\` default found in $1. This check reads the RTL as the source" >&2
  echo "of the map; if the declaration was respelled, teach this script the new" >&2
  echo "spelling rather than dropping the comparison." >&2
  exit 1
}

hex_param() {  # $1 = file, $2 = parameter name
  raw=$(sed -nE "s/.*(parameter|localparam)[[:space:]]+logic[[:space:]]*\[31:0\][[:space:]]*$2[[:space:]]*=[[:space:]]*32'h([0-9a-fA-F_]*).*/\2/p" \
          "$REPO/$1" | head -1 | tr -d _)
  [ -n "$raw" ] || no_param "$1" "$2"
  echo $((0x$raw))
}

hexfmt() { printf '0x%08x' "$1"; }

TOP=nano/tt/src/tt_um_thejefflarson_nanocpu.v

PSRAM_BASE=$(hex_param "$TOP" PSRAM_BASE)
PSRAM_BYTES=$(hex_param "$TOP" PSRAM_BYTES)
UART_BASE=$(hex_param nano/uart.v BASE)
GPIO_BASE=$(hex_param nano/gpio.v BASE)
UART_BYTES=8
GPIO_BYTES=8
# Reserved for mtime/mtimecmp: four words, rtl/timer.v's one-hart shape.
RESERVED_BYTES=16

PSRAM_TOP=$((PSRAM_BASE + PSRAM_BYTES))
UART_TOP=$((UART_BASE + UART_BYTES))
GPIO_TOP=$((GPIO_BASE + GPIO_BYTES))
RESERVED_BASE=$GPIO_TOP
RESERVED_TOP=$((RESERVED_BASE + RESERVED_BYTES))

if [ "$PSRAM_TOP" -ne "$UART_BASE" ]; then
  fail "PSRAM ends at $(hexfmt "$PSRAM_TOP") and the UART starts at
$(hexfmt "$UART_BASE"). nano_bus joins PSRAM, the UART and GPIO with a chain of
mutually exclusive selects rather than a mux, which is only sound while every
region abuts the next with no gap and no overlap."
fi

if [ "$UART_TOP" -ne "$GPIO_BASE" ]; then
  fail "the UART ends at $(hexfmt "$UART_TOP") and GPIO starts at
$(hexfmt "$GPIO_BASE"). A gap here is merely wasted map; an overlap would answer
the same address from two peripherals at once, and nano_bus's select signals
would both read true."
fi

# The core's own check is the only thing that faults an out-of-window access, so it must
# cover exactly the span nano_bus routes -- no gap either way.
RAM_WORDS_RAW=$(sed -nE "s/.*RAM_WORDS[[:space:]]*=[[:space:]]*\(MAP_TOP[[:space:]]*-[[:space:]]*PSRAM_BASE\)[[:space:]]*\/[[:space:]]*4;.*/present/p" "$REPO/$TOP" | head -1)
if [ "$RAM_WORDS_RAW" != present ]; then
  fail "$TOP no longer derives RAM_WORDS as (MAP_TOP - PSRAM_BASE) / 4. This check
reads that derivation as the statement that the core's own fault window covers
exactly this module's map; teach it the new spelling rather than dropping the
comparison."
fi
MAP_TOP_RAW=$(sed -nE "s/.*MAP_TOP[[:space:]]*=[[:space:]]*PSRAM_BASE[[:space:]]*\+[[:space:]]*PSRAM_BYTES[[:space:]]*\+[[:space:]]*32'd8[[:space:]]*\+[[:space:]]*32'd8[[:space:]]*\+[[:space:]]*32'd16;.*/present/p" "$REPO/$TOP" | head -1)
if [ "$MAP_TOP_RAW" != present ]; then
  fail "$TOP's MAP_TOP is no longer PSRAM_BASE + PSRAM_BYTES + the UART's, GPIO's
and the reserved span's byte counts, in that order. This check computes the
expected window the same way; a divergent formula would pass here while faulting
the wrong addresses on real hardware."
fi
if [ "$MAP_TOP_RAW" = present ] && [ "$RESERVED_TOP" -eq 0 ]; then
  fail "internal error: RESERVED_TOP computed as zero."
fi

if [ $((PSRAM_BASE % PSRAM_BYTES)) -ne 0 ]; then
  fail "PSRAM's base $(hexfmt "$PSRAM_BASE") is not a multiple of its own
$PSRAM_BYTES-byte window."
fi
if [ $((UART_BASE % UART_BYTES)) -ne 0 ]; then
  fail "the UART's base $(hexfmt "$UART_BASE") is not 8-byte aligned."
fi
if [ $((GPIO_BASE % GPIO_BYTES)) -ne 0 ]; then
  fail "GPIO's base $(hexfmt "$GPIO_BASE") is not 8-byte aligned."
fi

if [ "$rc" -eq 0 ]; then
  echo "Memory map agreed on: PSRAM $(hexfmt "$PSRAM_BASE")-$(hexfmt $((PSRAM_TOP - 1))), \
UART $(hexfmt "$UART_BASE")-$(hexfmt $((UART_TOP - 1))), \
GPIO $(hexfmt "$GPIO_BASE")-$(hexfmt $((GPIO_TOP - 1))), \
reserved $(hexfmt "$RESERVED_BASE")-$(hexfmt $((RESERVED_TOP - 1)))."
fi

exit "$rc"

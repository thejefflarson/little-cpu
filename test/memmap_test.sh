#!/bin/bash
# Asserts that every file describing this machine's memory map describes the same one.
set -euo pipefail

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

# Reads one file, or stops.
need() {
  local path=$1
  if [ ! -f "$REPO/$path" ]; then
    echo "error: $path is missing, so its copy of the memory map cannot be" >&2
    echo "compared. If it moved, move this check with it." >&2
    exit 1
  fi
}

for f in rtl/memory.v rtl/timer.v rtl/uart.v rtl/spiflash.v rtl/imemory.v \
         rtl/littlecpu.v rtl/littlesoc.v \
         test/testbench.v \
         test/cxxrtl.cc test/cosim.cc test/dual_cxxrtl.cc \
         test/asm/riscv_test.h test/asm/sections.lds \
         test/asm/boot.lds test/bench/bench.lds formal/traps.sv Makefile; do
  need "$f"
done

# A declaration this cannot read is fatal rather than empty: comparing against an empty
# string is how a check goes on reporting green over a file it has stopped understanding.
no_param() {  # $1 = file, $2 = parameter name
  echo "error: no \`$2\` parameter default found in $1. This check reads the" >&2
  echo "RTL as the source of the map; if the declaration was respelled, teach" >&2
  echo "this script the new spelling rather than dropping the comparison." >&2
  exit 1
}

# `32'h0001_0000` -> 65536.
hex_param() {  # $1 = file, $2 = parameter name
  local raw
  raw=$(sed -nE "s/.*parameter[[:space:]]+logic[[:space:]]*\[31:0\][[:space:]]*$2[[:space:]]*=[[:space:]]*32'h([0-9a-fA-F_]*).*/\1/p" \
          "$REPO/$1" | head -1 | tr -d _)
  [ -n "$raw" ] || no_param "$1" "$2"
  echo $((16#$raw))
}

int_param() {  # $1 = file, $2 = parameter name
  local raw
  raw=$(sed -nE "s/.*parameter[[:space:]]+integer[[:space:]]+$2[[:space:]]*=[[:space:]]*([0-9]+).*/\1/p" \
          "$REPO/$1" | head -1)
  [ -n "$raw" ] || no_param "$1" "$2"
  echo "$raw"
}

# The map's source: every BASE and depth below is read out of the RTL that declares it.

RAM_BASE=$(hex_param rtl/memory.v BASE)
RAM_WORDS=$(int_param rtl/memory.v RAM_WORDS)
TIMER_BASE=$(hex_param rtl/timer.v BASE)
TIMER_HARTS=$(int_param rtl/timer.v NHARTS)
UART_BASE=$(hex_param rtl/uart.v BASE)
FLASH_BASE=$(hex_param rtl/spiflash.v BASE)
RAM_BYTES=$((RAM_WORDS * 4))
RAM_TOP=$((RAM_BASE + RAM_BYTES))
# The UART does not size itself with a parameter -- two words, written into its range
# test -- so this is the one part of the map this file states rather than reads.
UART_BYTES=8
# rtl/spiflash.v's window is two words for the same reason and stated the same way: a
# data register and a control register, written into its range test.
FLASH_BYTES=8
UART_TOP=$((UART_BASE + UART_BYTES))

# The timer's window is two words of `mtime` plus two per hart, rounded up to a power of
# two: four words at one hart and eight at two.
timer_bytes() {  # $1 = NHARTS
  local words=$(( 2 + 2 * $1 )) rounded=1
  while [ "$rounded" -lt "$words" ]; do rounded=$((rounded * 2)); done
  echo $((rounded * 4))
}

TIMER_BYTES=$(timer_bytes "$TIMER_HARTS")
# THE MAP RESERVES THE WIDEST WINDOW THE TIMER CAN BE BUILT WITH, not the one this build
# decodes.
TIMER_RESERVED_HARTS=2
TIMER_RESERVED=$(timer_bytes "$TIMER_RESERVED_HARTS")
TIMER_RESERVED_TOP=$((TIMER_BASE + TIMER_RESERVED))

hexfmt() { printf '0x%08x' "$1"; }

# The shared default is only shared while both files stay silent.

for f in rtl/littlesoc.v test/testbench.v; do
  for m in memory timer uart spiflash; do
    if ! grep -qE "(^|[^[:alnum:]_])$m[[:space:]]*(#\(|[a-z_]+[[:space:]]*\()" "$REPO/$f"; then
      fail "$f does not instantiate \`$m\` at all. The comparison below would
pass vacuously, so a deleted memory is red here rather than silent."
    fi
    # THE MAP IS WHAT MAY NOT BE RESTATED, not every parameter.
    override=$(grep -E "(^|[^[:alnum:]_])$m[[:space:]]*#\(" "$REPO/$f" || true)
    if [ -n "$override" ]; then
      allowed=""
      [ "$m" = uart ] && allowed="CLOCK_HZ"
      # A parameter list this cannot read whole is refused rather than skimmed: no
      # closing parenthesis on the line means it is spread over several, and a check that
      # shrugged at that would be the silence this file exists to prevent.
      case $override in
        *')'*) ;;
        *) fail "$f spreads \`$m\`'s parameter list over more than one line, which
this check cannot read. Put it on one line, or the override it hides is
unreviewable here." ;;
      esac
      for param in $(printf '%s\n' "$override" | grep -oE '\.[A-Za-z_][A-Za-z0-9_]*[[:space:]]*\(' | tr -d '.( \t'); do
        case " $allowed " in
          *" $param "*) continue ;;
        esac
        fail "$f overrides \`$m\`'s parameters. The data RAM's base and size, the
timer's base, the UART's base and baud rate and the SPI controller's base are
rtl/$m.v's defaults precisely so that rtl/littlesoc.v and test/testbench.v
cannot describe different machines
-- the harness once modelled a RAM sixteen times smaller than the SoC's and every
program still fit. If this override is deliberate, it needs a reason recorded in
an ADR first. The one exception is \`uart\`'s CLOCK_HZ, a board clock rate that
names no address; \`$m\`'s \`$param\` is not it."
      done
    fi
  done
done

if [ "$TIMER_RESERVED_TOP" -ne "$UART_BASE" ]; then
  fail "the timer reserves through $(hexfmt $((TIMER_RESERVED_TOP - 1))) and the
UART starts at $(hexfmt "$UART_BASE"). The UART abuts the RESERVED span, not the
decoded one: at NHARTS=$TIMER_HARTS the timer answers only $TIMER_BYTES bytes, so
a UART inside the reservation would work perfectly until the second hart needed
those words, and the OR below would then hand back two live answers at once."
fi

if [ "$UART_TOP" -ne "$FLASH_BASE" ]; then
  fail "the UART ends at $(hexfmt "$UART_TOP") and the SPI controller starts at
$(hexfmt "$FLASH_BASE"). The five read buses join with an OR rather than a mux,
which is only sound while the ranges do not overlap. A gap here is merely wasted
map; an overlap ORs two live answers together and neither simulator would report
it."
fi

if [ "$RAM_TOP" -ne "$TIMER_BASE" ]; then
  fail "the data RAM ends at $(hexfmt $RAM_TOP) and the timer starts at
$(hexfmt "$TIMER_BASE"). rtl/littlesoc.v and test/testbench.v both join the five
read buses with an OR rather than a mux, which is only sound while the ranges do
not overlap; a gap is merely wasted map, but an overlap ORs two live answers
together and neither simulator would report it."
fi

if [ $((TIMER_BASE % TIMER_RESERVED)) -ne 0 ]; then
  fail "the timer's base $(hexfmt "$TIMER_BASE") is off its reserved
${TIMER_RESERVED}-byte window. It decodes $TIMER_BYTES bytes at
NHARTS=$TIMER_HARTS, so this build would elaborate and the two-hart one would
not -- the range test reads the bits above the window and admits addresses the
timer does not occupy at any other alignment."
fi

for f in "$REPO"/rtl/*.v; do
  name=$(basename "$f")
  [ "$name" = timer.v ] && continue
  raw=$(sed -nE "s/.*parameter[[:space:]]+logic[[:space:]]*\[31:0\][[:space:]]*BASE[[:space:]]*=[[:space:]]*32'h([0-9a-fA-F_]*).*/\1/p" \
          "$f" | head -1 | tr -d _)
  [ -n "$raw" ] || continue
  base=$((16#$raw))
  if [ "$base" -ge "$TIMER_BASE" ] && [ "$base" -lt "$TIMER_RESERVED_TOP" ]; then
    fail "rtl/$name puts its window at $(hexfmt "$base"), inside the
$(hexfmt "$TIMER_BASE")..$(hexfmt $((TIMER_RESERVED_TOP - 1))) the timer reserves
for one mtimecmp per hart. Move it to $(hexfmt "$TIMER_RESERVED_TOP") or above.
At NHARTS=$TIMER_HARTS the timer answers only the first $TIMER_BYTES bytes, so
nothing here would overlap today and nothing would report it either."
  fi
done

aligned_window() {  # $1 = whose, $2 = base, $3 = window size in bytes
  if [ $(($2 % $3)) -ne 0 ]; then
    fail "the $1's base $(hexfmt "$2") is not a multiple of its own
$3-byte window. Its range test reads the address bits above the window and
compares them against the base, which admits addresses the device does not
occupy at any other alignment."
  fi
}

aligned_window uart "$UART_BASE" "$UART_BYTES"
aligned_window "SPI controller" "$FLASH_BASE" "$FLASH_BYTES"

lds_field() {  # $1 = file, $2 = region, $3 = ORIGIN|LENGTH
  sed -nE "s/^[[:space:]]*$2[[:space:]]*\([^)]*\)[[:space:]]*:.*$3[[:space:]]*=[[:space:]]*([0-9A-Za-zx_]*).*/\1/p" \
    "$REPO/$1" | head -1
}

as_bytes() {  # $1 = an ld size literal
  local v=$1 mult=1 digits
  case "$v" in
    *K|*k)   digits=${v%[Kk]}; mult=1024 ;;
    *M|*m)   digits=${v%[Mm]}; mult=$((1024 * 1024)) ;;
    0x*|0X*) digits=${v#0[xX]}
             case "$digits" in
               ""|*[!0-9a-fA-F]*) digits="" ;;
               *) echo $((16#$digits)); return ;;
             esac ;;
    *)       digits=$v ;;
  esac
  case "$digits" in
    ""|*[!0-9]*)
      echo "error: '$v' is not a size this check can read. Teach it the" >&2
      echo "spelling rather than letting an unparsed region compare as zero." >&2
      exit 1 ;;
  esac
  echo $((digits * mult))
}

check_lds_ram() {  # $1 = file
  local origin length
  origin=$(lds_field "$1" ram ORIGIN)
  length=$(lds_field "$1" ram LENGTH)
  if [ -z "$origin" ] || [ -z "$length" ]; then
    fail "$1 declares no \`ram\` MEMORY region this check can read."
    return
  fi
  origin=$(as_bytes "$origin"); length=$(as_bytes "$length")
  if [ "$origin" -ne "$RAM_BASE" ]; then
    fail "$1 puts \`ram\` at $(hexfmt "$origin"), but rtl/memory.v's BASE is
$(hexfmt "$RAM_BASE"). Every program's \`.data\` would link to an address the
hardware does not decode."
  fi
  if [ "$length" -ne "$RAM_BYTES" ]; then
    fail "$1 gives \`ram\` $length bytes against the $RAM_BYTES bytes
rtl/memory.v actually has. Too small silently wastes most of the machine and is
how a 4 KB harness went unnoticed against a 64 KB SoC; too large links programs
that run off the end of it."
  fi
}

check_lds_rom() {  # $1 = file, $2 = expected words, $3 = whose
  local length
  length=$(lds_field "$1" rom LENGTH)
  if [ -z "$length" ]; then
    fail "$1 declares no \`rom\` MEMORY region this check can read."
    return
  fi
  length=$(as_bytes "$length")
  if [ "$length" -ne $(( $2 * 4 )) ]; then
    fail "$1 gives \`rom\` $length bytes against $3's $(( $2 * 4 )). A link that
succeeds here has to be one that machine can hold."
  fi
}

SOC_ROM_WORDS_RTL=$(sed -nE "s/.*\.ROM_WORDS\(([0-9]+)\).*/\1/p" "$REPO/rtl/littlesoc.v" | head -1)
TB_ROM_WORDS=$(sed -nE "s/.*localparam[[:space:]]+int[[:space:]]+ROM_WORDS[[:space:]]*=[[:space:]]*([0-9]+).*/\1/p" \
                 "$REPO/test/testbench.v" | head -1)

for pair in "rtl/littlesoc.v:$SOC_ROM_WORDS_RTL" "test/testbench.v:$TB_ROM_WORDS"; do
  if [ -z "${pair#*:}" ]; then
    echo "error: ${pair%%:*} names no ROM_WORDS. The ROM is the one size the" >&2
    echo "two machines differ on deliberately, so it is the one that must stay" >&2
    echo "written down in both." >&2
    exit 1
  fi
done

check_lds_ram test/asm/sections.lds
check_lds_ram test/asm/boot.lds
check_lds_ram test/bench/bench.lds

check_lds_rom test/asm/sections.lds "$TB_ROM_WORDS" "test/testbench.v"
check_lds_rom test/asm/boot.lds     "$TB_ROM_WORDS" "test/testbench.v"
check_lds_rom test/bench/bench.lds  "$SOC_ROM_WORDS_RTL" "rtl/littlesoc.v"

check_ram_base_cc() {  # $1 = file
  local raw
  raw=$(sed -nE "s/.*constexpr[[:space:]]+uint32_t[[:space:]]+kRamBase[[:space:]]*=[[:space:]]*0[xX]([0-9a-fA-F]*).*/\1/p" \
          "$REPO/$1" | head -1)
  if [ -z "$raw" ]; then
    fail "$1 declares no \`kRamBase\`, so nothing says where it thinks RAM is."
    return
  fi
  if [ $((16#$raw)) -ne "$RAM_BASE" ]; then
    fail "$1's kRamBase is $(hexfmt $((16#$raw))) against rtl/memory.v's
$(hexfmt "$RAM_BASE"). This runner subtracts it from every word of the RAM image
before poking it in, so the whole image would land at the wrong offset."
  fi
}

check_ram_base_cc test/cxxrtl.cc
check_ram_base_cc test/cosim.cc
check_ram_base_cc test/dual_cxxrtl.cc

MTIMER_RAW=$(sed -nE "s/^#define[[:space:]]+MTIMER_BASE[[:space:]]+0[xX]([0-9a-fA-F]*).*/\1/p" \
               "$REPO/test/asm/riscv_test.h" | head -1)
if [ -z "$MTIMER_RAW" ]; then
  fail "test/asm/riscv_test.h defines no MTIMER_BASE, so the programs that arm
the timer have no address to arm it at."
elif [ $((16#$MTIMER_RAW)) -ne "$TIMER_BASE" ]; then
  fail "test/asm/riscv_test.h's MTIMER_BASE is $(hexfmt $((16#$MTIMER_RAW)))
against rtl/timer.v's $(hexfmt "$TIMER_BASE"). A store to the wrong address is
dropped by every memory on the bus, so mtimer.S would wait for an interrupt that
is never armed rather than fail."
fi

UART_RAW=$(sed -nE "s/^#define[[:space:]]+UART_BASE[[:space:]]+0[xX]([0-9a-fA-F]*).*/\1/p" \
             "$REPO/test/asm/riscv_test.h" | head -1)
if [ -z "$UART_RAW" ]; then
  fail "test/asm/riscv_test.h defines no UART_BASE, so the program that prints
through the serial port has no address to print at."
elif [ $((16#$UART_RAW)) -ne "$UART_BASE" ]; then
  fail "test/asm/riscv_test.h's UART_BASE is $(hexfmt $((16#$UART_RAW)))
against rtl/uart.v's $(hexfmt "$UART_BASE"). The status register at the wrong
address reads zero from every memory on the bus, so uart.S would wait for a
transmission it never started rather than fail."
fi

MAP_TOP_RAW=$(sed -nE "s/^#define[[:space:]]+MAP_TOP[[:space:]]+0[xX]([0-9a-fA-F]*).*/\1/p" \
                "$REPO/test/asm/riscv_test.h" | head -1)
if [ -z "$MAP_TOP_RAW" ]; then
  fail "test/asm/riscv_test.h defines no MAP_TOP, so the two programs that probe
the region refusal have no address to probe it at."
elif [ $((16#$MAP_TOP_RAW)) -ne $((FLASH_BASE + FLASH_BYTES)) ]; then
  fail "test/asm/riscv_test.h's MAP_TOP is $(hexfmt $((16#$MAP_TOP_RAW)))
against the $(hexfmt $((FLASH_BASE + FLASH_BYTES))) the topmost window ends at.
A store there is meant to be refused; at an address a device DOES answer it is
accepted, and the two programs that read the refusal would fail for a reason
that is not in the core."
fi

SPI_RAW=$(sed -nE "s/^#define[[:space:]]+SPI_BASE[[:space:]]+0[xX]([0-9a-fA-F]*).*/\1/p" \
            "$REPO/test/asm/riscv_test.h" | head -1)
if [ -z "$SPI_RAW" ]; then
  fail "test/asm/riscv_test.h defines no SPI_BASE, so the program that reads the
flash has no address to read it through."
elif [ $((16#$SPI_RAW)) -ne "$FLASH_BASE" ]; then
  fail "test/asm/riscv_test.h's SPI_BASE is $(hexfmt $((16#$SPI_RAW)))
against rtl/spiflash.v's $(hexfmt "$FLASH_BASE"). The control register at the
wrong address reads zero from every memory on the bus, so spiflash.S would see a
controller that is never busy and read back nothing but zeroes."
fi

MK_ROM_WORDS=$(sed -nE 's/^SOC_ROM_WORDS[[:space:]]*:=[[:space:]]*([0-9]+).*/\1/p' \
                 "$REPO/Makefile" | head -1)
if [ -z "$MK_ROM_WORDS" ]; then
  fail "the Makefile sets no SOC_ROM_WORDS, so soc/rom_banks.py has no ceiling
to reject an oversized image against."
elif [ "$MK_ROM_WORDS" -ne "$SOC_ROM_WORDS_RTL" ]; then
  fail "the Makefile builds the SoC ROM image for $MK_ROM_WORDS words and
rtl/littlesoc.v instantiates $SOC_ROM_WORDS_RTL. soc/rom_banks.py grades the
image against the Makefile's number, so the two disagreeing means it either
rejects a program that fits or splits one that does not into banks the bitstream
then truncates."
fi

if [ "$TB_ROM_WORDS" -lt "$SOC_ROM_WORDS_RTL" ]; then
  fail "test/testbench.v simulates $TB_ROM_WORDS words of ROM against
rtl/littlesoc.v's $SOC_ROM_WORDS_RTL. The harness is allowed to be larger --
simulation has no block RAM to run out of, and rvc.S needs it -- but never
smaller, or a program the part can hold would fail in simulation."
fi

CPU_RAM_BASE=$(hex_param rtl/littlecpu.v LS_RAM_BASE)
CPU_RAM_WORDS=$(int_param rtl/littlecpu.v LS_RAM_WORDS)
CPU_TIMER_BASE=$(hex_param rtl/littlecpu.v LS_TIMER_BASE)
CPU_UART_BASE=$(hex_param rtl/littlecpu.v LS_UART_BASE)
CPU_FLASH_BASE=$(hex_param rtl/littlecpu.v LS_FLASH_BASE)
CPU_TEXT_WORDS=$(int_param rtl/littlecpu.v LS_TEXT_WORDS)

cpu_copy() {  # $1 = what, $2 = the core's copy, $3 = the memory's, $4 = whose
  if [ "$2" -ne "$3" ]; then
    fail "rtl/littlecpu.v's $1 is $2 against $4's $3. The core's copy of the map
decides which loads and stores decode refuses (causes 5 and 7) and which
\`make cycles\` reports as near a region edge, so a drifted one faults or
answers accesses about a machine neither file describes."
  fi
}

cpu_copy LS_RAM_BASE   "$CPU_RAM_BASE"   "$RAM_BASE"   rtl/memory.v
cpu_copy LS_RAM_WORDS  "$CPU_RAM_WORDS"  "$RAM_WORDS"  rtl/memory.v
cpu_copy LS_TIMER_BASE "$CPU_TIMER_BASE" "$TIMER_BASE" rtl/timer.v
cpu_copy LS_UART_BASE  "$CPU_UART_BASE"  "$UART_BASE"  rtl/uart.v
cpu_copy LS_FLASH_BASE "$CPU_FLASH_BASE" "$FLASH_BASE" rtl/spiflash.v
cpu_copy LS_TEXT_WORDS "$CPU_TEXT_WORDS" "$SOC_ROM_WORDS_RTL" rtl/littlesoc.v

named_param() {  # $1 = file, $2 = parameter name
  sed -nE "s/.*\.$2\(([^)]*)\).*/\1/p" "$REPO/$1" | head -1
}

for f in rtl/littlesoc.v test/testbench.v; do
  rom=$(named_param "$f" ROM_WORDS)
  text=$(named_param "$f" LS_TEXT_WORDS)
  if [ -z "$rom" ] || [ -z "$text" ]; then
    echo "error: $f names no .ROM_WORDS or no .LS_TEXT_WORDS. This file sizes" >&2
    echo "its own ROM and has to hand the core the same size; if the spelling" >&2
    echo "changed, teach this check the new one rather than dropping it." >&2
    exit 1
  fi
  if [ "$rom" != "$text" ]; then
    fail "$f gives its \`imemory\` $rom words of ROM and tells the core the text
window is $text. The core counts an access near the top of text against the
second, and the memory answers according to the first."
  fi
done

TRAPS_RAM_BASE=$(hex_param formal/traps.sv LS_RAM_BASE)
TRAPS_RAM_WORDS=$(int_param formal/traps.sv LS_RAM_WORDS)
TRAPS_TIMER_BASE=$(hex_param formal/traps.sv LS_TIMER_BASE)
TRAPS_UART_BASE=$(hex_param formal/traps.sv LS_UART_BASE)
TRAPS_FLASH_BASE=$(hex_param formal/traps.sv LS_FLASH_BASE)
TRAPS_TEXT_WORDS=$(int_param formal/traps.sv LS_TEXT_WORDS)

traps_copy() {  # $1 = what, $2 = the proof's copy, $3 = the memory's, $4 = whose
  if [ "$2" -ne "$3" ]; then
    fail "formal/traps.sv's $1 is $2 against $4's $3. That copy decides which
addresses the trap proof excuses from \`must_not_trap\`, so a drifted one proves
something about a machine neither file describes."
  fi
}

traps_copy LS_RAM_BASE   "$TRAPS_RAM_BASE"   "$RAM_BASE"   rtl/memory.v
traps_copy LS_RAM_WORDS  "$TRAPS_RAM_WORDS"  "$RAM_WORDS"  rtl/memory.v
traps_copy LS_TIMER_BASE "$TRAPS_TIMER_BASE" "$TIMER_BASE" rtl/timer.v
traps_copy LS_UART_BASE  "$TRAPS_UART_BASE"  "$UART_BASE"  rtl/uart.v
traps_copy LS_FLASH_BASE "$TRAPS_FLASH_BASE" "$FLASH_BASE" rtl/spiflash.v
traps_copy LS_TEXT_WORDS "$TRAPS_TEXT_WORDS" "$SOC_ROM_WORDS_RTL" rtl/littlesoc.v

if [ "$rc" -ne 0 ]; then
  echo >&2
  echo "The memory map is described in more than one place and they have" >&2
  echo "drifted. test/testbench.v is what the suite grades against and" >&2
  echo "rtl/littlesoc.v is what places on the part; where they disagree, the" >&2
  echo "suite is testing a machine that does not exist." >&2
  exit 1
fi

echo "Memory map agreed on: ram $(hexfmt "$RAM_BASE")+${RAM_BYTES}B, timer $(hexfmt "$TIMER_BASE")+${TIMER_BYTES}B of ${TIMER_RESERVED}B reserved, uart $(hexfmt "$UART_BASE"), spi $(hexfmt "$FLASH_BASE"), rom ${SOC_ROM_WORDS_RTL} words on the part / ${TB_ROM_WORDS} simulated"

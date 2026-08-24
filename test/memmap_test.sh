#!/bin/bash
# Asserts that every file describing this machine's memory map describes the
# same one.
#
# Usage: memmap_test.sh [repo-root]     # defaults to this script's parent
#
# WHY THIS EXISTS. test/testbench.v is what every program in the suite runs
# against and rtl/littlesoc.v is what places on the part, and for the project's
# whole life nothing compared them. The harness modelled a 4 KB data RAM against
# the SoC's 64 KB -- sixteen times smaller -- and no program was large enough to
# notice, so the suite was grading a machine that did not exist. Two more copies
# of the same map went stale the same way in the same day.
#
# The map itself is no longer stated twice: rtl/memory.v, rtl/timer.v and
# rtl/uart.v carry the base and the size as their own parameter defaults, and
# rtl/littlesoc.v and test/testbench.v both instantiate them without overriding
# anything, so the two integrators have nothing to disagree about. THE FIRST CHECK BELOW IS WHAT KEEPS
# THAT TRUE -- an override reappearing in either file is the whole defect coming
# back, and it would otherwise be invisible.
#
# The rest cannot share a parameter, because they are C++, linker scripts,
# assembly, make -- and one SystemVerilog module that instantiates no memory at
# all. For those a comparison is the only instrument left.
#
# Hermetic: grep, sed and shell arithmetic. No toolchain, no simulator, no
# yosys, so this runs inside `make test` anywhere.
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

# Reads one file, or stops. A check that silently skips a missing file is a
# check that deletes itself the day the file is renamed.
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

# A declaration this cannot read is fatal rather than empty: comparing against
# an empty string is how a check goes on reporting green over a file it has
# stopped understanding.
no_param() {  # $1 = file, $2 = parameter name
  echo "error: no \`$2\` parameter default found in $1. This check reads the" >&2
  echo "RTL as the source of the map; if the declaration was respelled, teach" >&2
  echo "this script the new spelling rather than dropping the comparison." >&2
  exit 1
}

# `32'h0001_0000` -> 65536. The underscores are the readable spelling in the
# RTL and mean nothing to arithmetic.
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

# ---- the source of the map ------------------------------------------------

RAM_BASE=$(hex_param rtl/memory.v BASE)
RAM_WORDS=$(int_param rtl/memory.v RAM_WORDS)
TIMER_BASE=$(hex_param rtl/timer.v BASE)
TIMER_HARTS=$(int_param rtl/timer.v NHARTS)
UART_BASE=$(hex_param rtl/uart.v BASE)
FLASH_BASE=$(hex_param rtl/spiflash.v BASE)
RAM_BYTES=$((RAM_WORDS * 4))
RAM_TOP=$((RAM_BASE + RAM_BYTES))
# The UART does not size itself with a parameter -- two words, written into its
# range test -- so this is the one part of the map this file states rather than
# reads. It is here and not in three places because the two integrators take the
# default untouched. The timer's size is computed from NHARTS just below.
UART_BYTES=8
# rtl/spiflash.v's window is two words for the same reason and stated the same
# way: a data register and a control register, written into its range test.
FLASH_BYTES=8
UART_TOP=$((UART_BASE + UART_BYTES))

# The timer's window is two words of `mtime` plus two per hart, rounded up to a
# power of two: four words at one hart and eight at two. Computed the way
# rtl/timer.v computes it rather than copied, because a copied constant is the
# drift this file exists to catch.
timer_bytes() {  # $1 = NHARTS
  local words=$(( 2 + 2 * $1 )) rounded=1
  while [ "$rounded" -lt "$words" ]; do rounded=$((rounded * 2)); done
  echo $((rounded * 4))
}

TIMER_BYTES=$(timer_bytes "$TIMER_HARTS")
# THE MAP RESERVES THE WIDEST WINDOW THE TIMER CAN BE BUILT WITH, not the one
# this build decodes. Two harts need eight words where one needs four, and a
# device placed in the four words between them would have to move on the day the
# second hart lands -- silently, because at one hart those addresses read zero
# and nothing would report the overlap. So the reservation is stated here and
# checked below, and the cost is 16 bytes of address space that read zero on the
# shipping machine.
TIMER_RESERVED_HARTS=2
TIMER_RESERVED=$(timer_bytes "$TIMER_RESERVED_HARTS")
TIMER_RESERVED_TOP=$((TIMER_BASE + TIMER_RESERVED))

hexfmt() { printf '0x%08x' "$1"; }

# ---- 1. neither integrator restates it ------------------------------------
#
# The shared default is only shared while both files stay silent. `imemory` is
# excluded from the `memory` pattern by the leading boundary: it is a different
# module with a size of its own.

for f in rtl/littlesoc.v test/testbench.v; do
  for m in memory timer uart spiflash; do
    if ! grep -qE "(^|[^[:alnum:]_])$m[[:space:]]*(#\(|[a-z_]+[[:space:]]*\()" "$REPO/$f"; then
      fail "$f does not instantiate \`$m\` at all. The comparison below would
pass vacuously, so a deleted memory is red here rather than silent."
    fi
    if grep -qE "(^|[^[:alnum:]_])$m[[:space:]]*#\(" "$REPO/$f"; then
      fail "$f overrides \`$m\`'s parameters. The data RAM's base and size, the
timer's base, the UART's base and baud rate and the SPI master's base are
rtl/$m.v's defaults precisely
so that rtl/littlesoc.v and test/testbench.v cannot describe different machines
-- the harness once modelled a RAM sixteen times smaller than the SoC's and every
program still fit. If this override is deliberate, it needs a reason recorded in
an ADR first."
    fi
  done
done

# ---- 2. the regions abut ---------------------------------------------------

if [ "$TIMER_RESERVED_TOP" -ne "$UART_BASE" ]; then
  fail "the timer reserves through $(hexfmt $((TIMER_RESERVED_TOP - 1))) and the
UART starts at $(hexfmt "$UART_BASE"). The UART abuts the RESERVED span, not the
decoded one: at NHARTS=$TIMER_HARTS the timer answers only $TIMER_BYTES bytes, so
a UART inside the reservation would work perfectly until the second hart needed
those words, and the OR below would then hand back two live answers at once."
fi

if [ "$UART_TOP" -ne "$FLASH_BASE" ]; then
  fail "the UART ends at $(hexfmt "$UART_TOP") and the SPI master starts at
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

# Its range test is an equality on the bits above the window, which is only the
# window while the base is a multiple of the whole of it. rtl/timer.v refuses to
# elaborate otherwise and `make window-test` forces that both ways; this says the
# same thing about the RESERVED span, so a base that is legal for this build and
# not for the two-hart one is caught here rather than on the day it is built.
if [ $((TIMER_BASE % TIMER_RESERVED)) -ne 0 ]; then
  fail "the timer's base $(hexfmt "$TIMER_BASE") is off its reserved
${TIMER_RESERVED}-byte window. It decodes $TIMER_BYTES bytes at
NHARTS=$TIMER_HARTS, so this build would elaborate and the two-hart one would
not -- the range test reads the bits above the window and admits addresses the
timer does not occupy at any other alignment."
fi

# NOTHING ELSE MAY SIT IN THE RESERVED SPAN. Every peripheral on this bus states
# its own base as a `BASE` parameter default, so they are read from rtl/ rather
# than listed here -- a list is what goes stale when a device is added, and a
# device landing in the timer's reserved words is exactly the change that would
# not be noticed: at one hart those addresses read zero from every memory on the
# bus, so the new device would work perfectly until the second hart needed them.
#
# The loop cannot come up empty: rtl/memory.v is in the `need` list above and
# states a `BASE`, and the `hex_param` that reads it stops the whole run rather
# than comparing against an empty string if that is ever respelled.
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

# Each device's window is a power of two on a multiple of its own size, which is
# what lets its range test be an equality on the bits above the window rather
# than a subtraction. rtl/timer.v, rtl/uart.v and rtl/littlecpu.v each refuse to
# elaborate otherwise and `make window-test` forces them; this is the same
# statement made about the numbers this file has already read, so a base that
# drifted is caught here rather than at the next elaboration.
aligned_window() {  # $1 = whose, $2 = base, $3 = window size in bytes
  if [ $(($2 % $3)) -ne 0 ]; then
    fail "the $1's base $(hexfmt "$2") is not a multiple of its own
$3-byte window. Its range test reads the address bits above the window and
compares them against the base, which admits addresses the device does not
occupy at any other alignment."
  fi
}

aligned_window uart "$UART_BASE" "$UART_BYTES"
aligned_window "SPI master" "$FLASH_BASE" "$FLASH_BYTES"

# ---- 3. the linker scripts -------------------------------------------------
#
# `LENGTH = 64K` -> bytes. ld also accepts M and a bare count.

lds_field() {  # $1 = file, $2 = region, $3 = ORIGIN|LENGTH
  sed -nE "s/^[[:space:]]*$2[[:space:]]*\([^)]*\)[[:space:]]*:.*$3[[:space:]]*=[[:space:]]*([0-9A-Za-zx_]*).*/\1/p" \
    "$REPO/$1" | head -1
}

# A size bash cannot read must stop the run. Left to arithmetic expansion, a
# non-numeric literal is treated as a VARIABLE NAME and quietly becomes 0, which
# compares unequal and reports a drift that is really a parse failure. So the
# digits are checked before any arithmetic sees them.
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

# The suite's two scripts link against the SIMULATED ROM, which is larger than
# the part's on purpose. bench.lds links against the part's, because the point of
# building a benchmark is to find out whether it fits.
check_lds_rom test/asm/sections.lds "$TB_ROM_WORDS" "test/testbench.v"
check_lds_rom test/asm/boot.lds     "$TB_ROM_WORDS" "test/testbench.v"
check_lds_rom test/bench/bench.lds  "$SOC_ROM_WORDS_RTL" "rtl/littlesoc.v"

# ---- 4. the runners --------------------------------------------------------

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

# ---- 5. the assembly header ------------------------------------------------

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

SPI_RAW=$(sed -nE "s/^#define[[:space:]]+SPI_BASE[[:space:]]+0[xX]([0-9a-fA-F]*).*/\1/p" \
            "$REPO/test/asm/riscv_test.h" | head -1)
if [ -z "$SPI_RAW" ]; then
  fail "test/asm/riscv_test.h defines no SPI_BASE, so the program that reads the
flash has no address to read it through."
elif [ $((16#$SPI_RAW)) -ne "$FLASH_BASE" ]; then
  fail "test/asm/riscv_test.h's SPI_BASE is $(hexfmt $((16#$SPI_RAW)))
against rtl/spiflash.v's $(hexfmt "$FLASH_BASE"). The control register at the
wrong address reads zero from every memory on the bus, so spiflash.S would see a
master that is never busy and read back nothing but zeroes."
fi

# ---- 6. the SoC ROM image --------------------------------------------------

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

# ---- 7. the one deliberate difference --------------------------------------

if [ "$TB_ROM_WORDS" -lt "$SOC_ROM_WORDS_RTL" ]; then
  fail "test/testbench.v simulates $TB_ROM_WORDS words of ROM against
rtl/littlesoc.v's $SOC_ROM_WORDS_RTL. The harness is allowed to be larger --
simulation has no block RAM to run out of, and rvc.S needs it -- but never
smaller, or a program the part can hold would fail in simulation."
fi

# ---- 8. the core's own copy ------------------------------------------------
#
# rtl/littlecpu.v restates the map for its load/store locality counters, because
# a module cannot read another module's parameters. Nothing in the datapath
# reads it, so a copy that drifted would not fail anything -- it would go on
# counting accesses against a machine that does not exist, which is a measurement
# that is wrong rather than absent.

CPU_RAM_BASE=$(hex_param rtl/littlecpu.v LS_RAM_BASE)
CPU_RAM_WORDS=$(int_param rtl/littlecpu.v LS_RAM_WORDS)
CPU_TIMER_BASE=$(hex_param rtl/littlecpu.v LS_TIMER_BASE)
CPU_UART_BASE=$(hex_param rtl/littlecpu.v LS_UART_BASE)
CPU_FLASH_BASE=$(hex_param rtl/littlecpu.v LS_FLASH_BASE)
CPU_TEXT_WORDS=$(int_param rtl/littlecpu.v LS_TEXT_WORDS)

cpu_copy() {  # $1 = what, $2 = the core's copy, $3 = the memory's, $4 = whose
  if [ "$2" -ne "$3" ]; then
    fail "rtl/littlecpu.v's $1 is $2 against $4's $3. The core's copy of the map
decides which accesses \`make cycles\` reports as near a region edge, so a
drifted one answers about a machine neither file describes."
  fi
}

cpu_copy LS_RAM_BASE   "$CPU_RAM_BASE"   "$RAM_BASE"   rtl/memory.v
cpu_copy LS_RAM_WORDS  "$CPU_RAM_WORDS"  "$RAM_WORDS"  rtl/memory.v
cpu_copy LS_TIMER_BASE "$CPU_TIMER_BASE" "$TIMER_BASE" rtl/timer.v
cpu_copy LS_UART_BASE  "$CPU_UART_BASE"  "$UART_BASE"  rtl/uart.v
cpu_copy LS_FLASH_BASE "$CPU_FLASH_BASE" "$FLASH_BASE" rtl/spiflash.v
# The default is what every harness that does not state a ROM size gets --
# formal/wrapper.v, soc/compare/bench_littlecpu.v -- so it is the part's.
cpu_copy LS_TEXT_WORDS "$CPU_TEXT_WORDS" "$SOC_ROM_WORDS_RTL" rtl/littlesoc.v

# The text window is the one part of the map an integrator states, because the
# harness simulates a larger ROM than the part has. Compared as the TEXT each
# file passes rather than as a number: in test/testbench.v both are the same
# localparam, and a check that resolved it would stop being able to say so. Each
# of these two names appears on exactly one instantiation in either file.
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

# ---- 9. the trap proof's copy ----------------------------------------------
#
# formal/traps.sv models a load or store access fault, so it needs to know which
# addresses a memory here answers -- and no port of the core carries that, so it
# restates the map. Nothing else reads its copy, which is why a drifted one is
# silent: the proof would go on passing, having excused the wrong accesses from
# `must_not_trap` and demanded causes 5 and 7 for a machine no file describes.

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
# The part's text window, not the harness's larger simulated one: the proof has
# no imemory in it to size, so what it describes is the machine that ships.
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

#!/bin/bash
# Builds Dhrystone 2.1 for this core, reports its image against the SoC's two
# memories, runs it under the cxxrtl runner and prints the program's own report
# plus the per-reason cycle accounting. Invoked by `make dhrystone`.
#
# NOT A GATE, AND DELIBERATELY OFF `make test`. There is no CPI ratchet in this
# repo and this adds none: a DMIPS/MHz figure is a measurement to compare
# against the last one and against other cores, not something to fail a merge
# on. The benchmark lives in test/bench rather than test/asm for the same
# reason -- test/asm IS the graded suite, both sim legs glob it, and a program
# that needs two million cycles would time out against the suite's 5000.
#
# Usage: run_dhrystone.sh <sim-binary> <runs> <cycle-limit> <cflags>
#
# WHAT IS PINNED AND WHY. <cflags> arrives from the Makefile and is both what
# the sources are compiled with and what dhry_port.c prints, from one string --
# so the flags beside the number are the flags that produced it rather than a
# comment that can go stale. dhry_port.c refuses to compile without them.
set -euo pipefail

if [ "$#" -ne 4 ]; then
  echo "usage: run_dhrystone.sh <sim-binary> <runs> <cycle-limit> <cflags>" >&2
  exit 1
fi

SIM=$1
RUNS=$2
CYCLE_LIMIT=$3
CFLAGS=$4
HERE=$(cd "$(dirname "$0")" && pwd)
TEST_DIR=$(cd "$HERE/.." && pwd)

# Both budgets are read out of the linker script rather than passed in, because
# the linker script is what ENFORCES them -- ld reports an overflow in bytes and
# no image gets built. A second copy of 8192 in the Makefile would be free to
# drift, and the copy that drifted would be the one printed next to the result.
lds_region_bytes() {
  awk -v region="$1" \
    '{ sub(/^[ \t]+/, "") }
     index($0, region "(") == 1 && match($0, /LENGTH = [0-9]+K/) {
       print substr($0, RSTART + 9, RLENGTH - 10) * 1024; exit
     }' "$HERE/bench.lds"
}

ROM_BUDGET=$(lds_region_bytes rom)
RAM_BUDGET=$(lds_region_bytes ram)
if [ -z "$ROM_BUDGET" ] || [ -z "$RAM_BUDGET" ]; then
  echo "error: could not read the 'rom' and 'ram' region lengths out of" >&2
  echo "$HERE/bench.lds, so there is no budget to report the image against." >&2
  echo "Nothing was built." >&2
  exit 1
fi

if [ ! -x "$SIM" ]; then
  echo "error: '$SIM' is not an executable runner; build it with 'make sim'." >&2
  exit 1
fi

CC=""
for candidate in riscv64-elf-gcc riscv64-unknown-elf-gcc; do
  if command -v "$candidate" >/dev/null 2>&1; then
    CC=$candidate
    break
  fi
done
if [ -z "$CC" ]; then
  echo "error: no RISC-V cross compiler found (tried riscv64-elf-gcc, riscv64-unknown-elf-gcc)." >&2
  echo "Run 'make setup' to install one." >&2
  exit 1
fi

OBJCOPY=${CC%gcc}objcopy
NM=${CC%gcc}nm
for tool in "$OBJCOPY" "$NM"; do
  if ! command -v "$tool" >/dev/null 2>&1; then
    echo "error: found $CC but not its matching $tool (half-installed toolchain)." >&2
    exit 1
  fi
done

tmp=$(mktemp -d "${TMPDIR:-/tmp}/dhrystone.XXXXXX") || {
  echo "error: could not create a temporary directory under ${TMPDIR:-/tmp}." >&2
  exit 1
}
if [ -z "$tmp" ] || [ ! -d "$tmp" ]; then
  echo "error: mktemp -d produced no usable directory under ${TMPDIR:-/tmp}." >&2
  exit 1
fi
trap 'rm -rf "$tmp"' EXIT

# THREE SEPARATE COMPILATIONS, NO -flto. dhry_1.c holds the measured loop and
# dhry_2.c holds six of the procedures it calls; letting the compiler see both
# at once lets it inline and then delete most of the benchmark, and every
# published Dhrystone number is from a build where it cannot. Do not merge these
# into one command with several sources either -- gcc still compiles them
# separately, but a future -flto or -fwhole-program would not be visible here.
objects=()
for unit in dhry_1 dhry_2 dhry_port; do
  # shellcheck disable=SC2086
  $CC $CFLAGS -DDHRY_RUNS="$RUNS" -DDHRY_FLAGS="\"$CFLAGS\"" \
    -c "$HERE/$unit.c" -o "$tmp/$unit.o"
  objects+=("$tmp/$unit.o")
done

elf="$tmp/dhrystone.elf"
# shellcheck disable=SC2086
if ! $CC $CFLAGS -nostdlib -T "$HERE/bench.lds" -o "$elf" \
     "$TEST_DIR/crt0.S" "${objects[@]}" 2> "$tmp/link.log"; then
  cat "$tmp/link.log" >&2
  echo >&2
  echo "*** Dhrystone did not link. If the message above is a region overflow" >&2
  echo "*** on 'rom', it did not fit the SoC's $ROM_BUDGET-byte instruction" >&2
  echo "*** memory. That is the answer to report, not something to fix by" >&2
  echo "*** trimming the benchmark: a trimmed Dhrystone is not Dhrystone and" >&2
  echo "*** its number would not be comparable, which is the whole point." >&2
  exit 1
fi
if [ -s "$tmp/link.log" ]; then
  cat "$tmp/link.log" >&2
  echo "error: the link produced diagnostics; warnings are errors here." >&2
  exit 1
fi

# The ROM image is what the part has to hold: `.text` (which carries `.rodata`)
# plus `.data`'s load copy, contiguous, exactly as soc-rom would burn it. Taken
# from the flat binary rather than from `size`, whose `data` column counts the
# `.tohost` window -- which is loadable, and lives in RAM.
$OBJCOPY -O binary -j .text -j .data "$elf" "$tmp/rom.bin"
rom_bytes=$(wc -c < "$tmp/rom.bin" | tr -d ' ')

# Everything the program needs in RAM. `.bss` is nearly all of it: Dhrystone's
# Arr_2_Glob is 10 KB on its own, which is why the simulated RAM had to become
# the SoC's real 64 KB rather than the 4 KB the assembly suite gets by with.
ram_bytes=$("${CC%gcc}size" -A "$elf" \
  | awk '$1 == ".tohost" || $1 == ".data" || $1 == ".bss" { total += $2 }
         END { print total + 0 }')

$OBJCOPY -O verilog --verilog-data-width=4 -j .text -j .data "$elf" "$tmp/rom.hex"
$OBJCOPY -O verilog --verilog-data-width=4 -j .tohost "$elf" "$tmp/ram.hex"
for image in "$tmp/rom.hex" "$tmp/ram.hex"; do
  if [ ! -s "$image" ]; then
    echo "error: objcopy produced an empty $image." >&2
    exit 1
  fi
done

console_addr=$($NM "$elf" | awk '$3 == "dhry_console" { print "0x" $1 }')
if [ -z "$console_addr" ]; then
  echo "error: dhry_console is not in the linked image, so the program's own" >&2
  echo "report cannot be read back out of RAM. Nothing was run." >&2
  exit 1
fi

echo "== Dhrystone 2.1, built for this core =="
echo "compiler : $CC $($CC -dumpversion)"
echo "flags    : $CFLAGS"
echo "runs     : $RUNS"
echo
echo "== the image against the SoC's memories =="
echo "rom (.text + .data load copy): $rom_bytes of $ROM_BUDGET bytes"
if [ "$rom_bytes" -gt "$ROM_BUDGET" ]; then
  echo "*** over budget by $((rom_bytes - ROM_BUDGET)) bytes." >&2
  exit 1
fi
echo "                               $((ROM_BUDGET - rom_bytes)) free"
echo "ram (.tohost + .data + .bss)   $ram_bytes of $RAM_BUDGET bytes"
echo

set +e
"$SIM" --rom "$tmp/rom.hex" --ram "$tmp/ram.hex" --cycles "$CYCLE_LIMIT" \
  --stalls --console "$console_addr" > "$tmp/run.log" 2>&1
sim_status=$?
set -e

# test/testbench.v $displays every fetch and every bus cycle, which is a
# microscope on a 5000-cycle test program and a million lines on this one. The
# report, the verdict and the two counter lines are what is wanted; awk rather
# than a pipeline off the runner so its exit status above is still its own.
awk '!/^(ifetch |write  |read   |trap!)/' "$tmp/run.log"

if [ "$sim_status" -ne 0 ]; then
  echo >&2
  echo "*** the run did not reach a PASS verdict (runner exit $sim_status)." >&2
  echo "*** Exit 2 is the cycle limit: raise DHRY_CYCLES. Anything else is the" >&2
  echo "*** benchmark's own self-check, or the per-retire monitor, saying the" >&2
  echo "*** core computed the wrong thing -- in which case the timing number" >&2
  echo "*** below describes a run that was not correct and means nothing." >&2
  exit "$sim_status"
fi

stall_line=$(awk '/^STALLS /{$1=""; print; exit}' "$tmp/run.log")
retires=$(awk '/^RETIRES /{print $2; exit}' "$tmp/run.log")
if [ -z "$stall_line" ] || [ -z "$retires" ]; then
  echo "error: the run printed no STALLS/RETIRES line, so there is nothing to" >&2
  echo "account for. Was '$SIM' built from test/cxxrtl.cc?" >&2
  exit 1
fi

# What fraction of the table is NOT the benchmark. The startup zeroes Dhrystone's
# 10 KB Arr_2_Glob one word at a time before main, and that loop's stall mix is
# nothing like the benchmark's -- so the accounting is only worth reading while
# this stays small, and printing it is what says whether it did.
measured=$(awk '/^Cycles  *:/ { print $3; exit }' "$tmp/run.log")
counted=$(awk '/^STALLS / { for (i = 1; i <= NF; i++) if (sub(/^cycles=/, "", $i)) { print $i; exit } }' \
  "$tmp/run.log")
outside=$(awk -v c="$counted" -v m="$measured" \
  'BEGIN { printf "%d cycles, %.1f%%", c - m, 100 * (c - m) / c }')

echo "dhrystone.c $stall_line retires=$retires" > "$tmp/stall_counts"
python3 "$TEST_DIR/stall_report.py" "$tmp/stall_counts" --workload \
"READ THE CPI AS A PROPERTY OF DHRYSTONE. This is compiled C with a real call
structure, loops and string work, so the mix is nothing like the hand-written
assembly \`make cycles\` accounts for. Reading the two tables against each other
is what running this is for; reading either against another core's is not
something either one supports.

The table covers the WHOLE run and the DMIPS/MHz figure above covers only the
cycles between the two mcycle reads. The difference -- test/crt0.S zeroing
Dhrystone's 10 KB Arr_2_Glob a word at a time, the setup, and the report
formatting -- is $outside of the table, at $RUNS runs."

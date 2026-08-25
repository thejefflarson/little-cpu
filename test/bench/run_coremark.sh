#!/bin/bash
# Builds CoreMark for this core, runs it under the cxxrtl runner and prints
# both core_main.c's own report and this port's CoreMark/MHz trailer.
# Invoked by `make coremark`.
#
# NOT A GATE, AND DELIBERATELY OFF `make test` -- the same reasons
# run_dhrystone.sh gives: there is no CPI ratchet here, and the benchmark
# lives in test/bench rather than test/asm because both sim legs glob test/asm
# against a 5000-cycle limit CoreMark's iteration count is nowhere near.
#
# Usage: run_coremark.sh <sim-binary> <iterations> <cycle-limit> <cflags>
#
# SIMULATED AT 16 KB OF ROM, NOT THIS PART'S 8. test/bench/coremark.lds gives
# `rom` that length because test/testbench.v's ROM_WORDS is double
# rtl/imemory.v's shipping 2048 words, and CoreMark does not fit the smaller
# one -- several times Dhrystone's 3568 bytes, the wall test/asm/rvc.S hits at
# 12256. The figure this prints describes a machine that cannot be built until
# this part's deferred SPI-flash boot path lands and the ROM grows; every line
# below says so again, because a figure that forgets its own memory
# configuration is not one EEMBC's run rules would let stand.
set -euo pipefail

if [ "$#" -ne 4 ]; then
  echo "usage: run_coremark.sh <sim-binary> <iterations> <cycle-limit> <cflags>" >&2
  exit 1
fi

SIM=$1
ITERATIONS=$2
CYCLE_LIMIT=$3
CFLAGS=$4
HERE=$(cd "$(dirname "$0")" && pwd)
TEST_DIR=$(cd "$HERE/.." && pwd)
VENDOR_DIR="$HERE/coremark"

# Read out of the linker scripts rather than hardcoded, the same reason
# run_dhrystone.sh reads bench.lds: a second copy of either budget would be
# free to drift, and the copy that drifted would be the one printed next to
# the result.
lds_region_bytes() {  # $1 = lds path, $2 = region name
  awk -v region="$2" \
    '{ sub(/^[ \t]+/, "") }
     index($0, region "(") == 1 && match($0, /LENGTH = [0-9]+K/) {
       print substr($0, RSTART + 9, RLENGTH - 10) * 1024; exit
     }' "$1"
}
SIM_ROM_BUDGET=$(lds_region_bytes "$HERE/coremark.lds" rom)
SHIP_ROM_BUDGET=$(lds_region_bytes "$HERE/bench.lds" rom)
if [ -z "$SIM_ROM_BUDGET" ] || [ -z "$SHIP_ROM_BUDGET" ]; then
  echo "error: could not read the 'rom' region length out of" >&2
  echo "$HERE/coremark.lds or $HERE/bench.lds, so there is no budget to" >&2
  echo "report the image against. Nothing was built." >&2
  exit 1
fi

# Membership is a two-way match, not just shasum -c's one-way one: shasum only
# verifies the names PINNED.sha256 lists, and says nothing about a file
# dropped in beside them. That is concretely exploitable here -- coremark.h's
# `#include "core_portme.h"` is a quoted include, which searches the including
# file's own directory FIRST, so an unlisted core_portme.h in $VENDOR_DIR would
# shadow this port's real header (which carries the timing hooks) for every
# vendored unit while shasum -c reports the tree unmodified.
manifest_files=$(awk '!/^#/ && NF { print $NF }' "$VENDOR_DIR/PINNED.sha256" | sort)
tree_files=$(cd "$VENDOR_DIR" && for f in *; do
  if [ -f "$f" ] && [ "$f" != "PINNED.sha256" ]; then
    echo "$f"
  fi
done | sort)
# comm, the same two-way idiom test/check_suite_shape.sh and test/dual_build.sh
# use for a manifest against a directory: -23 is named but missing, -13 is
# present but unnamed.
missing=$(comm -23 <(printf '%s\n' "$manifest_files") <(printf '%s\n' "$tree_files"))
unlisted=$(comm -13 <(printf '%s\n' "$manifest_files") <(printf '%s\n' "$tree_files"))
if [ -n "$missing" ] || [ -n "$unlisted" ]; then
  echo "error: $VENDOR_DIR does not have exactly the files PINNED.sha256" >&2
  echo "lists -- shasum -c cannot see a file that manifest never named." >&2
  if [ -n "$missing" ]; then
    echo "named in the manifest but missing from the directory:" >&2
    echo "$missing" | sed 's/^/  /' >&2
  fi
  if [ -n "$unlisted" ]; then
    echo "in the directory but not named in the manifest:" >&2
    echo "$unlisted" | sed 's/^/  /' >&2
  fi
  exit 1
fi

if command -v shasum >/dev/null 2>&1; then
  SHA_CHECK=(shasum -a 256 -c --strict)
elif command -v sha256sum >/dev/null 2>&1; then
  SHA_CHECK=(sha256sum -c --strict)
else
  echo "error: neither shasum nor sha256sum is on PATH, so the vendored" >&2
  echo "CoreMark sources cannot be checked against $VENDOR_DIR/PINNED.sha256." >&2
  exit 1
fi
pin_check=$(mktemp "${TMPDIR:-/tmp}/coremark_pin_check.XXXXXX") || {
  echo "error: could not create a temporary file under ${TMPDIR:-/tmp}." >&2
  exit 1
}
trap 'rm -f "$pin_check"' EXIT
if ! (cd "$VENDOR_DIR" && "${SHA_CHECK[@]}" PINNED.sha256) >"$pin_check" 2>&1; then
  cat "$pin_check" >&2
  echo >&2
  echo "*** $VENDOR_DIR no longer matches PINNED.sha256. CoreMark's own" >&2
  echo "*** trademark terms permit quoting the name only for an unmodified" >&2
  echo "*** copy of the benchmark -- re-vendor from the pinned commit" >&2
  echo "*** rather than editing a file in that directory." >&2
  exit 1
fi
# --strict makes a malformed manifest line fail the check above; this is the
# quieter half of the same guard -- a WARNING for a line that is merely
# unusual (a comment shasum tolerates, say) does not fail the run, so it must
# not be silently dropped either.
cat "$pin_check" >&2
rm -f "$pin_check"

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

tmp=$(mktemp -d "${TMPDIR:-/tmp}/coremark.XXXXXX") || {
  echo "error: could not create a temporary directory under ${TMPDIR:-/tmp}." >&2
  exit 1
}
trap 'rm -rf "$tmp"' EXIT

# Six separate compilations, no -flto -- the same reason run_dhrystone.sh gives
# for Dhrystone's three: every published CoreMark number is from a build where
# the algorithm units cannot see across each other and inline the benchmark
# away. -I twice: coremark.h needs core_portme.h beside it, and
# coremark_port.c needs coremark.h beside it, and neither lives in the other's
# directory.
objects=()
for unit in coremark/core_list_join coremark/core_main coremark/core_matrix \
            coremark/core_state coremark/core_util coremark_port; do
  name=$(basename "$unit")
  # shellcheck disable=SC2086
  $CC $CFLAGS -I "$HERE" -I "$VENDOR_DIR" \
    -DITERATIONS="$ITERATIONS" "-DCOREMARK_FLAGS=\"$CFLAGS\"" \
    -c "$HERE/$unit.c" -o "$tmp/$name.o"
  objects+=("$tmp/$name.o")
done

elf="$tmp/coremark.elf"
# shellcheck disable=SC2086
if ! $CC $CFLAGS -nostdlib -T "$HERE/coremark.lds" -o "$elf" \
     "$TEST_DIR/crt0.S" "${objects[@]}" 2> "$tmp/link.log"; then
  cat "$tmp/link.log" >&2
  echo >&2
  echo "*** CoreMark did not link. If the message above is a region overflow" >&2
  echo "*** on 'rom', it did not fit the SIMULATED 16 KB either -- raise" >&2
  echo "*** test/testbench.v's ROM_WORDS to measure a bigger machine, rather" >&2
  echo "*** than trimming the benchmark. A trimmed CoreMark is not CoreMark." >&2
  exit 1
fi
if [ -s "$tmp/link.log" ]; then
  cat "$tmp/link.log" >&2
  echo "error: the link produced diagnostics; warnings are errors here." >&2
  exit 1
fi

$OBJCOPY -O binary -j .text -j .data "$elf" "$tmp/rom.bin"
rom_bytes=$(wc -c < "$tmp/rom.bin" | tr -d ' ')

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

console_addr=$($NM "$elf" | awk '$3 == "coremark_console" { print "0x" $1 }')
if [ -z "$console_addr" ]; then
  echo "error: coremark_console is not in the linked image, so neither" >&2
  echo "report can be read back out of RAM. Nothing was run." >&2
  exit 1
fi

echo "== CoreMark, built for this core -- SIMULATED AT 16 KB OF ROM =="
echo "compiler   : $CC $($CC -dumpversion)"
echo "flags      : $CFLAGS"
echo "iterations : $ITERATIONS"
echo
echo "== the image against the SIMULATED memory, not the shipping SoC's =="
echo "rom (.text + .data load copy): $rom_bytes of $SIM_ROM_BUDGET simulated bytes"
if [ "$rom_bytes" -gt "$SIM_ROM_BUDGET" ]; then
  echo "*** over the simulated budget by $((rom_bytes - SIM_ROM_BUDGET)) bytes." >&2
  exit 1
fi
echo "                               $((SIM_ROM_BUDGET - rom_bytes)) free"
if [ "$rom_bytes" -gt "$SHIP_ROM_BUDGET" ]; then
  echo "*** $((rom_bytes - SHIP_ROM_BUDGET)) bytes over the SHIPPING SoC's" \
       "$SHIP_ROM_BUDGET-byte ROM. This image cannot boot rtl/littlesoc.v" \
       "as it stands; it needs the SPI-flash boot path CLAUDE.md lists as" \
       "still deferred."
fi
echo "ram (.tohost + .data + .bss)   $ram_bytes of 65536 bytes"
echo

set +e
"$SIM" --rom "$tmp/rom.hex" --ram "$tmp/ram.hex" --cycles "$CYCLE_LIMIT" \
  --stalls --console "$console_addr" > "$tmp/run.log" 2>&1
sim_status=$?
set -e

# One pass over run.log rather than three: at COREMARK_CYCLES' default budget
# this log is 50x the size run_dhrystone.sh's DHRY_CYCLES ever produces. The
# LAST STALLS/RETIRES line is kept, not the first: test/cxxrtl.cc's
# report_counts() prints the guest's own console buffer before its own
# STALLS/RETIRES lines and nothing after them, so a run whose guest program
# writes text that happens to start with "STALLS " or "RETIRES " (coremark.h's
# console buffer is otherwise-arbitrary bytes this port copies verbatim) must
# not have that text mistaken for the runner's own accounting.
: > "$tmp/extract"
awk -v out="$tmp/extract" \
  '/^STALLS /{stalls = $0}
   /^RETIRES /{retires = $2}
   !/^(ifetch |write  |read   |trap!)/
   END {
     if (stalls != "") { sub(/^STALLS/, "", stalls); print "S" stalls > out }
     if (retires != "") print "R" retires > out
   }' "$tmp/run.log"

if [ "$sim_status" -ne 0 ]; then
  echo >&2
  echo "*** the run did not reach a PASS verdict (runner exit $sim_status)." >&2
  echo "*** Exit 2 is the cycle limit: raise the third argument. Anything" >&2
  echo "*** else is a CRC self-check core_main.c printed above, this port's" >&2
  echo "*** own could-not-be-validated verdict, or the per-retire monitor," >&2
  echo "*** saying the core computed the wrong thing -- in which case the" >&2
  echo "*** timing number below describes a run that was not correct and" >&2
  echo "*** means nothing." >&2
  exit "$sim_status"
fi

stall_line=$(awk '/^S/{sub(/^S/, ""); print; exit}' "$tmp/extract")
retires=$(awk '/^R/{sub(/^R/, ""); print; exit}' "$tmp/extract")
if [ -z "$stall_line" ] || [ -z "$retires" ]; then
  echo "error: the run printed no STALLS/RETIRES line, so there is nothing to" >&2
  echo "account for. Was '$SIM' built from test/cxxrtl.cc?" >&2
  exit 1
fi
echo "coremark.c $stall_line retires=$retires" > "$tmp/stall_counts"
python3 "$TEST_DIR/stall_report.py" "$tmp/stall_counts" --workload \
"READ THE CPI AS A PROPERTY OF COREMARK, and COREMARK'S AS A PROPERTY OF THIS
CONFIGURATION. This core has no forwarding network (priced and declined --
see the hazards commitment in CLAUDE.md) and no bitmanip extension, and
CoreMark leans on both harder than Dhrystone does. Reading this table against
\`make cycles\`'s hand-written-assembly one is what running this is for; reading
either DMIPS/MHz or CoreMark/MHz against a number this repo did not measure on
its own hardware, at its own ROM size, is not something either supports."

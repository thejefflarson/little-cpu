#!/bin/bash
# Builds CoreMark for this core, runs it under the cxxrtl runner and prints both
# core_main.c's own report and this port's CoreMark/MHz trailer.
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

# Read out of the linker scripts rather than hardcoded: a second copy of either budget
# would be free to drift, and the copy that drifted is the one printed with the result.
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

# ONE implementation for every route that checks the vendored tree against PINNED.sha256.
"$HERE/coremark_pin_check.sh" "$VENDOR_DIR"

# coremark_port.c restates both 2K runs' CRCs independently, so a mutated literal in
# either copy is caught against the pinned vendor array before a compiler runs.
known_crc() {  # $1 = array name in core_main.c, $2 = 1-based entry
  awk "/$1\\[\\]/,/;/" "$VENDOR_DIR/core_main.c" | grep -oE '0x[0-9a-fA-F]+' | sed -n "$2p"
}
port_crc() {  # $1 = #define name in coremark_port.c
  grep -m1 -oE "#define $1 0x[0-9a-fA-F]+" "$HERE/coremark_port.c" | grep -oE '0x[0-9a-fA-F]+'
}
# core_main.c indexes both arrays by `known_id`: performance fourth, validation fifth.
for triple in "list_known_crc:4:COREMARK_2K_PERF_CRCLIST" \
              "matrix_known_crc:4:COREMARK_2K_PERF_CRCMATRIX" \
              "state_known_crc:4:COREMARK_2K_PERF_CRCSTATE" \
              "list_known_crc:5:COREMARK_2K_VALIDATION_CRCLIST" \
              "matrix_known_crc:5:COREMARK_2K_VALIDATION_CRCMATRIX" \
              "state_known_crc:5:COREMARK_2K_VALIDATION_CRCSTATE"; do
  vendor_name=${triple%%:*}
  rest=${triple#*:}
  entry=${rest%%:*}
  port_name=${rest#*:}
  vendor_val=$(known_crc "$vendor_name" "$entry")
  port_val=$(port_crc "$port_name")
  if [ -z "$vendor_val" ] || [ -z "$port_val" ]; then
    echo "error: could not read $vendor_name entry $entry from" >&2
    echo "$VENDOR_DIR/core_main.c, or $port_name from" >&2
    echo "$HERE/coremark_port.c -- nothing to cross-check." >&2
    exit 1
  fi
  if [ "$((vendor_val))" != "$((port_val))" ]; then
    echo "error: coremark_port.c's $port_name ($port_val) does not match" >&2
    echo "$VENDOR_DIR/core_main.c's $vendor_name entry $entry ($vendor_val) --" >&2
    echo "a 2K CRC has drifted from the pinned vendor copy." >&2
    exit 1
  fi
done

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
echo "SIMULATED AT 16 KB OF ROM -- double this part's 8 KB. This figure"
echo "describes a machine that cannot be built until the ROM grows; the"
echo "budget line above says how close the image itself came to the part's"
echo "real 8 KB anyway."
echo

set +e
"$SIM" --rom "$tmp/rom.hex" --ram "$tmp/ram.hex" --cycles "$CYCLE_LIMIT" \
  --stalls --console "$console_addr" > "$tmp/run.log" 2>&1
sim_status=$?
set -e

# One pass over run.log rather than three: at COREMARK_CYCLES' default budget this log is
# 50x the size run_dhrystone.sh's DHRY_CYCLES ever produces.
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

# EEMBC's "Required 2": the 2K validation configuration must also pass. iterate() latches
# the three CRCs from the first iteration, so one iteration checks them.
echo
echo "== CoreMark, 2K validation configuration -- EEMBC's second required"
echo "   self-check, not a second score; see coremark_port.c's header =="
val_obj="$tmp/coremark_port_validation.o"
# shellcheck disable=SC2086
$CC $CFLAGS -I "$HERE" -I "$VENDOR_DIR" -DITERATIONS=1 -DCOREMARK_VALIDATION \
  "-DCOREMARK_FLAGS=\"$CFLAGS\"" -c "$HERE/coremark_port.c" -o "$val_obj"
val_objects=("${objects[@]:0:5}" "$val_obj")

val_elf="$tmp/coremark_validation.elf"
# shellcheck disable=SC2086
if ! $CC $CFLAGS -nostdlib -T "$HERE/coremark.lds" -o "$val_elf" \
     "$TEST_DIR/crt0.S" "${val_objects[@]}" 2> "$tmp/val_link.log"; then
  cat "$tmp/val_link.log" >&2
  echo "*** the 2K validation image did not link." >&2
  exit 1
fi
if [ -s "$tmp/val_link.log" ]; then
  cat "$tmp/val_link.log" >&2
  echo "error: the validation link produced diagnostics; warnings are errors here." >&2
  exit 1
fi

$OBJCOPY -O verilog --verilog-data-width=4 -j .text -j .data "$val_elf" "$tmp/val_rom.hex"
$OBJCOPY -O verilog --verilog-data-width=4 -j .tohost "$val_elf" "$tmp/val_ram.hex"
for image in "$tmp/val_rom.hex" "$tmp/val_ram.hex"; do
  if [ ! -s "$image" ]; then
    echo "error: objcopy produced an empty $image for the validation image." >&2
    exit 1
  fi
done

set +e
"$SIM" --rom "$tmp/val_rom.hex" --ram "$tmp/val_ram.hex" --cycles "$CYCLE_LIMIT" \
  > "$tmp/val_run.log" 2>&1
val_status=$?
set -e
awk '!/^(ifetch |write  |read   |trap!)/' "$tmp/val_run.log"

if [ "$val_status" -ne 0 ]; then
  echo >&2
  echo "*** the 2K validation configuration did not reach a PASS verdict" >&2
  echo "*** (runner exit $val_status). This is required by EEMBC's own run" >&2
  echo "*** rules and is not optional -- see the CRC lines above." >&2
  exit "$val_status"
fi
echo "2K validation configuration: PASS"

echo
echo "READ THE FLAGS AND THE ROM SIZE WITH THE NUMBER. CoreMark is"
echo "less string-dominated than Dhrystone and harder for the optimiser"
echo "to delete, but it is still a compiled figure: the compiler, the"
echo "flags and the iteration count travel with it because EEMBC's own"
echo "run rules require disclosing all three. This core is stall-only"
echo "with no bitmanip extension, and CoreMark leans on both -- a figure"
echo "well under a core built with forwarding and Zba/Zbb/Zbs is the"
echo "price of this core's four goals, not a defect in the port."

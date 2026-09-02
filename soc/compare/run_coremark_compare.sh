#!/bin/bash
# Builds one CoreMark image for littlecpu and Hazard3's iCE40 configuration,
# reports it against the geometry the harness can actually place, and runs
# both cores on it in one iverilog simulation. Mirrors
# soc/compare/run_dhrystone.sh's split for the identical reasons.
#
# Usage: run_coremark_compare.sh <iterations> <cycle-limit> <cflags> [vvp-binary]
#
# WHAT THIS IS AND IS NOT. `make compare-timing` places each core and reports
# a clock. This reports the other factor of throughput -- cycles for the same
# work, on the SAME cores this repository already measured a clock for
# (docs/adr/0139-*.md) -- for both, from the same image, so that a CoreMark/MHz
# figure for either side is measured here rather than quoted from a project's
# own README.
#
# IT IS A SIMULATION AND CANNOT BE A PLACEMENT. CoreMark needs more memory than
# an hx8k has block RAM for; soc/compare/coremark.lds carries the arithmetic
# and the report below prints it every run, because the distortion has to
# travel with the number. The clock to multiply these cycles by is
# `make compare-timing`'s, taken at the SMALLER placed geometry, and that
# mismatch is the headline caveat.
set -euo pipefail

if [ "$#" -lt 3 ] || [ "$#" -gt 4 ]; then
  echo "usage: run_coremark_compare.sh <iterations> <cycle-limit> <cflags> [vvp-binary]" >&2
  exit 1
fi

ITERATIONS=$1
CYCLE_LIMIT=$2
CFLAGS=$3
VVP_BIN=${4:-compare.coremark.vvp}
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
VENDOR_DIR="$REPO/test/bench/coremark"

lds_field() {  # $1 = region, $2 = LENGTH|ORIGIN, $3 = linker script
  awk -v region="$1" -v key="$2" \
    '{ sub(/^[ \t]+/, "") }
     index($0, region "(") != 1 { next }
     match($0, key " = [^,)]+") {
       value = substr($0, RSTART + length(key) + 3, RLENGTH - length(key) - 3)
       sub(/[ \t].*/, "", value)
       if (key == "LENGTH") { sub(/K$/, "", value); value = value * 1024 }
       print value; exit
     }' "$3"
}

PLACED_ROM=$(lds_field rom LENGTH "$HERE/bench.lds")
PLACED_RAM=$(lds_field ram LENGTH "$HERE/bench.lds")
SIM_ROM=$(lds_field rom LENGTH "$HERE/coremark.lds")
SIM_RAM=$(lds_field ram LENGTH "$HERE/coremark.lds")
RAM_BASE=$(lds_field ram ORIGIN "$HERE/coremark.lds")
for pair in "placed rom length:$PLACED_ROM" "placed ram length:$PLACED_RAM" \
            "simulated rom length:$SIM_ROM" "simulated ram length:$SIM_RAM" \
            "simulated ram origin:$RAM_BASE"; do
  if [ -z "${pair#*:}" ]; then
    echo "error: could not read the ${pair%%:*} out of its linker script, so" >&2
    echo "there is no map to build the image against." >&2
    exit 1
  fi
done

CC=""
for candidate in riscv64-elf-gcc riscv64-unknown-elf-gcc; do
  if command -v "$candidate" >/dev/null 2>&1; then
    CC=$candidate
    break
  fi
done
if [ -z "$CC" ]; then
  echo "error: no RISC-V cross compiler found; see \`make setup\`." >&2
  exit 1
fi
OBJCOPY=${CC%gcc}objcopy
SIZE=${CC%gcc}size
NM=${CC%gcc}nm
for tool in "$OBJCOPY" "$SIZE" "$NM"; do
  command -v "$tool" >/dev/null 2>&1 || {
    echo "error: found $CC but not its matching $tool (half-installed toolchain)." >&2
    exit 1
  }
done

# Membership is a two-way match against PINNED.sha256, the same reason
# test/bench/run_coremark.sh checks it: a file dropped in beside the vendored
# tree that shasum was never told to look at is invisible to a one-way check.
# coremark.h's `#include "core_portme.h"` is a quoted include, which searches
# the including file's own directory FIRST, so an unlisted core_portme.h in
# $VENDOR_DIR would shadow this port's real header (which carries the timing
# hooks) for every vendored unit while shasum -c reports the tree unmodified.
manifest_files=$(awk '!/^#/ && NF { print $NF }' "$VENDOR_DIR/PINNED.sha256" | sort)
tree_files=$(cd "$VENDOR_DIR" && for f in *; do
  if [ -f "$f" ] && [ "$f" != "PINNED.sha256" ]; then
    echo "$f"
  fi
done | sort)
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
pin_check=$(mktemp "${TMPDIR:-/tmp}/coremark_compare_pin_check.XXXXXX") || {
  echo "error: could not create a temporary file under ${TMPDIR:-/tmp}." >&2
  exit 1
}
trap 'rm -f "$pin_check"' EXIT
if ! (cd "$VENDOR_DIR" && "${SHA_CHECK[@]}" PINNED.sha256) >"$pin_check" 2>&1; then
  cat "$pin_check" >&2
  echo >&2
  echo "error: $VENDOR_DIR no longer matches PINNED.sha256; run" >&2
  echo "'make coremark' first, which checks this the same way and explains it." >&2
  exit 1
fi
cat "$pin_check" >&2
rm -f "$pin_check"

tmp=$(mktemp -d "${TMPDIR:-/tmp}/compare-coremark.XXXXXX")
test -n "$tmp" -a -d "$tmp"
trap 'rm -rf "$tmp"' EXIT

# Six separate compilations, no -flto -- the same reason run_dhrystone.sh gives
# for Dhrystone's three: every published CoreMark number is from a build where
# the algorithm units cannot see across each other and inline the benchmark
# away.
objects=()
for unit in "$VENDOR_DIR/core_list_join" "$VENDOR_DIR/core_main" \
            "$VENDOR_DIR/core_matrix" "$VENDOR_DIR/core_state" \
            "$VENDOR_DIR/core_util" "$HERE/coremark_compare_port"; do
  name=$(basename "${unit%.c}")
  out="$tmp/$name.o"
  # shellcheck disable=SC2086
  $CC $CFLAGS -I "$REPO/test/bench" -I "$VENDOR_DIR" -DITERATIONS="$ITERATIONS" \
    "-DCOREMARK_FLAGS=\"$CFLAGS\"" -c "$unit.c" -o "$out"
  objects+=("$out")
done

elf="$tmp/coremark.elf"
# shellcheck disable=SC2086
if ! $CC $CFLAGS -nostdlib -T "$HERE/coremark.lds" -o "$elf" \
     "$HERE/coremark_start.S" "${objects[@]}" -lgcc 2> "$tmp/link.log"; then
  cat "$tmp/link.log" >&2
  echo >&2
  echo "*** CoreMark did not link. A region overflow here is a statement about" >&2
  echo "*** the harness, not something to fix by shrinking the benchmark: a" >&2
  echo "*** shrunk CoreMark is not CoreMark and its number is not comparable." >&2
  exit 1
fi
if [ -s "$tmp/link.log" ]; then
  cat "$tmp/link.log" >&2
  echo "error: the link produced diagnostics; warnings are errors here." >&2
  exit 1
fi

$OBJCOPY -O binary -j .text "$elf" "$tmp/rom.bin"
$OBJCOPY -O binary -j .data "$elf" "$tmp/data.bin"
rom_bytes=$(wc -c < "$tmp/rom.bin" | tr -d ' ')
ram_bytes=$($SIZE -A "$elf" \
  | awk '$1 == ".coremarkctl" || $1 == ".data" || $1 == ".bss" { total += $2 }
         END { print total + 0 }')

built_flags=$(LC_ALL=C tr -c '[:print:]' '\n' < "$tmp/data.bin" \
  | grep -m1 -- '-march=' || true)
if [ -z "$built_flags" ]; then
  echo "error: the flags string is not in the linked image, so the number below" >&2
  echo "would be quoted without the flags that produced it. Nothing was run." >&2
  exit 1
fi

echo "== CoreMark in the cross-core harness =="
echo "compiler : $CC $($CC -dumpversion)"
echo "flags    : $built_flags"
echo "mul/div  : hardware, both cores' own -- this image is RV32IMA"
echo "iterations : $ITERATIONS"
echo

echo "== the image against the geometry this harness can PLACE =="
printf 'rom (.text):                       %s bytes; placed budget %s\n' \
  "$rom_bytes" "$PLACED_ROM"
printf 'ram (.coremarkctl + .data + .bss): %s bytes; placed budget %s\n' \
  "$ram_bytes" "$PLACED_RAM"
python3 "$HERE/coremark_fit.py" --rom-bytes "$rom_bytes" --ram-bytes "$ram_bytes" \
  --placed-rom "$PLACED_ROM" --placed-ram "$PLACED_RAM" \
  --sim-rom "$SIM_ROM" --sim-ram "$SIM_RAM" --tb "$HERE/coremark_tb.v" \
  --core "littlecpu=$REPO/compare.littlecpu.core.log" \
  --core "hazard3=$REPO/compare.hazard3.core.log"
echo

$OBJCOPY -O verilog --verilog-data-width=4 -j .text "$elf" "$tmp/rom.hex"
test -s "$tmp/rom.hex" || { echo "error: objcopy produced an empty ROM image." >&2; exit 1; }
sim_rom_words=$((SIM_ROM / 4))
python3 "$REPO/soc/rom_banks.py" "$tmp/rom.hex" \
  "$HERE/coremark_even.hex" "$HERE/coremark_odd.hex" --rom-words "$sim_rom_words"
python3 "$HERE/rom_flat.py" "$tmp/rom.hex" "$HERE/coremark_flat.hex" \
  --rom-words "$sim_rom_words"

data_start=$($NM "$elf" | awk '$3 == "__data_start" { print "0x" $1 }')
if [ -z "$data_start" ]; then
  echo "error: __data_start is not in the linked image, so there is no offset to" >&2
  echo "poke the data image at. Nothing was run." >&2
  exit 1
fi
python3 "$HERE/dhry_ram.py" "$tmp/data.bin" "$HERE/coremark_ram.hex" \
  --ram-words $((SIM_RAM / 4)) \
  --offset-words $(((data_start - RAM_BASE) / 4))

if [ ! -x "$(command -v vvp 2>/dev/null || echo /nonexistent)" ]; then
  echo "error: vvp (iverilog) is not on PATH, so nothing can be run." >&2
  exit 1
fi
if [ ! -s "$VVP_BIN" ]; then
  echo "error: '$VVP_BIN' does not exist; build it with \`make compare-coremark\`." >&2
  exit 1
fi

echo "== both cores, one image, one simulation =="
set +e
vvp "$VVP_BIN" +cycles="$CYCLE_LIMIT" > "$tmp/run.log" 2>&1
sim_status=$?
set -e
cat "$tmp/run.log"
if [ "$sim_status" -ne 0 ]; then
  echo "*** the simulation failed; there is no number to report." >&2
  exit "$sim_status"
fi
echo

# COMPARE_COREMARK_MHZ is the operator's, the same reason COMPARE_DHRY_MHZ is:
# the clock belongs to a placement -- soc/compare/sweep.sh, read on the worst
# of five -- and a copy of it stored in this repository would be a number
# that stops tracking the RTL.
mhz_args=()
for spec in ${COMPARE_COREMARK_MHZ:-}; do
  mhz_args+=(--mhz "$spec")
done
python3 "$HERE/coremark_dmips.py" "$tmp/run.log" --iterations "$ITERATIONS" \
  "${mhz_args[@]+"${mhz_args[@]}"}"

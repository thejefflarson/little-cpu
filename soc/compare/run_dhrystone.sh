#!/bin/bash
# Builds one Dhrystone image for the core(s) named on the command line, reports
# it against the geometry the harness can actually place, and runs it in one
# iverilog simulation.
#
# Usage: run_dhrystone.sh <runs> <cycle-limit> <cflags> <muldiv> <vvp-binary> \
#          <cores-csv> <core=standalone-log>...
#
# <muldiv> is `hardware` or `libgcc`, the caller's own answer for whether
# <cflags>' `-march` includes M -- the Makefile already knows which of
# COMPARE_DHRY_CFLAGS or DHRY_CFLAGS built this image, so that is a fact to
# pass through rather than one for this script to re-derive by pattern-matching
# the flag string, which would also plant a second `-march=`-shaped literal for
# test/march_test.sh's scan of that exact flag to trip over.
#
# <cores-csv> is soc/compare/dhry_dmips.py's --cores: the first name is the
# reference core its data RAM is compared against every other name in the list,
# and is skipped entirely for a single-core run. Each `<core=standalone-log>`
# is that core's own yosys census, for soc/compare/dhry_fit.py's block-RAM
# arithmetic -- the same core names as <cores-csv>, but this script does not
# require the two lists to match, since a single-core solo run still wants
# every core's fit line reported for context.
#
# WHAT THIS IS AND IS NOT. `make compare-timing` places each core and reports a
# clock. This reports the other factor of throughput -- cycles for the same work
# -- from the same image, the same compiler and the same string routines, so
# that a DMIPS figure for any side is measured here rather than quoted from a
# project's own README.
#
# IT IS A SIMULATION AND CANNOT BE A PLACEMENT. Dhrystone needs more memory than
# an hx8k has block RAM for; soc/compare/dhry.lds carries the arithmetic and the
# report below prints it every run, because the distortion has to travel with the
# number. The clock to multiply these cycles by is `make compare-timing`'s, taken
# at the SMALLER placed geometry, and that mismatch is the headline caveat.
#
# The flags arrive from the Makefile, are compiled into the image as
# `dhry_flags`, and are read back OUT of the image below rather than reprinted
# from the argument -- so what is reported is what the image was built with.
set -euo pipefail

if [ "$#" -lt 7 ]; then
  echo "usage: run_dhrystone.sh <runs> <cycle-limit> <cflags> <muldiv> \\" >&2
  echo "         <vvp-binary> <cores-csv> <core=standalone-log>..." >&2
  exit 1
fi

RUNS=$1
CYCLE_LIMIT=$2
CFLAGS=$3
MULDIV=$4
VVP_BIN=$5
CORES_CSV=$6
shift 6
case "$MULDIV" in
  hardware|libgcc) ;;
  *) echo "error: <muldiv> is '$MULDIV', not 'hardware' or 'libgcc'." >&2; exit 1 ;;
esac
CORE_LOGS=("$@")
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)

# One reader for both of a MEMORY line's fields, because a second awk over the
# same file is a second thing that can stop matching it. LENGTH comes back in
# bytes; ORIGIN comes back as the literal the linker script states.
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
SIM_ROM=$(lds_field rom LENGTH "$HERE/dhry.lds")
SIM_RAM=$(lds_field ram LENGTH "$HERE/dhry.lds")
RAM_BASE=$(lds_field ram ORIGIN "$HERE/dhry.lds")
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

tmp=$(mktemp -d "${TMPDIR:-/tmp}/compare-dhrystone.XXXXXX")
test -n "$tmp" -a -d "$tmp"
trap 'rm -rf "$tmp"' EXIT

# THREE SEPARATE COMPILATIONS, NO -flto, for test/bench/run_dhrystone.sh's
# reason: letting the compiler see dhry_1.c and dhry_2.c at once lets it inline
# and then delete most of the benchmark, and no published number is built that
# way. The first two sources are the ones `make dhrystone` compiles, unedited --
# only the port differs, because none of the cores here can rely on `csrr
# mcycle` to time a run (VexRiscv has no CSR file; Hazard3's iCE40 build sets
# CSR_COUNTER=0), so soc/compare/dhry_port.c times off the bus instead.
objects=()
for unit in "$REPO/test/bench/dhry_1.c" "$REPO/test/bench/dhry_2.c" \
            "$HERE/dhry_port.c"; do
  out="$tmp/$(basename "${unit%.c}").o"
  # shellcheck disable=SC2086
  $CC $CFLAGS -I "$REPO/test/bench" -DDHRY_RUNS="$RUNS" \
    -DDHRY_FLAGS="\"$CFLAGS\"" -c "$unit" -o "$out"
  objects+=("$out")
done

elf="$tmp/dhrystone.elf"
# shellcheck disable=SC2086
if ! $CC $CFLAGS -nostdlib -T "$HERE/dhry.lds" -o "$elf" \
     "$HERE/dhry_start.S" "${objects[@]}" -lgcc 2> "$tmp/link.log"; then
  cat "$tmp/link.log" >&2
  echo >&2
  echo "*** Dhrystone did not link. A region overflow here is a statement about" >&2
  echo "*** the harness, not something to fix by trimming the benchmark: a" >&2
  echo "*** trimmed Dhrystone is not Dhrystone and its number is not comparable." >&2
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
  | awk '$1 == ".dhryctl" || $1 == ".data" || $1 == ".bss" { total += $2 }
         END { print total + 0 }')

# Out of the image, not out of "$CFLAGS": a string read back from the bytes the
# machine will actually run cannot be the flags of some other build. It is in
# `.data` because that is where this map puts read-only data -- see dhry.lds.
built_flags=$(LC_ALL=C tr -c '[:print:]' '\n' < "$tmp/data.bin" \
  | grep -m1 -- '-march=' || true)
if [ -z "$built_flags" ]; then
  echo "error: the flags string is not in the linked image, so the number below" >&2
  echo "would be quoted without the flags that produced it. Nothing was run." >&2
  exit 1
fi

case "$MULDIV" in
  hardware) muldiv_msg="hardware, this image's -march includes M" ;;
  libgcc)   muldiv_msg="libgcc calls, this image's -march has no M" ;;
esac

echo "== Dhrystone 2.1 in the cross-core harness =="
echo "compiler : $CC $($CC -dumpversion)"
echo "flags    : $built_flags"
echo "strings  : soc/compare/dhry_port.c's own byte loops, not a libc's"
echo "mul/div  : $muldiv_msg"
echo "runs     : $RUNS"
echo

echo "== the image against the geometry this harness can PLACE =="
printf 'rom (.text):                   %s bytes; placed budget %s\n' \
  "$rom_bytes" "$PLACED_ROM"
printf 'ram (.dhryctl + .data + .bss): %s bytes; placed budget %s\n' \
  "$ram_bytes" "$PLACED_RAM"
fit_core_args=()
for spec in "${CORE_LOGS[@]}"; do
  fit_core_args+=(--core "$spec")
done
python3 "$HERE/dhry_fit.py" --rom-bytes "$rom_bytes" --ram-bytes "$ram_bytes" \
  --placed-rom "$PLACED_ROM" --placed-ram "$PLACED_RAM" \
  --sim-rom "$SIM_ROM" --sim-ram "$SIM_RAM" --tb "$HERE/dhry_tb.v" \
  "${fit_core_args[@]}"
echo

$OBJCOPY -O verilog --verilog-data-width=4 -j .text "$elf" "$tmp/rom.hex"
test -s "$tmp/rom.hex" || { echo "error: objcopy produced an empty ROM image." >&2; exit 1; }
sim_rom_words=$((SIM_ROM / 4))
python3 "$REPO/soc/rom_banks.py" "$tmp/rom.hex" \
  "$HERE/dhry_even.hex" "$HERE/dhry_odd.hex" --rom-words "$sim_rom_words"
python3 "$HERE/rom_flat.py" "$tmp/rom.hex" "$HERE/dhry_flat.hex" \
  --rom-words "$sim_rom_words"

# The data RAM every core starts from. The offset comes from the linked symbol
# rather than from a constant here, so the image lands where the program's own
# pointers say it is.
data_start=$($NM "$elf" | awk '$3 == "__data_start" { print "0x" $1 }')
if [ -z "$data_start" ]; then
  echo "error: __data_start is not in the linked image, so there is no offset to" >&2
  echo "poke the data image at. Nothing was run." >&2
  exit 1
fi
python3 "$HERE/dhry_ram.py" "$tmp/data.bin" "$HERE/dhry_ram.hex" \
  --ram-words $((SIM_RAM / 4)) \
  --offset-words $(((data_start - RAM_BASE) / 4))

if [ ! -x "$(command -v vvp 2>/dev/null || echo /nonexistent)" ]; then
  echo "error: vvp (iverilog) is not on PATH, so nothing can be run." >&2
  exit 1
fi
if [ ! -s "$VVP_BIN" ]; then
  echo "error: '$VVP_BIN' does not exist; build it first." >&2
  exit 1
fi

echo "== $CORES_CSV, one image, one simulation =="
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

# COMPARE_DHRY_MHZ is the operator's, deliberately: the clock belongs to a
# placement -- `soc/compare/sweep.sh`, read on the worst of twelve -- and a copy
# of it stored in this repository would be a number that stops tracking the
# RTL. Set it to something like 'littlecpu=32.61 vexriscv=48.19 hazard3=32.60'
# for the absolute column.
mhz_args=()
for spec in ${COMPARE_DHRY_MHZ:-}; do
  mhz_args+=(--mhz "$spec")
done
python3 "$HERE/dhry_dmips.py" "$tmp/run.log" --runs "$RUNS" --cores "$CORES_CSV" \
  "${mhz_args[@]+"${mhz_args[@]}"}"

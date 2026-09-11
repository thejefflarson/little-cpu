#!/bin/bash
# Builds one CoreMark image for the core(s) named on the command line, reports it against
# the geometry the harness can actually place, and runs it in one iverilog simulation --
# the same split soc/compare/run_dhrystone.sh uses for Dhrystone.
set -euo pipefail

if [ "$#" -lt 3 ]; then
  echo "usage: run_coremark_compare.sh <iterations> <cycle-limit> <cflags> \\" >&2
  echo "         [vvp-binary [cores-csv [core=standalone-log]...]]" >&2
  exit 1
fi

ITERATIONS=$1
CYCLE_LIMIT=$2
CFLAGS=$3
VVP_BIN=${4:-compare.coremark.vvp}
CORES_CSV=${5:-littlecpu,vexriscv,hazard3}
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
VENDOR_DIR="$REPO/test/bench/coremark"

# Explicit core=log pairs win; otherwise derive one standalone-log path per core named in
# CORES_CSV, so the default core list has one source rather than a second, separately
# maintained array that could drift from it.
if [ "$#" -ge 6 ]; then
  shift 5
  CORE_LOGS=("$@")
else
  CORE_LOGS=()
  IFS=',' read -ra default_cores <<< "$CORES_CSV"
  for core in "${default_cores[@]}"; do
    CORE_LOGS+=("$core=$REPO/compare.$core.core.log")
  done
fi

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

# ONE implementation for every route that checks the vendored tree against
# PINNED.sha256, the same one 'make coremark' runs -- see that script's header for why
# membership is checked before any hash.
"$REPO/test/bench/coremark_pin_check.sh" "$VENDOR_DIR"

tmp=$(mktemp -d "${TMPDIR:-/tmp}/compare-coremark.XXXXXX")
test -n "$tmp" -a -d "$tmp"
trap 'rm -rf "$tmp"' EXIT

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
echo "mul/div  : hardware, every core here has its own real M"
echo "iterations : $ITERATIONS"
echo

echo "== the image against the geometry this harness can PLACE =="
printf 'rom (.text):                       %s bytes; placed budget %s\n' \
  "$rom_bytes" "$PLACED_ROM"
printf 'ram (.coremarkctl + .data + .bss): %s bytes; placed budget %s\n' \
  "$ram_bytes" "$PLACED_RAM"
fit_core_args=()
for spec in "${CORE_LOGS[@]}"; do
  fit_core_args+=(--core "$spec")
done
python3 "$HERE/coremark_fit.py" --rom-bytes "$rom_bytes" --ram-bytes "$ram_bytes" \
  --placed-rom "$PLACED_ROM" --placed-ram "$PLACED_RAM" \
  --sim-rom "$SIM_ROM" --sim-ram "$SIM_RAM" --tb "$HERE/coremark_tb.v" \
  "${fit_core_args[@]}"
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

mhz_args=()
for spec in ${COMPARE_COREMARK_MHZ:-}; do
  mhz_args+=(--mhz "$spec")
done
python3 "$HERE/coremark_dmips.py" "$tmp/run.log" --iterations "$ITERATIONS" \
  --cores "$CORES_CSV" "${mhz_args[@]+"${mhz_args[@]}"}"

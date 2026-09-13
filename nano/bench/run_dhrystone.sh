#!/bin/bash
# Builds Dhrystone 2.1 for nanocpu and runs it under nano-sim's --bench mode, which
# watches soc/compare/dhry_monitor.v's marker addresses instead of a CSR nano has none of.
set -euo pipefail

if [ "$#" -ne 4 ]; then
  echo "usage: run_dhrystone.sh <sim-binary> <runs> <cycle-limit> <nano-march-cflags>" >&2
  exit 1
fi

SIM=$1
RUNS=$2
CYCLE_LIMIT=$3
CFLAGS="$4 -O2 -std=c11 -ffreestanding -fno-tree-loop-distribute-patterns -Wall -Wextra -Werror"
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)

if [ ! -x "$SIM" ]; then
  echo "error: '$SIM' is not an executable runner; build it with 'make nano-sim'." >&2
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
  echo "error: no RISC-V cross compiler found; see 'make setup'." >&2
  exit 1
fi
OBJCOPY=${CC%gcc}objcopy

tmp=$(mktemp -d "${TMPDIR:-/tmp}/nano-dhrystone.XXXXXX")
test -n "$tmp" -a -d "$tmp"
trap 'rm -rf "$tmp"' EXIT

objects=()
# shellcheck disable=SC2086
for unit in "$REPO/test/bench/dhry_1.c" "$REPO/test/bench/dhry_2.c" "$REPO/soc/compare/dhry_port.c"; do
  out="$tmp/$(basename "${unit%.c}").o"
  $CC $CFLAGS -I "$REPO/test/bench" -DDHRY_RUNS="$RUNS" -DDHRY_FLAGS="\"$CFLAGS\"" \
    -c "$unit" -o "$out"
  objects+=("$out")
done

elf="$tmp/dhrystone.elf"
# shellcheck disable=SC2086
if ! $CC $CFLAGS -nostdlib -T "$HERE/dhry.lds" -o "$elf" \
     "$HERE/start.S" "${objects[@]}" -lgcc 2> "$tmp/link.log"; then
  cat "$tmp/link.log" >&2
  exit 1
fi
if [ -s "$tmp/link.log" ]; then
  cat "$tmp/link.log" >&2
  echo "error: the link produced diagnostics; warnings are errors here." >&2
  exit 1
fi

$OBJCOPY -O verilog --verilog-data-width=4 --only-section=.text "$elf" "$tmp/rom.hex"
$OBJCOPY -O verilog --verilog-data-width=4 --remove-section=.text "$elf" "$tmp/ram.hex"
for image in "$tmp/rom.hex" "$tmp/ram.hex"; do
  if [ ! -s "$image" ]; then
    echo "error: objcopy produced an empty $image." >&2
    exit 1
  fi
done

echo "== Dhrystone 2.1 on nanocpu =="
echo "compiler    : $CC $($CC -dumpversion)"
echo "flags       : $CFLAGS"
echo "runs        : $RUNS"
echo "memory model: nano/tb/nano_memory.v, behavioural, zero-wait-state, 20480 words (80 KB) flat"
echo "              -- a core-only figure. No QSPI/PSRAM front end exists yet, so this is not"
echo "              the Tiny Tapeout board's own timing."
echo

set +e
"$SIM" --rom "$tmp/rom.hex" --ram "$tmp/ram.hex" --cycles "$CYCLE_LIMIT" --bench \
  > "$tmp/run.log" 2>&1
sim_status=$?
set -e
cat "$tmp/run.log"
if [ "$sim_status" -ne 0 ]; then
  echo "*** the run did not reach a passing verdict (runner exit $sim_status)." >&2
  exit "$sim_status"
fi

python3 "$HERE/bench_report.py" "$tmp/run.log" --kind dhrystone --runs "$RUNS"

#!/bin/bash
# Sweeps nano's QSPI timing model (nano/tb/nano_qspi_memory.v) over one nano-qspi-sim per
# configuration, running Dhrystone and CoreMark against each. Reporting only, no ratchet.
set -euo pipefail

if [ "$#" -ne 1 ]; then
  echo "usage: run_qspi_timing.sh <nano-march-cflags>" >&2
  exit 1
fi

CFLAGS=$1
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
DHRY_RUNS=400
COREMARK_ITERATIONS=5
DHRY_CYCLE_LIMIT=600000000
COREMARK_CYCLE_LIMIT=2500000000

CONFIG_NAMES=(no-overlap fifo2 fifo4 fifo2-loop8 fifo2-loop16 fifo4-loop16 fifo4-loop16-qpi)
CONFIG_DEPTHS=(0 2 4 2 2 4 4)
CONFIG_WINDOWS=(0 0 0 8 16 16 16)
CONFIG_PREAMBLES=(24 24 24 24 24 24 20)

tmp=$(mktemp -d "${TMPDIR:-/tmp}/nano-qspi-timing.XXXXXX")
trap 'rm -rf "$tmp"' EXIT

echo "== control: nano-sim (zero-wait, nano/tb/nano_memory.v) =="
"$REPO/nano/bench/run_dhrystone.sh" "$REPO/nano-sim" "$DHRY_RUNS" 4000000 "$CFLAGS" \
  > "$tmp/control-dhry.log" 2>&1 || { cat "$tmp/control-dhry.log" >&2; exit 1; }
grep -E '^(cycles|DMIPS/MHz)' "$tmp/control-dhry.log"
"$REPO/nano/bench/run_coremark.sh" "$REPO/nano-sim" "$COREMARK_ITERATIONS" 20000000 "$CFLAGS" \
  > "$tmp/control-cm.log" 2>&1 || { cat "$tmp/control-cm.log" >&2; exit 1; }
grep -E '^(cycles|CoreMark/MHz)' "$tmp/control-cm.log"
echo

echo -e "config\tkind\tcycles\tper_unit\tmetric\tabsolute\texecute%\tparcel_wait%\tredirect_preamble%\tpsram_wait%"
for i in "${!CONFIG_NAMES[@]}"; do
  name=${CONFIG_NAMES[$i]} depth=${CONFIG_DEPTHS[$i]}
  window=${CONFIG_WINDOWS[$i]} preamble=${CONFIG_PREAMBLES[$i]}

  echo "building $name (depth=$depth window=$window preamble=$preamble)..." >&2
  make -C "$REPO" nano-qspi-sim \
    NANO_QSPI_PREFETCH_DEPTH="$depth" NANO_QSPI_LOOP_WINDOW="$window" \
    NANO_QSPI_PREAMBLE_CYCLES="$preamble" > "$tmp/$name.build.log" 2>&1 \
    || { tail -60 "$tmp/$name.build.log" >&2; exit 1; }
  sim="$tmp/$name.nano-qspi-sim"
  cp "$REPO/nano-qspi-sim" "$sim"

  "$REPO/nano/bench/run_dhrystone.sh" "$sim" "$DHRY_RUNS" "$DHRY_CYCLE_LIMIT" "$CFLAGS" \
    > "$tmp/$name.dhry.log" 2>&1 || { tail -60 "$tmp/$name.dhry.log" >&2; exit 1; }
  python3 "$HERE/qspi_timing_report.py" "$tmp/$name.dhry.log" --config "$name" \
    --kind dhrystone --runs "$DHRY_RUNS"

  "$REPO/nano/bench/run_coremark.sh" "$sim" "$COREMARK_ITERATIONS" "$COREMARK_CYCLE_LIMIT" "$CFLAGS" \
    > "$tmp/$name.cm.log" 2>&1 || { tail -60 "$tmp/$name.cm.log" >&2; exit 1; }
  python3 "$HERE/qspi_timing_report.py" "$tmp/$name.cm.log" --config "$name" \
    --kind coremark --runs "$COREMARK_ITERATIONS"
done

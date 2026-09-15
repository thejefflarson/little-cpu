#!/bin/bash
# Sweeps nano's QSPI timing model (nano/tb/nano_qspi_memory.v) over one nano-qspi-sim per
# configuration against Dhrystone and CoreMark. Reporting only, no ratchet, except that the zero-wait control must reproduce ADR-0182's own cycle counts exactly.
set -euo pipefail

if [ "$#" -ne 1 ]; then
  echo "usage: run_qspi_timing.sh <nano-march-cflags>" >&2
  exit 1
fi

CFLAGS=$1
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
DHRY_RUNS=200
COREMARK_ITERATIONS=5
DHRY_CYCLE_LIMIT=600000000
COREMARK_CYCLE_LIMIT=2500000000
CONTROL_DHRY_CYCLES=505295
CONTROL_COREMARK_CYCLES=9242400

# 16-parcel windows are excluded: both loop-buffer kinds take over 15M cycles under CoreMark without reaching its first marker; see the ADR for what was tried.
CONFIG_NAMES=(no-overlap fifo2 fifo4 fifo2-tagged8 fifo2-cam8)
CONFIG_DEPTHS=(0 2 4 2 2)
CONFIG_KINDS=(0 0 0 1 2)
CONFIG_WINDOWS=(0 0 0 8 8)

tmp=$(mktemp -d "${TMPDIR:-/tmp}/nano-qspi-timing.XXXXXX")
trap 'rm -rf "$tmp"' EXIT

keep_log() {  # source, destination name; NANO_QSPI_LOG_DIR set keeps a copy
  [ -n "${NANO_QSPI_LOG_DIR:-}" ] || return 0
  mkdir -p "$NANO_QSPI_LOG_DIR"
  cp "$1" "$NANO_QSPI_LOG_DIR/$2"
}

echo "== control: nano-sim (zero-wait, nano/tb/nano_memory.v) =="
"$REPO/nano/bench/run_dhrystone.sh" "$REPO/nano-sim" "$DHRY_RUNS" 4000000 "$CFLAGS" \
  > "$tmp/control-dhry.log" 2>&1 || { cat "$tmp/control-dhry.log" >&2; exit 1; }
keep_log "$tmp/control-dhry.log" control-dhry.log
control_dhry_cycles=$(grep -m1 '^BENCH ' "$tmp/control-dhry.log" | grep -oE 'cycles=[0-9]+' | cut -d= -f2)
if [ "$control_dhry_cycles" != "$CONTROL_DHRY_CYCLES" ]; then
  echo "error: control Dhrystone ($DHRY_RUNS runs) read $control_dhry_cycles cycles," \
    "not ADR-0182's $CONTROL_DHRY_CYCLES -- the zero-wait model no longer reproduces it." >&2
  exit 1
fi
grep -E '^(cycles|DMIPS/MHz)' "$tmp/control-dhry.log"

"$REPO/nano/bench/run_coremark.sh" "$REPO/nano-sim" "$COREMARK_ITERATIONS" 20000000 "$CFLAGS" \
  > "$tmp/control-cm.log" 2>&1 || { cat "$tmp/control-cm.log" >&2; exit 1; }
keep_log "$tmp/control-cm.log" control-coremark.log
control_cm_cycles=$(grep -m1 '^BENCH ' "$tmp/control-cm.log" | grep -oE 'cycles=[0-9]+' | cut -d= -f2)
if [ "$control_cm_cycles" != "$CONTROL_COREMARK_CYCLES" ]; then
  echo "error: control CoreMark ($COREMARK_ITERATIONS iterations) read $control_cm_cycles" \
    "cycles, not ADR-0182's $CONTROL_COREMARK_CYCLES." >&2
  exit 1
fi
grep -E '^(cycles|CoreMark/MHz)' "$tmp/control-cm.log"
echo

run_config() {  # name, depth, kind, window, preamble -> both report rows, DHRY_CYCLES_OUT
  local name=$1 depth=$2 kind=$3 window=$4 preamble=$5
  echo "building $name (depth=$depth kind=$kind window=$window preamble=$preamble)..." >&2
  make -C "$REPO" nano-qspi-sim NANO_QSPI_TAG="$name" NANO_QSPI_PREFETCH_DEPTH="$depth" \
    NANO_QSPI_LOOP_KIND="$kind" NANO_QSPI_LOOP_WINDOW="$window" \
    NANO_QSPI_PREAMBLE_CYCLES="$preamble" > "$tmp/$name.build.log" 2>&1 \
    || { tail -60 "$tmp/$name.build.log" >&2; exit 1; }
  keep_log "$tmp/$name.build.log" "$name.build.log"
  local sim="$REPO/nano-qspi-sim.$name"

  "$REPO/nano/bench/run_dhrystone.sh" "$sim" "$DHRY_RUNS" "$DHRY_CYCLE_LIMIT" "$CFLAGS" \
    > "$tmp/$name.dhry.log" 2>&1 || { tail -60 "$tmp/$name.dhry.log" >&2; exit 1; }
  keep_log "$tmp/$name.dhry.log" "$name.dhry.log"
  python3 "$HERE/qspi_timing_report.py" "$tmp/$name.dhry.log" --config "$name" \
    --kind dhrystone --runs "$DHRY_RUNS"
  DHRY_CYCLES_OUT=$(grep -m1 '^BENCH ' "$tmp/$name.dhry.log" | grep -oE 'cycles=[0-9]+' | cut -d= -f2)

  "$REPO/nano/bench/run_coremark.sh" "$sim" "$COREMARK_ITERATIONS" "$COREMARK_CYCLE_LIMIT" "$CFLAGS" \
    > "$tmp/$name.cm.log" 2>&1 || { tail -60 "$tmp/$name.cm.log" >&2; exit 1; }
  keep_log "$tmp/$name.cm.log" "$name.cm.log"
  python3 "$HERE/qspi_timing_report.py" "$tmp/$name.cm.log" --config "$name" \
    --kind coremark --runs "$COREMARK_ITERATIONS"
  rm -f "$sim"
}

echo -e "config\tkind\tcycles\tper_unit\tmetric\tabsolute\texecute%\tparcel_wait%\tredirect_preamble%\tloop_hit%\thandshake%\tpsram_wait%"
best_name="" best_depth="" best_kind="" best_window="" best_cycles=""
for i in "${!CONFIG_NAMES[@]}"; do
  name=${CONFIG_NAMES[$i]}
  run_config "$name" "${CONFIG_DEPTHS[$i]}" "${CONFIG_KINDS[$i]}" "${CONFIG_WINDOWS[$i]}" 24
  if [ -z "$best_cycles" ] || [ "$DHRY_CYCLES_OUT" -lt "$best_cycles" ]; then
    best_name=$name best_depth=${CONFIG_DEPTHS[$i]} best_kind=${CONFIG_KINDS[$i]}
    best_window=${CONFIG_WINDOWS[$i]} best_cycles=$DHRY_CYCLES_OUT
  fi
done

# QPI (preamble 20 rather than 24) is only worth trying on the fewest-cycle config above.
run_config "$best_name-qpi" "$best_depth" "$best_kind" "$best_window" 20

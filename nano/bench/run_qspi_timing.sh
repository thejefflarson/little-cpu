#!/bin/bash
# Sweeps nano's QSPI timing model (nano/tb/nano_qspi_memory.v) over one nano-qspi-sim per
# configuration against Dhrystone and CoreMark. Reporting only, no ratchet, except that the zero-wait control must reproduce nano/bench/QSPI_CONTROL's cycle counts exactly. --control-only stops after it.
set -euo pipefail

CONTROL_ONLY=0
if [ "$#" -eq 2 ] && [ "$2" = "--control-only" ]; then
  CONTROL_ONLY=1
elif [ "$#" -ne 1 ]; then
  echo "usage: run_qspi_timing.sh <nano-march-cflags> [--control-only]" >&2
  exit 1
fi

CFLAGS=$1
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
DHRY_RUNS=200
COREMARK_ITERATIONS=5
DHRY_CYCLE_LIMIT=600000000
COREMARK_CYCLE_LIMIT=2500000000
CONTROL_FILE=$HERE/QSPI_CONTROL

CONFIG_NAMES=(no-overlap fifo2 fifo4 fifo2-tagged8 fifo2-cam8 fifo2-tagged16 fifo2-cam16)
CONFIG_DEPTHS=(0 2 4 2 2 2 2)
CONFIG_KINDS=(0 0 0 1 2 1 2)
CONFIG_WINDOWS=(0 0 0 8 8 16 16)

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
bench_cycles() {  # a log -> the cycles on its first BENCH line; no such line is an error, not an empty answer
  local cycles
  cycles=$(grep -m1 '^BENCH ' "$1" | grep -oE 'cycles=[0-9]+' | cut -d= -f2) || cycles=""
  if [ -z "$cycles" ]; then
    echo "error: no BENCH line with a cycle count in $1: that run did not happen." >&2
    exit 1
  fi
  printf '%s' "$cycles"
}

"$HERE/qspi_control_check.sh" "$tmp/control-dhry.log" dhrystone "$DHRY_RUNS" "$CONTROL_FILE" || exit 1
grep -E '^(cycles|DMIPS/MHz)' "$tmp/control-dhry.log"

"$REPO/nano/bench/run_coremark.sh" "$REPO/nano-sim" "$COREMARK_ITERATIONS" 30000000 "$CFLAGS" \
  > "$tmp/control-cm.log" 2>&1 || { cat "$tmp/control-cm.log" >&2; exit 1; }
keep_log "$tmp/control-cm.log" control-coremark.log
"$HERE/qspi_control_check.sh" "$tmp/control-cm.log" coremark "$COREMARK_ITERATIONS" "$CONTROL_FILE" || exit 1
grep -E '^(cycles|CoreMark/MHz)' "$tmp/control-cm.log"
echo
[ "$CONTROL_ONLY" -eq 0 ] || exit 0

run_config() {  # name, depth, kind, window, preamble -> both report rows on stdout, Dhrystone cycles in $tmp/<name>.dhry_cycles
  local name=$1 depth=$2 kind=$3 window=$4 preamble=$5
  echo "building $name (depth=$depth kind=$kind window=$window preamble=$preamble)..." >&2
  make -C "$REPO" nano-qspi-sim NANO_QSPI_TAG="$name" NANO_QSPI_PREFETCH_DEPTH="$depth" \
    NANO_QSPI_LOOP_KIND="$kind" NANO_QSPI_LOOP_WINDOW="$window" \
    NANO_QSPI_PREAMBLE_CYCLES="$preamble" > "$tmp/$name.build.log" 2>&1 \
    || { tail -60 "$tmp/$name.build.log" >&2; exit 1; }
  keep_log "$tmp/$name.build.log" "$name.build.log"
  local sim="$REPO/nano-qspi-sim.$name"
  local memory="nano/tb/nano_qspi_memory.v, QSPI timing model: depth=$depth loop_kind=$kind loop_window=$window preamble=$preamble"

  NANO_BENCH_MEMORY="$memory" "$REPO/nano/bench/run_dhrystone.sh" "$sim" "$DHRY_RUNS" "$DHRY_CYCLE_LIMIT" "$CFLAGS" \
    > "$tmp/$name.dhry.log" 2>&1 || { tail -60 "$tmp/$name.dhry.log" >&2; exit 1; }
  keep_log "$tmp/$name.dhry.log" "$name.dhry.log"
  python3 "$HERE/qspi_timing_report.py" "$tmp/$name.dhry.log" --config "$name" \
    --kind dhrystone --runs "$DHRY_RUNS" --depth "$depth" --loop-kind "$kind" \
    --loop-window "$window" --preamble "$preamble" || exit 1
  bench_cycles "$tmp/$name.dhry.log" > "$tmp/$name.dhry_cycles" || exit 1

  NANO_BENCH_MEMORY="$memory" "$REPO/nano/bench/run_coremark.sh" "$sim" "$COREMARK_ITERATIONS" "$COREMARK_CYCLE_LIMIT" "$CFLAGS" \
    > "$tmp/$name.cm.log" 2>&1 || { tail -60 "$tmp/$name.cm.log" >&2; exit 1; }
  keep_log "$tmp/$name.cm.log" "$name.cm.log"
  python3 "$HERE/qspi_timing_report.py" "$tmp/$name.cm.log" --config "$name" \
    --kind coremark --runs "$COREMARK_ITERATIONS" --depth "$depth" --loop-kind "$kind" \
    --loop-window "$window" --preamble "$preamble" || exit 1
  rm -f "$sim"
}

echo -e "config\tkind\tcycles\tper_unit\tmetric\tabsolute\texecute%\tparcel_wait%\tredirect_preamble%\tloop_hit%\thandshake%\tpsram_wait%"

# CoreMark is a hundred million cycles a configuration in this model, so the configurations run
# side by side; every row is printed in table order once all of them have finished.
make -C "$REPO" rvfi_macros.vh test/monitor.sim.v > "$tmp/prereq.build.log" 2>&1 \
  || { tail -60 "$tmp/prereq.build.log" >&2; exit 1; }
pids=()
for i in "${!CONFIG_NAMES[@]}"; do
  run_config "${CONFIG_NAMES[$i]}" "${CONFIG_DEPTHS[$i]}" "${CONFIG_KINDS[$i]}" "${CONFIG_WINDOWS[$i]}" 24 \
    > "$tmp/${CONFIG_NAMES[$i]}.rows" &
  pids+=($!)
done
failed=0
for pid in "${pids[@]}"; do
  wait "$pid" || failed=1
done
[ "$failed" -eq 0 ] || { echo "error: a configuration failed above; no table was printed." >&2; exit 1; }

best_name="" best_depth="" best_kind="" best_window="" best_cycles=""
for i in "${!CONFIG_NAMES[@]}"; do
  name=${CONFIG_NAMES[$i]}
  cat "$tmp/$name.rows"
  cycles=$(cat "$tmp/$name.dhry_cycles")
  if [ -z "$best_cycles" ] || [ "$cycles" -lt "$best_cycles" ]; then
    best_name=$name best_depth=${CONFIG_DEPTHS[$i]} best_kind=${CONFIG_KINDS[$i]}
    best_window=${CONFIG_WINDOWS[$i]} best_cycles=$cycles
  fi
done

# QPI (preamble 20 rather than 24) is only worth trying on the fewest-cycle config above.
run_config "$best_name-qpi" "$best_depth" "$best_kind" "$best_window" 20

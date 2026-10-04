#!/bin/bash
# Grades a zero-wait benchmark log's cycle count against nano/bench/QSPI_CONTROL, the one
# declared source for run_qspi_timing.sh's control. Format and provenance:
# docs/manifests/qspi-control.md.
set -euo pipefail

if [ "$#" -ne 4 ]; then
  echo "usage: qspi_control_check.sh <log> <dhrystone|coremark> <runs> <control-file>" >&2
  exit 2
fi

log=$1 kind=$2 runs=$3 control=$4

[ -f "$log" ] || { echo "error: no log at $log: that run did not happen." >&2; exit 1; }
[ -f "$control" ] || { echo "error: no control file at $control." >&2; exit 1; }

observed=$(grep -m1 '^BENCH ' "$log" | grep -oE 'cycles=[0-9]+' | cut -d= -f2) || observed=""
if [ -z "$observed" ]; then
  echo "error: no BENCH line with a cycle count in $log: that run did not happen." >&2
  exit 1
fi

lines=$(grep -E "^$kind[[:space:]]" "$control") || lines=""
if [ "$(printf '%s' "$lines" | grep -c .)" -ne 1 ]; then
  echo "error: $control must hold exactly one '$kind <runs> <cycles>' line." >&2
  exit 1
fi
read -r _ want_runs want_cycles extra <<< "$lines"
if [ -n "$extra" ] || ! [[ "$want_runs" =~ ^[0-9]+$ && "$want_cycles" =~ ^[0-9]+$ ]]; then
  echo "error: malformed $kind line in $control: '$lines'." >&2
  exit 1
fi
if [ "$want_runs" != "$runs" ]; then
  echo "error: $control records $kind at $want_runs runs; this sweep runs $runs." >&2
  exit 1
fi
if [ "$observed" != "$want_cycles" ]; then
  echo "error: control $kind ($runs runs) read $observed cycles, not the $want_cycles in $control" \
    "-- nano's zero-wait timing or the toolchain moved. If the change is intended, re-take the" \
    "control (docs/manifests/qspi-control.md); otherwise this is a regression." >&2
  exit 1
fi

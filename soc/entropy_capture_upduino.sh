#!/bin/bash
# Builds the capture bitstream for the UPduino's SB_LFOSC; soc/entropy_upduino.py reads it back.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT"
if [ "$#" -lt 3 ] || [ "$#" -gt 6 ]; then
  echo "usage: entropy_capture_upduino.sh <tag> <crystal|internal> <intervals|stream> [holdoff_log2] [fab_log2] [fill] -> build/entropy_<tag>.bin" >&2
  exit 2
fi
TAG=$1
case $TAG in ''|*[!A-Za-z0-9_-]*) echo "error: tag '$TAG' must match [A-Za-z0-9_-]+" >&2; exit 2;; esac
case $2 in crystal) OSC=0;; internal) OSC=1;; *) echo "error: the clock is 'crystal' or 'internal', got '$2'" >&2; exit 2;; esac
case $3 in intervals) STREAM=0;; stream) STREAM=1;; *) echo "error: the capture is 'intervals' or 'stream', got '$3'" >&2; exit 2;; esac
HOLD=${4-4}; FAB=${5-0}; FILL=${6-0}
for v in "holdoff_log2=$HOLD" "fab_log2=$FAB" "fill=$FILL"; do
  case ${v#*=} in ''|*[!0-9]*) echo "error: ${v%%=*} must be digits, got '${v#*=}'" >&2; exit 2;; esac
done
mkdir -p build
B=build/entropy_$TAG
SRCS="soc/entropy_capture_upduino.v soc/entropy_interval_capture.v soc/entropy_bit_framer.v soc/miso_share_enable.v"
yosys -p "read_verilog -sv $SRCS; chparam -set INTERNAL_OSC $OSC -set STREAM $STREAM -set HOLDOFF_LOG2 $HOLD -set FAB_LOG2 $FAB -set FILL $FILL entropy_capture_upduino; synth_ice40 -device u -top entropy_capture_upduino -json $B.json" > "$B.synth.log" 2>&1 || { tail -40 "$B.synth.log"; exit 1; }
rm -f "$B.asc"
status=0
nextpnr-ice40 --up5k --package sg48 --pcf soc/upduino.pcf --json "$B.json" --asc "$B.asc" > "$B.pnr.log" 2>&1 || status=$?
sh soc/pnr_check.sh 'soc/entropy_capture_upduino.sh' "$status" "$B.pnr.log" "$B.asc"
icepack "$B.asc" "$B.bin"
grep -E 'Max frequency for clock' "$B.pnr.log" | tail -2
ls -l "$B.bin"

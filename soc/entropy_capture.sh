#!/bin/bash
# Builds the capture bitstream for one OSCG setting; soc/entropy_assess.py reads it back.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT"
if [ "$#" -ne 4 ]; then
  echo "usage: entropy_capture.sh <tag> <OSC_DIV> <FAB_LOG2> <CORE_PAD> -> build/entropy_<tag>.bit" >&2
  exit 2
fi
TAG=$1; DIV=$2; FAB=$3; PAD=$4
mkdir -p build
B=build/entropy_$TAG
yosys -p "read_verilog -sv soc/entropy_capture_icesugar_pro.v soc/icesugar_pro_entropy.v soc/icesugar_pro_pll.v; chparam -set OSC_DIV $DIV -set FAB_LOG2 $FAB -set CORE_PAD $PAD entropy_capture_icesugar_pro; synth_ecp5 -top entropy_capture_icesugar_pro -json $B.json" > $B.synth.log 2>&1 || { tail -40 $B.synth.log; exit 1; }
python3 soc/bram_reset_check.py $B.json --gate 'soc/entropy_capture.sh'
nextpnr-ecp5 --25k --package CABGA256 --speed 6 --json $B.json --lpf soc/icesugar_pro.lpf --textcfg $B.config > $B.pnr.log 2>&1 || { tail -30 $B.pnr.log; exit 1; }
ecppack $B.config $B.bit
grep -E 'Max frequency for clock' $B.pnr.log | tail -2
ls -l $B.bit

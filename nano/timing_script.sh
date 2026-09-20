#!/bin/sh
# Prints `make nano-timing`'s yosys -p script: ABC's `stime -c` and `stat -liberty -json`
# give delay and area from one run. `techmap -map` maps NANO_LATCH_RF's latches to real cells (a no-op on the flops build); without it `stat -liberty` prices them at zero.
liberty=$1
latchmap=$2
stat_json=$3
define=$4
shift 4

defines=""
if [ -n "$define" ]; then
  defines=" -D $define"
fi

srcs=""
for f in "$@"; do
  srcs="$srcs \"$f\""
done

printf 'read_verilog -sv%s%s; hierarchy -auto-top; flatten -noscopeinfo; synth; dfflibmap -liberty "%s"; techmap -map "%s"; abc -liberty "%s" -script +strash;dch,-f;map,-B,0.2;topo;stime,-c; tee -o "%s" stat -liberty "%s" -json\n' \
  "$defines" "$srcs" "$liberty" "$latchmap" "$liberty" "$stat_json" "$liberty"

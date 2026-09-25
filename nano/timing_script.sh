#!/bin/sh
# Prints `make nano-timing`'s yosys -p script: ABC's `stime -c` and `stat -liberty -json`
# give delay and area from one run.
liberty=$1
stat_json=$2
shift 2

srcs=""
for f in "$@"; do
  srcs="$srcs \"$f\""
done

printf 'read_verilog -sv%s; hierarchy -auto-top; flatten -noscopeinfo; synth; dfflibmap -liberty "%s"; abc -liberty "%s" -script +strash;dch,-f;map,-B,0.2;topo;stime,-c; tee -o "%s" stat -liberty "%s" -json\n' \
  "$srcs" "$liberty" "$liberty" "$stat_json" "$liberty"

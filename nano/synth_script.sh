#!/bin/sh
# Prints `make nano-area`'s yosys -p script, quoting every path (an unquoted `;` opens a
# second command). No `techmap -map`: NANO_LATCH_RF's latches would map to nothing and price at zero here silently -- nano/timing_script.sh carries that fix.
liberty=$1
shift
srcs=""
for f in "$@"; do
  srcs="$srcs \"$f\""
done
printf 'read_verilog -sv%s; hierarchy -auto-top; synth; dfflibmap -liberty "%s"; abc -liberty "%s"; tee -o nano/area.json stat -liberty "%s" -json\n' \
  "$srcs" "$liberty" "$liberty" "$liberty"

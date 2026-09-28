#!/bin/sh
# Prints `make nano-area`'s yosys -p script, quoting every path (an unquoted `;` opens a
# second command).
liberty=$1
shift
srcs=""
for f in "$@"; do
  srcs="$srcs \"$f\""
done
printf 'read_verilog -sv%s; hierarchy -auto-top; flatten -noscopeinfo; synth; clockgate -min_net_size 8 -pos sky130_fd_sc_hd__dlclkp_1 GATE:CLK:GCLK; dfflibmap -liberty "%s"; abc -liberty "%s"; tee -o nano/area.json stat -liberty "%s" -json\n' \
  "$srcs" "$liberty" "$liberty" "$liberty"

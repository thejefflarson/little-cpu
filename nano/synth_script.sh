#!/bin/sh
# Prints `make nano-area`'s yosys -p script, quoting every path (an unquoted `;` opens a
# second command).
liberty=$1
shift
srcs=""
for f in "$@"; do
  srcs="$srcs \"$f\""
done
printf 'read_verilog -sv%s; hierarchy -auto-top; flatten -noscopeinfo; synth; dfflibmap -liberty "%s"; abc -liberty "%s"; tee -o nano/area.json stat -liberty "%s" -json\n' \
  "$srcs" "$liberty" "$liberty" "$liberty"

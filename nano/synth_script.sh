#!/bin/sh
# Prints `make nano-area`'s yosys -p script, quoting every path as its own token: an
# unquoted `;` in a TOOL_CACHE-derived path is a second yosys command, and a space splits it.
liberty=$1
shift
srcs=""
for f in "$@"; do
  srcs="$srcs \"$f\""
done
printf 'read_verilog -sv%s; hierarchy -auto-top; synth; dfflibmap -liberty "%s"; abc -liberty "%s"; tee -o nano/area.json stat -liberty "%s" -json\n' \
  "$srcs" "$liberty" "$liberty" "$liberty"

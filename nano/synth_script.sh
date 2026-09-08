#!/bin/sh
# Prints the yosys -p script `make nano-area` runs, with every path quoted as its
# own yosys-script token. NANO_LIBERTY and NANO_SRCS both derive from TOOL_CACHE
# (XDG_CACHE_HOME or HOME), and yosys's own script parser treats an unquoted `;`
# as a command separator -- reachable through its own `exec` pass -- so an
# unquoted path handed one was a second command, not a typo; unquoted, a space in
# the path (a stock, unremarkable $HOME on macOS) splits it into two arguments.
#
# Usage: synth_script.sh LIBERTY SRC [SRC...]
liberty=$1
shift
srcs=""
for f in "$@"; do
  srcs="$srcs \"$f\""
done
printf 'read_verilog -sv%s; hierarchy -auto-top; synth; dfflibmap -liberty "%s"; abc -liberty "%s"; tee -o nano/area.json stat -liberty "%s" -json\n' \
  "$srcs" "$liberty" "$liberty" "$liberty"

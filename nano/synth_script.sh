#!/bin/sh
# Prints `make nano-area`'s yosys -p script with every path quoted (an unquoted `;` opens a
# second command) and excluded cell names bare, since dfflibmap matches -dont_use literally.
liberty=$1
excluded=$2
shift 2
if [ ! -r "$excluded" ]; then
  echo "error: no excluded-cell list at '$excluded'; refusing to measure cells the flow never uses." >&2
  exit 1
fi
srcs=""
for f in "$@"; do
  srcs="$srcs \"$f\""
done
cells=$(grep -v -e '^#' -e '^[[:space:]]*$' "$excluded" | sort -u)
if printf '%s\n' "$cells" | sed '/^$/d' | grep -v -q -x -E 'sky130_fd_sc_hd__[a-z0-9_]+'; then
  echo "error: '$excluded' names something other than a sky130_fd_sc_hd cell." >&2
  exit 1
fi
dont_use=$(printf '%s\n' "$cells" | sed '/^$/d; s/.*/ -dont_use &/' | tr -d '\n')
printf 'read_verilog -sv%s; hierarchy -auto-top; blackbox rf_top; flatten -noscopeinfo; synth; dfflibmap -liberty "%s"%s; abc -liberty "%s"%s; tee -o nano/area.json stat -liberty "%s" -json\n' \
  "$srcs" "$liberty" "$dont_use" "$liberty" "$dont_use" "$liberty"

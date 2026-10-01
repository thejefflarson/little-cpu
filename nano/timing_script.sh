#!/bin/sh
# Prints `make nano-timing`'s yosys -p script, delay and area from one run; excluded cell
# names go bare, since dfflibmap matches -dont_use literally, quotes included.
liberty=$1
excluded=$2
stat_json=$3
shift 3
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

printf 'read_liberty -overwrite -setattr liberty_cell -lib "%s"; read_verilog -sv -D SCL_sky130_fd_sc_hd%s; hierarchy -auto-top; flatten -noscopeinfo; synth; dfflibmap -liberty "%s"%s; abc -liberty "%s"%s -script +strash;dch,-f;map,-B,0.2;topo;stime,-c; tee -o "%s" stat -liberty "%s" -json\n' \
  "$liberty" "$srcs" "$liberty" "$dont_use" "$liberty" "$dont_use" "$stat_json" "$liberty"

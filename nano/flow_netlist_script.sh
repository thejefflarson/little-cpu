#!/bin/sh
# Prints the yosys -p script for a gate-level netlist of the tt_um top: the flow's SCL define, the liberty as a blackbox library, then flatten, synth, dfflibmap and abc over the allowed cells.
# usage: flow_netlist_script.sh <liberty> <excluded cells> <out.v> <source.v>...
liberty=$1
excluded=$2
out=$3
shift 3
if [ ! -r "$excluded" ]; then
  echo "error: no excluded-cell list at '$excluded'; refusing to map cells the flow never uses." >&2
  exit 1
fi

srcs=""
for f in "$@"; do
  srcs="$srcs \"$f\""
done
dont_use=$(grep -v -e '^#' -e '^[[:space:]]*$' "$excluded" | sort -u | sed 's/.*/ -dont_use &/' | tr -d '\n')

printf 'read_liberty -overwrite -setattr liberty_cell -lib "%s"; read_verilog -sv -D SCL_sky130_fd_sc_hd%s; hierarchy -top tt_um_thejefflarson_nanocpu; flatten -noscopeinfo; synth; dfflibmap -liberty "%s"%s; abc -liberty "%s"%s; hilomap -hicell sky130_fd_sc_hd__conb_1 HI -locell sky130_fd_sc_hd__conb_1 LO; opt_clean -purge; write_verilog -noattr -noexpr -nohex -nodec -defparam "%s"\n' \
  "$liberty" "$srcs" "$liberty" "$dont_use" "$liberty" "$dont_use" "$out" 

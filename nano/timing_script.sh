#!/bin/sh
# Prints `make nano-timing`'s yosys -p script: ABC's `stime -c` and `stat -liberty -json`
# give delay and area from one run, over the cells the flow allows.
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
dont_use=$(grep -v -e '^#' -e '^[[:space:]]*$' "$excluded" | sort -u | sed 's/.*/ -dont_use "&"/' | tr -d '\n')

printf 'read_verilog -sv%s; hierarchy -auto-top; flatten -noscopeinfo; synth; dfflibmap -liberty "%s"%s; abc -liberty "%s"%s -script +strash;dch,-f;map,-B,0.2;topo;stime,-c; tee -o "%s" stat -liberty "%s" -json\n' \
  "$srcs" "$liberty" "$dont_use" "$liberty" "$dont_use" "$stat_json" "$liberty"

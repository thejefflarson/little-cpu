#!/bin/sh
# Prints `make nano-area`'s yosys -p script, quoting every path (an unquoted `;` opens a
# second command). Cells the flow excludes from synthesis are excluded here too.
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
dont_use=$(grep -v -e '^#' -e '^[[:space:]]*$' "$excluded" | sort -u | sed 's/.*/ -dont_use "&"/' | tr -d '\n')
printf 'read_verilog -sv%s; hierarchy -auto-top; flatten -noscopeinfo; synth; dfflibmap -liberty "%s"%s; abc -liberty "%s"%s; tee -o nano/area.json stat -liberty "%s" -json\n' \
  "$srcs" "$liberty" "$dont_use" "$liberty" "$dont_use" "$liberty"

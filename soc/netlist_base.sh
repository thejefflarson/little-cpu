#!/bin/sh
# Synthesise another commit's canonical netlist, so `make netlist-diff` has something to
# compare this tree's against.
set -eu

cd "$(dirname "$0")/.."

if [ -z "${NETLIST_SYNTH:-}" ]; then
  echo "*** make netlist-diff: NETLIST_SYNTH is not set, so there is nothing to" >&2
  echo "*** synthesise. The Makefile's part table sets it." >&2
  exit 2
fi
if [ "$#" -ne 2 ]; then
  echo "*** usage: soc/netlist_base.sh <ref> <output json>" >&2
  exit 2
fi

ref=$1
out=$2
case $out in
  /*) ;;
  *)  out=$PWD/$out ;;
esac

git rev-parse --verify --quiet "$ref^{commit}" > /dev/null || {
  echo "*** make netlist-diff: '$ref' does not name a commit here, so there is" >&2
  echo "*** nothing to compare against." >&2
  exit 2
}

tree=$(mktemp -d "${TMPDIR:-/tmp}/netlist-base.XXXXXX")
test -n "$tree" && test -d "$tree" || {
  echo "*** make netlist-diff: could not create a temporary directory." >&2
  exit 2
}
trap 'rm -rf "$tree"' EXIT

git archive --format=tar "$ref" | tar -x -C "$tree"

for src in $(printf '%s\n' "$NETLIST_SYNTH" | tr ' ;' '\n\n' | grep '\.v$' || true); do
  test -e "$tree/$src" || {
    echo "*** make netlist-diff: $ref has no $src, so the two trees do not" >&2
    echo "*** synthesise the same source list and their netlists are not" >&2
    echo "*** comparable. Spend the seeds." >&2
    exit 2
  }
done

echo "netlist-diff: building $ref's ROM with its own recipe"
make --no-print-directory -C "$tree" soc-rom SOC_PROG="${SOC_PROG:-datainit.c}" \
  > "$tree/rom.log" 2>&1 || {
  tail -20 "$tree/rom.log" >&2
  echo "*** make netlist-diff: $ref could not build its ROM image." >&2
  exit 2
}

if base_print=$(env -u NETLIST_SYNTH make --no-print-directory -C "$tree" \
                  print-NETLIST_SYNTH 2> "$tree/print.log"); then
  base_synth=$(printf '%s\n' "$base_print" | tail -1)
elif grep -q 'No rule to make target' "$tree/print.log"; then
  base_synth=""
else
  cat "$tree/print.log" >&2
  echo "*** make netlist-diff: $ref's make could not be asked which synth" >&2
  echo "*** script it uses, and its netlist is not comparable without that." >&2
  echo "*** Spend the seeds." >&2
  exit 2
fi

if [ -z "$base_synth" ]; then
  base_synth=$NETLIST_SYNTH
  echo "netlist-diff: $ref names no synth script of its own, so this tree's was"
  echo "netlist-diff: used. A flag that moved between the two is invisible here."
elif [ "$base_synth" != "$NETLIST_SYNTH" ]; then
  echo "netlist-diff: $ref synthesises with a different script, and it is that"
  echo "netlist-diff: script's netlist being digested:"
  echo "netlist-diff:   $base_synth"
fi

( cd "$tree" && yosys -p "$base_synth; opt_clean -purge; write_json $out" ) \
  > "$tree/synth.log" 2>&1 || {
  tail -40 "$tree/synth.log" >&2
  echo "*** make netlist-diff: yosys could not synthesise $ref." >&2
  exit 2
}
echo "netlist-diff: $ref's canonical netlist is $out"

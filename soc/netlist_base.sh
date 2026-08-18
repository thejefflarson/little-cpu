#!/bin/sh
# Synthesise another commit's canonical netlist, so `make netlist-diff` has
# something to compare this tree's against.
#
#   soc/netlist_base.sh <ref> <output json>
#
# `git archive` rather than a worktree or a checkout: it hands over tracked files
# only, leaves no bookkeeping behind in the repository, and cannot disturb
# whatever branch the caller is on. The ROM is then built inside that tree by the
# tree's OWN Makefile, because the image is part of what gets synthesised and
# building it with this tree's recipe would compare a program against itself.
#
# THE SYNTH SCRIPT IS THE BASE TREE'S OWN where it names one. A comparison run
# with this tree's flags on both sides is blind to a change in the flags -- which
# is a change to the placer's input like any other, and the one this gate would
# be most embarrassed to miss. A tree from before this target existed names none,
# and then this tree's is used and the report says so.
#
# BUILDING ANOTHER COMMIT MEANS RUNNING ITS RECIPES -- its `soc-rom` and its
# synth script both come from the extracted tree. That is the same trust as
# `git checkout <ref> && make`, and it is why the argument is a ref in this
# repository and nothing this script fetches.
#
# Driven by the Makefile, which owns the part table: NETLIST_SYNTH, and SOC_PROG
# for the ROM image.
set -eu

cd "$(dirname "$0")/.."

# Spelled out rather than left to `${VAR:?}` for the reason its neighbour gives:
# that construct's exit status belongs to whichever /bin/sh is running it.
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

# The source list travels with the tree, so a file this tree synthesises that
# the base does not have is not a netlist difference to report -- it is a
# comparison that cannot be made. Say which file, and let the seeds be spent.
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

# `env -u` because make lets a variable the makefile does not define fall
# through from the environment: with this tree's script passed in, the base tree
# would print it back and the comparison below could never see a difference.
#
# NOT A PIPELINE, and the status is make's own. `set -eu` carries no pipefail, so
# `make ... | tail -1` collects TAIL's status, which is always zero -- and then
# every way the base tree's make can fail, a Makefile that errors during parse or
# a `print-%` rule that is gone included, reads as the one case meant to be
# handled here. The base commit would be synthesised with THIS tree's flags while
# the report said nothing and the digest said "no sweep is owed".
if base_print=$(env -u NETLIST_SYNTH make --no-print-directory -C "$tree" \
                  print-NETLIST_SYNTH 2> "$tree/print.log"); then
  base_synth=$(printf '%s\n' "$base_print" | tail -1)
elif grep -q 'No rule to make target' "$tree/print.log"; then
  # The one expected failure: a tree from before this target existed has no
  # `print-%` rule at all.
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

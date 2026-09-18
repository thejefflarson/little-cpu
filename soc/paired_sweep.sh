#!/bin/sh
# ONE COMMAND: a named base ref against the working tree, both parts, paired by seed.
#
# Reuses soc/baseline_sweep.sh for each half rather than re-implementing placement, so
# a base ref sweeps in an extracted copy of its own tree (its own Makefile, its own
# synth scripts) while the working tree sweeps in place, dirty or not. Both write into
# one BASELINE_OUT, and soc/baseline_sweep.sh's own resume logic means a rerun after an
# interruption skips every seed it already placed rather than starting the sweep over.
#
# Two refusals neither half nor --allow-mismatch can get past: fewer than MIN_SEEDS
# seeds on a side, and two halves whose toolchain lines disagree (soc/baseline_summary.py's
# own default, since --allow-mismatch is never passed here).
set -eu

cd "$(dirname "$0")/.."

MIN_SEEDS=12

usage() {
  echo "usage: soc/paired_sweep.sh <base-ref> [part...]" >&2
  echo "*** parts default to 'up5k ecp5'. Each part sweeps at its own seed" >&2
  echo "*** count (PAIRED_SEEDS_UP5K / PAIRED_SEEDS_ECP5 to override) and" >&2
  echo "*** refuses below $MIN_SEEDS." >&2
}

if [ "$#" -lt 1 ]; then
  usage
  exit 2
fi

base_ref=$1
shift
parts=${*:-"up5k ecp5"}

git rev-parse --verify --quiet "$base_ref^{commit}" > /dev/null || {
  echo "*** soc/paired_sweep.sh: '$base_ref' does not name a commit here, so" >&2
  echo "*** there is nothing to sweep it against." >&2
  exit 2
}
base_sha=$(git rev-parse "$base_ref")

out=${PAIRED_OUT:-paired.out}
mkdir -p "$out"
out=$(cd "$out" && pwd)

seeds_up5k=${PAIRED_SEEDS_UP5K:-"default 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15"}
seeds_ecp5=${PAIRED_SEEDS_ECP5:-"default 1 2 3 4 5 6 7 8 9 10 11"}

count() {
  # shellcheck disable=SC2086
  set -- $1
  echo "$#"
}

for part in $parts; do
  case $part in
    up5k) n=$(count "$seeds_up5k") ;;
    ecp5) n=$(count "$seeds_ecp5") ;;
    *)
      echo "*** soc/paired_sweep.sh: '$part' is not a part this repo places." >&2
      echo "*** Known: up5k, ecp5." >&2
      exit 2
      ;;
  esac
  if [ "$n" -lt "$MIN_SEEDS" ]; then
    echo "*** soc/paired_sweep.sh: $part names $n seeds, and a go/no-go is" >&2
    echo "*** never taken under $MIN_SEEDS -- see CLAUDE.md's 'Measurements and" >&2
    echo "*** ratchets'. Add seeds; nothing here lowers the floor." >&2
    exit 1
  fi
done

# The base ref's own tree, extracted once per SHA and reused across parts and
# reruns -- soc/netlist_base.sh's own pattern, so a base ref sweeps with its own
# Makefile and synth scripts rather than this tree's.
base_tree="$out/base-tree"
base_marker="$out/base-tree.sha"
if [ ! -f "$base_marker" ] || [ "$(cat "$base_marker")" != "$base_sha" ]; then
  echo "soc/paired_sweep.sh: extracting $base_ref ($base_sha) into $base_tree"
  rm -rf "$base_tree"
  mkdir -p "$base_tree"
  git archive --format=tar "$base_ref" | tar -x -C "$base_tree"
  echo "$base_sha" > "$base_marker"
else
  echo "soc/paired_sweep.sh: reusing the extracted tree for $base_sha"
fi

sweep() {  # <role> <part> <seeds> <treedir>
  role=$1 part=$2 seeds=$3 treedir=$4
  echo
  echo "== $role/$part: sweeping $(count "$seeds") seeds in $treedir =="
  ( cd "$treedir" && BASELINE_PART="$part" SOC_SEEDS="$seeds" \
      BASELINE_OUT="$out" BASELINE_NAME="$role-$part" sh soc/baseline_sweep.sh )
}

digest() {  # <part>
  part=$1
  if [ "$part" != up5k ]; then
    echo "soc/paired_sweep.sh: $part has no netlist-digest instrument (the" \
         "Makefile's NETLIST_PART table names only up5k) -- nothing recorded."
    return
  fi
  log="$out/netlist-diff.$part.log"
  echo "soc/paired_sweep.sh: netlist digest, $base_ref against the working tree"
  if make netlist-diff "BASE=$base_ref" > "$log" 2>&1; then
    tail -8 "$log"
  else
    tail -20 "$log"
    echo "*** soc/paired_sweep.sh: netlist-diff did not run cleanly; see $log." >&2
  fi
  echo "  (full log: $log)"
}

status=0
for part in $parts; do
  case $part in
    up5k) seeds=$seeds_up5k ;;
    ecp5) seeds=$seeds_ecp5 ;;
  esac
  sweep base "$part" "$seeds" "$base_tree"
  sweep candidate "$part" "$seeds" "$PWD"
  digest "$part"
  echo
  echo "== $part: verdict =="
  if ! python3 soc/baseline_summary.py \
        "$out/base-$part.csv" "$out/candidate-$part.csv" --min-seeds "$MIN_SEEDS"; then
    status=1
  fi
done

exit $status

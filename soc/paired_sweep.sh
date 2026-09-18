#!/bin/sh
# ONE COMMAND: a named base ref against the working tree, both parts, paired by
# seed, reusing soc/baseline_sweep.sh for each half.
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

# Extracted OUTSIDE this repo, like the toolchain cache: nested inside it, a
# plain `git rev-parse` there would silently answer for THIS repo instead.
cache=${XDG_CACHE_HOME:-$HOME/.cache}/little-cpu/paired-sweep
base_tree="$cache/$base_sha"
if [ ! -d "$base_tree" ]; then
  echo "soc/paired_sweep.sh: extracting $base_ref ($base_sha) into $base_tree"
  mkdir -p "$base_tree"
  git archive --format=tar "$base_ref" | tar -x -C "$base_tree"
else
  echo "soc/paired_sweep.sh: reusing the extracted tree for $base_sha"
fi

sweep() {  # <role> <part> <seeds> <treedir> <log> [base-sha]
  role=$1 part=$2 seeds=$3 treedir=$4 log=$5
  {
    echo "== $role/$part: sweeping $(count "$seeds") seeds in $treedir =="
    cd "$treedir" && BASELINE_PART="$part" SOC_SEEDS="$seeds" \
      BASELINE_OUT="$out" BASELINE_NAME="$role-$part" \
      BASELINE_BASE_OVERRIDE="${6:-}" BASELINE_DIRTY_OVERRIDE="${6:+no}" \
      sh soc/baseline_sweep.sh
  } > "$log" 2>&1
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

# Base and candidate write into two different directories, so running them as a
# pair rather than sequentially roughly halves this script's own wall time.
status=0
for part in $parts; do
  case $part in
    up5k) seeds=$seeds_up5k ;;
    ecp5) seeds=$seeds_ecp5 ;;
  esac
  base_log="$out/base-$part.sweep.log"
  cand_log="$out/candidate-$part.sweep.log"
  sweep base "$part" "$seeds" "$base_tree" "$base_log" "$base_sha" & base_pid=$!
  sweep candidate "$part" "$seeds" "$PWD" "$cand_log" & cand_pid=$!
  base_ok=0
  cand_ok=0
  wait "$base_pid" || base_ok=1
  wait "$cand_pid" || cand_ok=1
  echo
  cat "$base_log"
  echo
  cat "$cand_log"
  if [ "$base_ok" != 0 ] || [ "$cand_ok" != 0 ]; then
    echo "*** soc/paired_sweep.sh: $part's base or candidate sweep failed; see" >&2
    echo "*** the logs above." >&2
    status=1
    continue
  fi
  digest "$part"
  echo
  echo "== $part: verdict =="
  if ! python3 soc/baseline_summary.py \
        "$out/base-$part.csv" "$out/candidate-$part.csv" --min-seeds "$MIN_SEEDS"; then
    status=1
  fi
done

exit $status

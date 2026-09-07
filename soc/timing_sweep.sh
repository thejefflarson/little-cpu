#!/bin/sh
# One placement is a sample, not a verdict: twelve to sixteen seeds are the go/no-go.
set -eu

cd "$(dirname "$0")/.."

seeds=${SOC_SEEDS:-"default 1 2 3"}
rows=""

for seed in $seeds; do
  case $seed in
    default) arg="" ;;
    *)       arg=$seed ;;
  esac
  if ! out=$(make soc-timing SOC_SEED="$arg" "$@" 2>&1); then
    printf '%s\n' "$out" >&2
    echo "*** soc/timing_sweep.sh: seed '$seed' failed; the sweep stops here." >&2
    exit 1
  fi
  line=$(printf '%s\n' "$out" | grep '^critical path :') || {
    echo "*** soc/timing_sweep.sh: seed '$seed' exited 0 with no critical path" >&2
    echo "*** line, which soc/timing_split.py is supposed to make impossible." >&2
    exit 1
  }
  ns=$(printf '%s\n' "$line" | sed 's/^critical path : \([0-9.]*\) ns.*/\1/')
  mhz=$(printf '%s\n' "$line" | sed 's/.*(\([0-9.]*\) MHz).*/\1/')
  printf 'seed %-8s %8s ns  %6s MHz\n' "$seed" "$ns" "$mhz"
  rows="$rows$ns "
done

echo
echo "sorted: $(printf '%s\n' $rows | sort -n | tr '\n' ' ')"
echo "Compare distributions against a baseline sweep, not single runs."
python3 soc/bands.py up5k --note

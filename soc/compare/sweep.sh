#!/bin/sh
# Run `make compare-timing` for both cores at several placements each and print
# the two distributions.
#
#   soc/compare/sweep.sh                        # both cores, default + seeds 1-3
#   COMPARE_SEEDS='default 1 2 3 4 5' soc/compare/sweep.sh
#   COMPARE_CORES=vexriscv soc/compare/sweep.sh # one side only
#
# One placement is a sample. `make soc-timing`'s spread on the up5k is 1-2% and
# its edit churn 3.6%; nothing says the hx8k is tighter, so a claim about which
# of two cores is faster needs a distribution on both sides and is read on the
# worst placement of each.
#
# `make` is deliberately NOT in a pipeline, for the reason soc/timing_sweep.sh
# records: the default shell is errexit without pipefail, so a graded command
# piped into `grep` reports grep's status and a failed placement prints its row
# and exits 0. The status handed back here is make's.
set -eu

cd "$(dirname "$0")/../.."

seeds=${COMPARE_SEEDS:-"default 1 2 3"}
cores=${COMPARE_CORES:-"littlecpu vexriscv"}

for core in $cores; do
  echo "== $core"
  rows=""
  for seed in $seeds; do
    case $seed in
      default) arg="" ;;
      *)       arg=$seed ;;
    esac
    if ! out=$(make compare-timing COMPARE_CORE="$core" COMPARE_SEED="$arg" "$@" 2>&1); then
      printf '%s\n' "$out" >&2
      echo "*** soc/compare/sweep.sh: $core seed '$seed' failed; the sweep stops here." >&2
      exit 1
    fi
    line=$(printf '%s\n' "$out" | grep '^critical path :') || {
      echo "*** soc/compare/sweep.sh: $core seed '$seed' exited 0 with no critical" >&2
      echo "*** path line, which soc/timing_split.py is supposed to make impossible." >&2
      exit 1
    }
    levels=$(printf '%s\n' "$out" | grep '^  logic levels:' | sed 's/^  logic levels: //')
    lc=$(printf '%s\n' "$out" | grep 'placed ICESTORM_LC against')
    ns=$(printf '%s\n' "$line" | sed 's/^critical path : \([0-9.]*\) ns.*/\1/')
    mhz=$(printf '%s\n' "$line" | sed 's/.*(\([0-9.]*\) MHz).*/\1/')
    printf '  seed %-8s %8s ns  %6s MHz   %s\n' "$seed" "$ns" "$mhz" "$levels"
    printf '           %s\n' "$lc"
    rows="$rows$ns "
  done
  echo "  sorted: $(printf '%s\n' $rows | sort -n | tr '\n' ' ')"
  echo
done

echo "Read the WORST placement of each. Neither core's number here is its own"
echo "project's published figure, and the two ISAs are not the same -- ADR-0086."

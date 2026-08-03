#!/bin/sh
# Run `make soc-timing` at several placements of the same netlist and print the
# spread.
#
# One placement is a sample, not a measurement. Unmodified `main` spans about
# 1.2% across four seeds (ADR-0058) and an edit that changes no hardware moves
# the number by up to 3.6% (ADR-0054), so a comparison between two designs needs
# distributions on both sides. Three ADRs have needed exactly this and each ran
# it by hand.
#
#   soc/timing_sweep.sh                     # the default placement plus seeds 1-3
#   SOC_SEEDS='default 1 2 3 4 5' soc/timing_sweep.sh
#   soc/timing_sweep.sh SOC_PROG=lw.S       # arguments go through to make
#
# Nothing here grades anything. `make soc-timing` carries the SOC_MIN_MHZ ratchet
# and this script hands its exit status straight back, so a placement that fails
# stops the sweep with that target's own diagnostic rather than leaving a blank
# row in the table.
#
# `make` is deliberately NOT in a pipeline. The first version of this ran
# `make soc-timing | grep 'critical path'`, whose status is grep's -- so a
# placement under SOC_MIN_MHZ printed its row and the sweep exited 0. Same shape
# as the graded comparison piped into `tee` that hid a red formal gate for
# months (ADR-0037).
#
# It also calls the whole target per seed rather than re-running nextpnr with a
# new seed against one `soc.json`. That would skip four resyntheses and cost a
# second copy of the placement recipe -- the nextpnr invocation and its two
# guards -- which would then drift from the Makefile's. The report is parsed by
# soc/timing_split.py alone; what this reads is that script's printed summary.
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

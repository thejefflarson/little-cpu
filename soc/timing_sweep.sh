#!/bin/sh
# Run `make soc-timing` at several placements of the same netlist and print the
# spread.
#
# One placement is a sample, not a measurement: an unmodified netlist spans a
# wide range across seeds, and an edit that changes no hardware moves the number
# too, so a comparison between two designs needs distributions on both sides.
# BOTH FIGURES ARE soc/bands.py's AND THIS SCRIPT CARRIES NEITHER -- it prints
# them at the end from there. A band written into a comment is a band that stops
# being true silently: the copy that used to be here was four times too narrow,
# and so were the five others, for as long as it took a sixteen-seed sweep to
# measure it.
#
# FOUR SEEDS IS A LOOK AND NOT A VERDICT, which is why the default below is not
# the sweep a decision is made on. The spread is a range statistic: a short sweep
# does not sample a tighter distribution, it takes a shorter look at the same
# one, and this repo has twice had four seeds say something eight did not. A
# go/no-go is twelve to sixteen placements paired by seed --
# soc/baseline_sweep.sh, which keeps every report and stamps the tree and the
# toolchain that produced them.
#
#   soc/timing_sweep.sh                     # the default placement plus seeds 1-3
#   SOC_SEEDS='default 1 2 3 4 5' soc/timing_sweep.sh
#   soc/timing_sweep.sh SOC_PROG=lw.S       # arguments go through to make
#
# A sweep is only owed when the netlist moved. `make netlist-diff BASE=<ref>`
# says whether it did: digest unchanged, no sweep owed; digest changed, the
# paired sixteen-seed sweep in soc/baseline_sweep.sh is what is owed, not this
# four-seed look.
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
# months.
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
# Printed rather than commented, so the figures a reader judges these rows
# against are the ones that were last measured rather than the ones that were
# last typed. This target only ever places up5k.
python3 soc/bands.py up5k --note

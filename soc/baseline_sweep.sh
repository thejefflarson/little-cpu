#!/bin/sh
# Places the SoC at many seeds ON EITHER PART, KEEPS every seed's report, and
# stamps the whole sweep with the tree, the part and the toolchain that measured
# it.
#
# `make soc-timing` writes one `soc.timing.rpt` and overwrites it on the next
# seed, so a sweep survived as a column of frequencies with nothing attached: no
# base commit, no tool versions, no placed cell count, and no way to re-read a
# placement afterwards. Comparability was then a convention followed by hand, and
# it has failed twice here -- four seeds reading +4.35% where eight read inside
# the band, and two toolchains disagreeing about the SIGN of a period change on
# identical RTL. The provenance block ahead of the rows is what lets a later
# reader tell those apart, and soc/baseline_summary.py REFUSES to subtract two
# sweeps whose blocks disagree rather than warning about it.
#
#   soc/baseline_sweep.sh                          # 16 up5k seeds into baseline.out/
#   BASELINE_PART=ecp5 soc/baseline_sweep.sh       # the same sweep on the other part
#   SOC_SEEDS='default 1 2' soc/baseline_sweep.sh  # a shorter run
#   BASELINE_NAME=before soc/baseline_sweep.sh SOC_PROG=lw.S
#
# Sixteen placements of the shipping SoC is about 12 minutes of compute on up5k
# and about 9 on ECP5.
#
# ONE SCRIPT, TWO PARTS, AND THE PART IS STAMPED. A second copy of this recipe
# for the other part would be a fifth placement loop in this repo, and drift
# between two of the existing four was already worth four "placements" that were
# one placement. What is per-part is a table at the top -- which make target
# places, which variable carries the seed, which artifacts a finished placement
# leaves, and which tools the number is a property of -- and nothing below it
# names a part.
#
# The tool set is a function of the part because the instruments are different
# CLASSES, not two settings of one: up5k is placed by nextpnr-ice40 and then READ
# BACK by icetime, and ECP5 has no icetime at all, so nextpnr-ecp5's own
# estimator is both the placer and the grader. A sweep of one part stamped with
# the other's tools is a file whose header describes an experiment nobody ran,
# and soc/baseline_summary.py rejects it on exactly that.
#
# ASK WHETHER THEY ARE OWED BEFORE SPENDING THEM. `make netlist-diff BASE=<ref>`
# digests the mapped netlist on both sides of a change, and the landing
# procedure is: DIGEST UNCHANGED, NO SWEEP OWED -- the placer's input differs
# only in dead nets and source attributes, which is measured not to move the
# placement; DIGEST CHANGED, the paired sixteen-seed sweep is owed and this is
# it. Sound in that one direction only: a digest that moved says nothing about
# the period, and at today's margin it is a stop-and-redesign signal rather than
# merely a sweep owed, because there is nowhere to put the cells.
#
# Nothing here grades anything and nothing here is a gate. `make soc-timing`
# carries the SOC_MIN_MHZ requirement and this script hands its status straight
# back, so a placement under the board clock stops the sweep with that target's
# own diagnostic instead of leaving a row that means nothing. `make ecp5-timing`
# carries no frequency requirement at all -- what it gates is the mapping -- and
# this script does not invent one.
#
# Two rules are soc/timing_sweep.sh's and are kept for its reasons. `make` is NOT
# in a pipeline: a graded command in one exits with the pipeline's last status,
# which is how a red gate here once printed a row and returned 0. And the whole
# target is called per seed rather than nextpnr re-run against one `soc.json`:
# that would save four minutes and cost a second copy of the placement recipe,
# which drifts from the Makefile's -- already worth four "placements" that were
# one placement.
#
# The rows are soc/depth/row.py's, so every report is walked by the one reader
# its part has and by nothing else.
set -eu

cd "$(dirname "$0")/.."

part=${BASELINE_PART:-up5k}
case $part in
  up5k)
    toolchain_target=soc-timing-toolchain
    place_target=soc-timing
    seed_var=SOC_SEED
    # `soc.timing.rpt` is icetime's; the other two are nextpnr's. All three are
    # kept because the row carries only a summary of the first.
    artifacts='soc.timing.rpt soc.asc soc.pnr.log'
    ;;
  ecp5)
    toolchain_target=ecp5-timing-toolchain
    place_target=ecp5-timing
    seed_var=ECP5_SEED
    # The report and the configuration are BOTH the measurement: the frequency
    # comes out of the first and the corner is graded off the second, so a kept
    # report with no configuration beside it could not be re-read.
    artifacts='ecp5.report.json ecp5.config ecp5.pnr.log'
    ;;
  *)
    echo "*** soc/baseline_sweep.sh: BASELINE_PART is '$part', which is not a" >&2
    echo "*** part this repo places. Name up5k or ecp5." >&2
    exit 2
    ;;
esac

# `-` rather than `:-`: an explicitly empty SOC_SEEDS is a mistake, and placing
# the default sixteen instead of the nothing that was asked for would hide it.
seeds=${SOC_SEEDS-"default 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15"}
if [ -z "$seeds" ]; then
  echo "*** soc/baseline_sweep.sh: SOC_SEEDS is empty, so nothing would be" >&2
  echo "*** placed. Name the seeds, or unset it for the default sixteen." >&2
  exit 2
fi

out=${BASELINE_OUT:-baseline.out}
name=${BASELINE_NAME:-baseline}
csv="$out/$name.csv"

# A tool with no resolved path and no version is a sweep nobody can reproduce,
# so soc/print_toolchain.sh stamps both for each and stops the run when one
# cannot be asked. Asked through the part's own `*-toolchain` target because that
# is the target the CI job for this part stamps itself with, so a sweep's
# provenance and CI's cannot become two answers to one question.
tools=$(make -s "$toolchain_target" "$@")

# Asked of make rather than repeated here: a second copy of either default would
# stamp the sweep with a program the build did not use the moment one moves, and
# these arguments are the ones the placements below are given.
prog=$(make -s print-SOC_PROG "$@")
rom_words=$(make -s print-SOC_ROM_WORDS "$@")

# The corner and the constraint are inputs to an ECP5 measurement rather than
# descriptions of it -- nextpnr stops working a path once the constraint is met,
# so two constraints are two experiments -- and they are what soc/depth/row.py
# grades the report against. up5k has neither: its corner is in the target's own
# nextpnr and icetime flags, and it is placed against nextpnr's default.
corner=
constraint=
clock=
if [ "$part" = ecp5 ]; then
  corner=$(make -s print-ECP5_PART "$@")
  constraint=$(make -s print-ECP5_TARGET_MHZ "$@")
  clock=$(make -s print-ECP5_CLOCK "$@")
fi

base=$(git rev-parse HEAD)
# Tracked modifications only, staged or not: an untracked file is not in any
# source list the build reads, and calling the tree dirty for one would make the
# flag useless exactly when it matters.
if git diff --quiet HEAD --; then dirty=no; else dirty=yes; fi

mkdir -p "$out"

block=$(
  echo "# baseline-sweep v1"
  echo "# date: $(date -u '+%Y-%m-%dT%H:%M:%SZ')"
  echo "# base: $base"
  echo "# dirty: $dirty"
  echo "# part: $part"
  printf '%s\n' "$tools"
  if [ "$part" = ecp5 ]; then
    echo "# corner: $corner"
    echo "# constraint_mhz: $constraint"
  fi
  echo "# prog: $prog"
  echo "# rom_words: $rom_words"
  echo "# seeds: $seeds"
  echo "# host: $(uname -s) $(uname -m) $(uname -r)"
  # The program travels in the environment rather than in the arguments because
  # it reaches make either way, and a run that took it from the environment
  # would otherwise reproduce as whatever the default had become by then. The
  # part travels the same way and for the same reason.
  echo "# reproduce: git checkout $base && BASELINE_PART=$part SOC_SEEDS='$seeds'" \
       "SOC_PROG=$prog soc/baseline_sweep.sh${*:+ $*}"
  echo "# end-provenance"
)

printf '%s\n' "$block" > "$csv"
python3 soc/depth/row.py --header >> "$csv"
printf '%s\n' "$block"

for seed in $seeds; do
  case $seed in
    default) arg="" ;;
    *)       arg=$seed ;;
  esac
  if ! log=$(make "$place_target" "$seed_var=$arg" "$@" 2>&1); then
    printf '%s\n' "$log" >&2
    echo "*** soc/baseline_sweep.sh: seed '$seed' failed; the sweep stops here." >&2
    exit 1
  fi
  for artifact in $artifacts; do
    if [ ! -s "$artifact" ]; then
      echo "*** soc/baseline_sweep.sh: seed '$seed' left no $artifact behind." >&2
      echo "*** That is a failed measurement, not a fast design." >&2
      exit 1
    fi
  done
  # One set per seed, so a placement can be re-read months later against the row
  # it produced. The fixed names the place targets write are the reason this copy
  # exists: the next seed overwrites all of them. The suffix is everything past
  # the first dot, so `soc.timing.rpt` is kept as `<name>.<seed>.timing.rpt` --
  # soc/routing_bins.py reads that name back for the up5k sweep.
  for artifact in $artifacts; do
    cp "$artifact" "$out/$name.$seed.${artifact#*.}"
  done

  case $part in
    up5k)
      # The PACKED cell count out of the placement, not `SB_LUT4` out of
      # synthesis. The two disagree in magnitude and in sign -- a LUT the flops
      # beside it were sharing a cell with is not a LUT anyone can spend -- so a
      # row that carried the other unit would be graded against the wrong band.
      lc=$(sed -n 's/.*ICESTORM_LC: *\([0-9]*\)\/.*/\1/p' "$out/$name.$seed.pnr.log" | tail -1)
      if [ -z "$lc" ]; then
        echo "*** soc/baseline_sweep.sh: seed '$seed' placed with no ICESTORM_LC in" >&2
        echo "*** its log, so the row would carry no cell count." >&2
        exit 1
      fi
      # The variant column carries the sweep's name, so rows from two sweeps
      # concatenated into one file still say which side each came from.
      row=$(python3 soc/depth/row.py "$out/$name.$seed.timing.rpt" up5k "$name" "$seed" "$lc")
      ;;
    ecp5)
      # No cell count is passed in: nextpnr's own utilisation table is in the
      # report, which soc/depth/row.py is already reading, so a second count off
      # the log would be a second thing that can disagree with it.
      row=$(python3 soc/depth/row.py --ecp5 \
              "$out/$name.$seed.report.json" "$out/$name.$seed.config" \
              "$corner" "$clock" "$constraint" "$name" "$seed")
      ;;
  esac
  printf '%s\n' "$row" >> "$csv"
  printf '%s\n' "$row"
done

echo
echo "$csv"
echo "Read it with: python3 soc/baseline_summary.py $csv"

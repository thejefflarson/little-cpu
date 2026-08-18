#!/bin/sh
# Places the SoC at many seeds, KEEPS every seed's report, and stamps the whole
# sweep with the tree and the toolchain that measured it.
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
#   soc/baseline_sweep.sh                          # 16 seeds into baseline.out/
#   SOC_SEEDS='default 1 2' soc/baseline_sweep.sh  # a shorter run
#   BASELINE_NAME=before soc/baseline_sweep.sh SOC_PROG=lw.S
#
# Sixteen placements of the shipping SoC is about 12 minutes of compute.
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
# own diagnostic instead of leaving a row that means nothing.
#
# Two rules are soc/timing_sweep.sh's and are kept for its reasons. `make` is NOT
# in a pipeline: a graded command in one exits with the pipeline's last status,
# which is how a red gate here once printed a row and returned 0. And the whole
# target is called per seed rather than nextpnr re-run against one `soc.json`:
# that would save four minutes and cost a second copy of the placement recipe,
# which drifts from the Makefile's -- already worth four "placements" that were
# one placement.
#
# The rows are soc/depth/row.py's, so every report is walked by
# soc/timing_split.py's single walk and by nothing else.
set -eu

cd "$(dirname "$0")/.."

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
# cannot be asked. Asked through `make soc-timing-toolchain` because that is the
# target the CI job grading SOC_MIN_MHZ stamps itself with, so a sweep's
# provenance and CI's cannot become two answers to one question.
tools=$(make -s soc-timing-toolchain "$@")

# Asked of make rather than repeated here: a second copy of either default would
# stamp the sweep with a program the build did not use the moment one moves, and
# these arguments are the ones the placements below are given.
prog=$(make -s print-SOC_PROG "$@")
rom_words=$(make -s print-SOC_ROM_WORDS "$@")

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
  printf '%s\n' "$tools"
  echo "# prog: $prog"
  echo "# rom_words: $rom_words"
  echo "# seeds: $seeds"
  echo "# host: $(uname -s) $(uname -m) $(uname -r)"
  # The program travels in the environment rather than in the arguments because
  # it reaches make either way, and a run that took it from the environment
  # would otherwise reproduce as whatever the default had become by then.
  echo "# reproduce: git checkout $base && SOC_SEEDS='$seeds' SOC_PROG=$prog" \
       "soc/baseline_sweep.sh${*:+ $*}"
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
  if ! log=$(make soc-timing SOC_SEED="$arg" "$@" 2>&1); then
    printf '%s\n' "$log" >&2
    echo "*** soc/baseline_sweep.sh: seed '$seed' failed; the sweep stops here." >&2
    exit 1
  fi
  for artifact in soc.timing.rpt soc.asc soc.pnr.log; do
    if [ ! -s "$artifact" ]; then
      echo "*** soc/baseline_sweep.sh: seed '$seed' left no $artifact behind." >&2
      echo "*** That is a failed measurement, not a fast design." >&2
      exit 1
    fi
  done
  # One set per seed, so a placement can be re-read months later against the row
  # it produced. The fixed names `make soc-timing` writes are the reason this
  # copy exists: the next seed overwrites all three.
  cp soc.timing.rpt "$out/$name.$seed.rpt"
  cp soc.asc "$out/$name.$seed.asc"
  cp soc.pnr.log "$out/$name.$seed.pnr.log"
  # The PACKED cell count out of the placement, not `SB_LUT4` out of synthesis.
  # The two disagree in magnitude and in sign -- a LUT the flops beside it were
  # sharing a cell with is not a LUT anyone can spend -- so a row that carried
  # the other unit would be graded against the wrong band.
  lc=$(sed -n 's/.*ICESTORM_LC: *\([0-9]*\)\/.*/\1/p' "$out/$name.$seed.pnr.log" | tail -1)
  if [ -z "$lc" ]; then
    echo "*** soc/baseline_sweep.sh: seed '$seed' placed with no ICESTORM_LC in" >&2
    echo "*** its log, so the row would carry no cell count." >&2
    exit 1
  fi
  # The variant column carries the sweep's name, so rows from two sweeps
  # concatenated into one file still say which side each came from.
  row=$(python3 soc/depth/row.py "$out/$name.$seed.rpt" up5k "$name" "$seed" "$lc")
  printf '%s\n' "$row" >> "$csv"
  printf '%s\n' "$row"
done

echo
echo "$csv"
echo "Read it with: python3 soc/baseline_summary.py $csv"

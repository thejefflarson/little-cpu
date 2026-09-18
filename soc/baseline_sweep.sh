#!/bin/sh
# Places the SoC at many seeds ON EITHER PART, KEEPS every seed's report, and stamps the
# whole sweep with the tree, the part and the toolchain that measured it.
#
# RESUMES RATHER THAN RESTARTS. A sixteen-seed sweep is tens of minutes, so a CSV
# already stamped with this run's own base, dirty flag and part is read rather than
# truncated, and a seed already carrying both its artifact and its row is skipped. A
# stamp that disagrees on any of those three fields is a different sweep and starts
# fresh, same as an empty or absent file.
set -eu

cd "$(dirname "$0")/.."

part=${BASELINE_PART:-up5k}
case $part in
  up5k)
    toolchain_target=soc-timing-toolchain
    place_target=soc-timing
    seed_var=SOC_SEED
    # `soc.timing.rpt` is icetime's; the other two are nextpnr's.
    artifacts='soc.timing.rpt soc.asc soc.pnr.log'
    ;;
  ecp5)
    toolchain_target=ecp5-timing-toolchain
    place_target=ecp5-timing
    seed_var=ECP5_SEED
    artifacts='ecp5.report.json ecp5.config ecp5.pnr.log'
    ;;
  *)
    echo "*** soc/baseline_sweep.sh: BASELINE_PART is '$part', which is not a" >&2
    echo "*** part this repo places. Name up5k or ecp5." >&2
    exit 2
    ;;
esac

# `-` rather than `:-`: an explicitly empty SOC_SEEDS is a mistake, and placing the
# default sixteen instead of the nothing that was asked for would hide it.
seeds=${SOC_SEEDS-"default 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15"}
if [ -z "$seeds" ]; then
  echo "*** soc/baseline_sweep.sh: SOC_SEEDS is empty, so nothing would be" >&2
  echo "*** placed. Name the seeds, or unset it for the default sixteen." >&2
  exit 2
fi

out=${BASELINE_OUT:-baseline.out}
name=${BASELINE_NAME:-baseline}
csv="$out/$name.csv"

tools=$(make -s "$toolchain_target" "$@")

prog=$(make -s print-SOC_PROG "$@")
rom_words=$(make -s print-SOC_ROM_WORDS "$@")

corner=
constraint=
clock=
if [ "$part" = ecp5 ]; then
  corner=$(make -s print-ECP5_PART "$@")
  constraint=$(make -s print-ECP5_TARGET_MHZ "$@")
  clock=$(make -s print-ECP5_CLOCK "$@")
fi

base=$(git rev-parse HEAD)
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
  echo "# reproduce: git checkout $base && BASELINE_PART=$part SOC_SEEDS='$seeds'" \
       "SOC_PROG=$prog soc/baseline_sweep.sh${*:+ $*}"
  echo "# end-provenance"
)

resume=0
if [ -s "$csv" ] && [ "$(sed -n '1p' "$csv")" = "# baseline-sweep v1" ]; then
  old_base=$(sed -n 's/^# base: //p' "$csv" | head -1)
  old_dirty=$(sed -n 's/^# dirty: //p' "$csv" | head -1)
  old_part=$(sed -n 's/^# part: //p' "$csv" | head -1)
  if [ "$old_base" = "$base" ] && [ "$old_dirty" = "$dirty" ] && [ "$old_part" = "$part" ]; then
    resume=1
  fi
fi

if [ "$resume" = 1 ]; then
  echo "soc/baseline_sweep.sh: resuming $csv -- same base, dirty flag and part"
else
  printf '%s\n' "$block" > "$csv"
  python3 soc/depth/row.py --header >> "$csv"
fi
printf '%s\n' "$block"

# The first artifact in the part's list, stripped of its `soc.`/`ecp5.` prefix, is
# what a resumed seed is checked against: it exists only once a placement finished.
first_suffix=$(set -- $artifacts; first=$1; echo "${first#*.}")

for seed in $seeds; do
  if [ "$resume" = 1 ] && [ -s "$out/$name.$seed.$first_suffix" ] && \
     awk -F, -v s="$seed" '$3 == s { found = 1 } END { exit !found }' "$csv"; then
    echo "soc/baseline_sweep.sh: seed '$seed' already placed, skipping"
    continue
  fi
  case $seed in
    default) arg="" ;;
    *)       arg=$seed ;;
  esac
  # up5k's own recipe writes soc.timing.rpt and THEN applies SOC_MIN_MHZ, so a seed
  # under the floor is a real placement with a nonzero exit, not a build failure. The
  # spread this sweep exists to measure is exactly the distribution a ratchet's own
  # worst-of-N would trip on, so that exit is read past rather than treated as fatal --
  # only artifacts missing outright (a genuine build or tool failure) stop the sweep.
  if log=$(make "$place_target" "$seed_var=$arg" "$@" 2>&1); then
    make_status=0
  else
    make_status=$?
  fi
  missing=0
  for artifact in $artifacts; do
    [ -s "$artifact" ] || missing=1
  done
  if [ "$missing" = 1 ]; then
    printf '%s\n' "$log" >&2
    echo "*** soc/baseline_sweep.sh: seed '$seed' left no artifacts behind;" >&2
    echo "*** that is a failed measurement, not a fast design. The sweep stops here." >&2
    exit 1
  fi
  if [ "$make_status" != 0 ]; then
    echo "soc/baseline_sweep.sh: seed '$seed' exited $make_status (a ratchet or" \
         "requirement it missed) but left every artifact behind -- recorded as a" \
         "real placement, not a sweep failure."
  fi
  for artifact in $artifacts; do
    cp "$artifact" "$out/$name.$seed.${artifact#*.}"
  done

  case $part in
    up5k)
      lc=$(sed -n 's/.*ICESTORM_LC: *\([0-9]*\)\/.*/\1/p' "$out/$name.$seed.pnr.log" | tail -1)
      if [ -z "$lc" ]; then
        echo "*** soc/baseline_sweep.sh: seed '$seed' placed with no ICESTORM_LC in" >&2
        echo "*** its log, so the row would carry no cell count." >&2
        exit 1
      fi
      row=$(python3 soc/depth/row.py "$out/$name.$seed.timing.rpt" up5k "$name" "$seed" "$lc")
      ;;
    ecp5)
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

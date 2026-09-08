#!/bin/sh
# Search SOC_SEED (soc/seed_hash.py's sha256 draw by default, never 1..N: small
# integers span a low-dimensional subspace of nextpnr's xorshift state)
# for a placement that clears SOC_MIN_MHZ with margin, then write soc/pin.json.
set -eu

cd "$(dirname "$0")/.."

MIN_MARGIN_PCT=5.0

if [ -n "${SOC_SEARCH_SEEDS-}" ]; then
  seeds=$SOC_SEARCH_SEEDS
  seeds_source="explicit SOC_SEARCH_SEEDS"
else
  count=${SOC_SEARCH_COUNT:-12}
  seeds=$(python3 soc/seed_hash.py "$count")
  seeds_source="sha256(little-cpu-soc-seed-<i>) default, count=$count"
fi
if [ -z "$seeds" ]; then
  echo "*** soc/soc_seed_search.sh: no seeds to try." >&2
  exit 2
fi

min_mhz=$(make -s print-SOC_MIN_MHZ)
tools=$(make -s soc-timing-toolchain)

samples=$(mktemp "${TMPDIR:-/tmp}/soc-seed-search-samples.XXXXXX")
dist=$(mktemp "${TMPDIR:-/tmp}/soc-seed-search-dist.XXXXXX")
trap 'rm -f "$samples" "$dist"' EXIT

best_seed=
best_mhz=0
for seed in $seeds; do
  out=$(make soc-timing SOC_SEED="$seed" SOC_MIN_MHZ=0 2>&1) || {
    printf '%s\n' "$out" >&2
    echo "*** soc/soc_seed_search.sh: seed $seed failed to PLACE (nextpnr produced no" >&2
    echo "*** bitstream, or wrote no utilisation table) -- a toolchain or netlist" >&2
    echo "*** problem, not merely a slow seed, so the search stops here." >&2
    exit 1
  }
  line=$(printf '%s\n' "$out" | grep '^critical path :') || {
    echo "*** soc/soc_seed_search.sh: seed $seed exited 0 with no critical path line," >&2
    echo "*** which soc/timing_split.py is supposed to make impossible." >&2
    exit 1
  }
  mhz=$(printf '%s\n' "$line" | sed 's/.*(\([0-9.]*\) MHz).*/\1/')
  printf 'seed %-12s %6s MHz\n' "$seed" "$mhz"
  echo "$seed $mhz" >> "$samples"
  if awk -v a="$mhz" -v b="$best_mhz" 'BEGIN{exit !(a>b)}'; then
    best_mhz=$mhz
    best_seed=$seed
  fi
done

margin_pct=$(awk -v mhz="$best_mhz" -v floor="$min_mhz" 'BEGIN{printf "%.4f", 100*(mhz-floor)/floor}')
echo
echo "best: seed $best_seed at $best_mhz MHz, ${margin_pct}% over $min_mhz MHz"

if ! awk -v m="$margin_pct" -v need="$MIN_MARGIN_PCT" 'BEGIN{exit !(m>=need)}'; then
  echo "*** soc/soc_seed_search.sh: no seed cleared a ${MIN_MARGIN_PCT}% margin over" >&2
  echo "*** $min_mhz MHz -- the placer-seed dimension is exhausted at this seed" >&2
  echo "*** count. The next lever is synthesis cell-name order (yosys's" >&2
  echo "*** 'rename -scramble-name'), documented in the pinned-placement ADR and not wired into" >&2
  echo "*** this script; verify it is seedable and byte-reproducible before" >&2
  echo "*** relying on it. Nothing was pinned." >&2
  exit 1
fi

make -s soc.canon.json > /dev/null
digest=$(python3 soc/soc_pin.py digest soc.canon.json)

python3 soc/seed_search_distribution.py "$samples" "$best_seed" "$seeds_source" > "$dist"

python3 soc/soc_pin.py write soc/pin.json \
  --digest "$digest" --seed "$best_seed" --mhz "$best_mhz" --min-mhz "$min_mhz" \
  --margin-pct "$margin_pct" --toolchain "$tools" \
  --distribution "$dist" --date "$(date -u '+%Y-%m-%d')"

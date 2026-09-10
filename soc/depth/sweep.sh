#!/usr/bin/env bash
# soc/depth/variants.py's four fetch-loop depths, placed on up5k, one CSV row per seed.
set -euo pipefail

part=${1:-up5k}
shift || true
seeds=("$@")
if [ ${#seeds[@]} -eq 0 ]; then seeds=(0 1 2 3); fi

root=$(cd "$(dirname "$0")/../.." && pwd)
cd "$root"
out=${DEPTH_OUT:-$root/depth.out}
mkdir -p "$out"
mem=$out/imemory_depth.v
python3 soc/depth/variants.py "$mem"

# make owns the source list; the copy this had went stale. The spike memory substitutes
# IN PLACE because read order sets ABC's mapping, and appending would not.
spike_srcs() {
  list=$(make -s "$@")
  case " $list " in
    *" rtl/imemory.v "*) ;;
    *)
      echo "*** soc/depth/sweep.sh: $1 does not name rtl/imemory.v, so the spike" >&2
      echo "*** memory has nothing to take the place of. The variable this reads" >&2
      echo "*** was renamed or restructured; re-find it rather than restating the" >&2
      echo "*** list here." >&2
      exit 2
      ;;
  esac
  printf '%s\n' "$list" | sed "s#rtl/imemory\\.v#$mem#"
}

case "$part" in
  up5k)
    srcs=$(spike_srcs print-SOC_SRCS)
    top=littlesoc
    synth_args="-dsp -spram"
    chp=""
    pcf=soc/littlesoc.pcf
    pnr_args="--up5k --package sg48"
    ice_args="-d up5k -P sg48"
    make -s soc-rom
    ;;
  *) echo "usage: $0 up5k [seed ...]" >&2; exit 2 ;;
esac

python3 soc/depth/row.py --header

for variant in base addr data both; do
  case "$variant" in
    base) ra=0; rd=0 ;;
    addr) ra=1; rd=0 ;;
    data) ra=0; rd=1 ;;
    both) ra=1; rd=1 ;;
  esac
  tag="$part.$variant"
  yosys -p "read_verilog -sv $srcs; \
            chparam -set REG_ADDR $ra -set REG_DATA $rd imemory; $chp \
            synth_ice40 $synth_args -top $top -json $out/$tag.json; stat" \
    > "$out/$tag.synth.log" 2>&1 \
    || { tail -30 "$out/$tag.synth.log"; exit 1; }

  for seed in "${seeds[@]}"; do
    asc="$out/$tag.$seed.asc"
    seed_arg=""
    [ "$seed" = 0 ] || seed_arg="--seed $seed"
    nextpnr-ice40 $pnr_args $seed_arg --json "$out/$tag.json" --pcf "$pcf" \
      --asc "$asc" > "$out/$tag.$seed.pnr.log" 2>&1 || true
    if [ ! -s "$asc" ]; then
      echo "*** $tag seed $seed produced no bitstream: a failed placement, not a slow design" >&2
      tail -20 "$out/$tag.$seed.pnr.log" >&2
      exit 1
    fi
    icetime $ice_args -p "$pcf" -t -r "$out/$tag.$seed.rpt" "$asc" \
      > "$out/$tag.$seed.icetime.log" 2>&1
    lc=$(sed -n 's/.*ICESTORM_LC: *\([0-9]*\)\/.*/\1/p' "$out/$tag.$seed.pnr.log" | tail -1)
    [ -n "$lc" ] || { echo "*** $tag seed $seed: no ICESTORM_LC in the placement log" >&2; exit 1; }
    python3 soc/depth/row.py "$out/$tag.$seed.rpt" "$part" "$variant" "$seed" "$lc"
  done
done

python3 soc/bands.py "$part" --note >&2

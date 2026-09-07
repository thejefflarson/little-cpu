#!/usr/bin/env bash
# Places and times the four fetch-loop depths soc/depth/variants.py writes, on both
# parts, over as many seeds as asked for, and prints one CSV row per placement.
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

CORE_SRCS="rtl/structs.v rtl/accessor.v rtl/csrs.v rtl/decoder.v rtl/executor.v \
rtl/fetcher.v $mem rtl/memory.v rtl/regfile.v rtl/regsel.v"

case "$part" in
  up5k)
    # The shipping SoC, with the spike memory swapped in for rtl/imemory.v.
    srcs="$CORE_SRCS rtl/timer.v rtl/writeback.v rtl/littlecpu.v rtl/littlesoc.v"
    top=littlesoc
    synth_args="-dsp -spram"
    chp=""
    pcf=soc/littlesoc.pcf
    pnr_args="--up5k --package sg48"
    ice_args="-d up5k -P sg48"
    make -s soc-rom
    ;;
  hx8k)
    srcs="$CORE_SRCS rtl/writeback.v rtl/littlecpu.v soc/compare/bench_littlecpu.v"
    top=bench_littlecpu
    synth_args=""
    chp="chparam -set ROM_WORDS 1024 -set RAM_WORDS 512 $top;"
    pcf=soc/compare/bench_hx8k.pcf
    pnr_args="--hx8k --package ct256"
    ice_args="-d hx8k -P ct256"
    make -s compare-rom
    ;;
  *) echo "usage: $0 <up5k|hx8k> [seed ...]" >&2; exit 2 ;;
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

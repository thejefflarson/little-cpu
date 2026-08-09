#!/usr/bin/env bash
# Places and times the four fetch-loop depths soc/depth/variants.py writes, on
# both parts, over as many seeds as asked for, and prints one CSV row per
# placement. soc/depth/summary.py turns the rows into distributions.
#
# A SPIKE. Nothing here is a gate, nothing here grades the shipping design, and
# the memory it measures is functionally wrong on purpose -- see that script's
# header. The output is a distribution to read against the churn bands (~3.6%
# edit churn, 1-2% placement spread), not a single number.
#
# Two parts because neither one alone answers the question. up5k is the board and
# carries `SOC_MIN_MHZ`; hx8k is where soc/compare/ put both cores side by side,
# so a level count here is comparable with VexRiscv's 17. They are different
# designs -- 8 KB of ROM and 64 KB of SPRAM against 4 KB and 2 KB of block RAM,
# and no timer on hx8k -- so their nanoseconds are never merged.
#
# THE `base` VARIANT IS THE CONTROL. It is the generated memory with both
# parameters zero, where each added register has no reader and yosys deletes it,
# so it has to reproduce rtl/imemory.v's own placement and cell count. Measured:
# 4790 logic cells against the shipping SoC's 4769, which is inside the +/-50
# churn band, and the same critical path endpoints. A `base` that had drifted
# would move every delta below with nothing to say so.
#
# Usage: soc/depth/sweep.sh <up5k|hx8k> [seed ...]
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

# The spike memory sits in rtl/imemory.v's place in the list rather than at the
# end of it. yosys names cells in the order it reads them and ABC's mapping
# follows those names, so a reordered source list is an edit-churn-sized move on
# its own -- 3.6% here, measured, which is the whole band.
CORE_SRCS="rtl/structs.v rtl/accessor.v rtl/csrs.v rtl/decoder.v rtl/executor.v \
rtl/fetcher.v $mem rtl/memory.v rtl/regfile.v"

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
    # soc/compare/'s geometry, so the level count is comparable with the
    # VexRiscv row in the same harness.
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
  # Synthesised once per variant: the seed only reaches nextpnr, and re-running
  # yosys per seed would spend minutes producing the same netlist. The ROM is
  # rebuilt above rather than depended on through make, because everything after
  # an order-only `|` in a prerequisite list is order-only and that has already
  # cost this repository four "placements" that were one placement.
  yosys -p "read_verilog -sv $srcs; \
            chparam -set REG_ADDR $ra -set REG_DATA $rd imemory; $chp \
            synth_ice40 $synth_args -top $top -json $out/$tag.json; stat" \
    > "$out/$tag.synth.log" 2>&1 \
    || { tail -30 "$out/$tag.synth.log"; exit 1; }

  for seed in "${seeds[@]}"; do
    asc="$out/$tag.$seed.asc"
    # Seed 0 is nextpnr's default placement with no `--seed` at all, which is
    # the one `make soc-timing` and `make compare-timing` report.
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

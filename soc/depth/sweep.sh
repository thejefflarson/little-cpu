#!/usr/bin/env bash
# Places and times the four fetch-loop depths soc/depth/variants.py writes, on
# both parts, over as many seeds as asked for, and prints one CSV row per
# placement. soc/depth/summary.py turns the rows into distributions.
#
# A SPIKE. Nothing here is a gate, nothing here grades the shipping design, and
# the memory it measures is functionally wrong on purpose -- see that script's
# header. The output is a distribution to read against the part's own bands,
# which are soc/bands.py's and are printed at the end from there rather than
# restated here. One of the two parts below has no band derived at all, and that
# script says so instead of lending it the other's.
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

# THE SOURCE LIST IS THE MAKEFILE'S, ASKED FOR, NOT A SECOND COPY OF IT. This
# script kept its own hand-written list for its whole life, and the list went
# stale the first time a module joined the SoC without anyone thinking of this
# file: rtl/uart.v and rtl/spiflash.v landed in rtl/littlesoc.v and in SOC_SRCS,
# nothing here moved, and yosys then stopped because the design had no spiflash
# in it. Naming the two missing files here would fix that day and re-break on the
# next module. So each part asks make for the list its own shipping flow places,
# the way soc/baseline_sweep.sh already asks for SOC_PROG, and the way
# soc/compare/ reads its geometry from the Makefile rather than from a second
# hand-kept list.
#
# The spike memory is SUBSTITUTED IN PLACE for rtl/imemory.v rather than appended.
# yosys names cells in the order it reads them and ABC's mapping follows those
# names, so a reordered source list moves the number by about as much as the
# whole edit-churn band on its own -- measured, and it is one of the functionally
# identical texts that band is derived from. Substitution keeps the shipping
# order exactly; appending would not.
spike_srcs() {
  # "$@" so a part can pass the variable assignments its list is a function of.
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
    # The shipping SoC, with the spike memory swapped in for rtl/imemory.v.
    srcs=$(spike_srcs print-SOC_SRCS)
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
    # VexRiscv row in the same harness -- and its list, so it stays comparable.
    srcs=$(spike_srcs print-COMPARE_SRCS COMPARE_CORE=littlecpu)
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

# The band these rows are read against, for the part they were placed on. Printed
# to stderr so it cannot land in the CSV a caller is redirecting stdout into.
python3 soc/bands.py "$part" --note >&2

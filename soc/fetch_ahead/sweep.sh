#!/usr/bin/env bash
# Place base (shipping rtl/), proto (predict-not-taken, one-cycle discard), btfn (proto
# plus a static backward-taken guess) and, on request, decoupled (prototype-decoupled.patch)
# on up5k or ECP5, one CSV row per seed. Each tree's own Makefile names its sources.
set -euo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
cd "$root"
out=${FETCH_AHEAD_OUT:-$root/fetch_ahead.out}
mkdir -p "$out"

part=${1:-up5k}
shift || true
seeds=("$@")
if [ ${#seeds[@]} -eq 0 ]; then seeds=(0 1 2 3 4 5 6 7 8 9 10 11); fi

python3 soc/depth/row.py --header

variants=(base proto btfn)
if [ -n "${FETCH_AHEAD_VARIANTS:-}" ]; then
  read -r -a variants <<< "$FETCH_AHEAD_VARIANTS"
fi
for variant in "${variants[@]}"; do
  case "$variant" in
    base)      srcdir=$root; chp="" ;;
    proto)     srcdir=$out/applied; chp="chparam -set PREDICT_BTFN 0 littlecpu;" ;;
    btfn)      srcdir=$out/applied; chp="chparam -set PREDICT_BTFN 1 littlecpu;" ;;
    decoupled) srcdir=$out/decoupled; chp="" ;;
    *) echo "unknown variant $variant: base, proto, btfn or decoupled" >&2; exit 2 ;;
  esac
  case "$variant" in
    proto|btfn) bash soc/fetch_ahead/apply.sh "$srcdir" >/dev/null ;;
    decoupled)  bash soc/fetch_ahead/apply-decoupled.sh "$srcdir" >/dev/null ;;
  esac
  SOC_SRCS=$(make -s -C "$srcdir" print-SOC_SRCS)
  tag="$part.$variant"

  make -C "$srcdir" -s soc-rom SOC_PROG=add.S

  case "$part" in
    up5k)
      ( cd "$srcdir" && yosys -p "read_verilog -sv $SOC_SRCS; $chp synth_ice40 -device u -dsp -spram -top littlesoc -json $out/$tag.json; stat" ) \
        > "$out/$tag.synth.log" 2>&1 || { tail -30 "$out/$tag.synth.log"; exit 1; }
      ;;
    ecp5)
      ( cd "$srcdir" && yosys -p "read_verilog -sv $SOC_SRCS; $chp synth_ecp5 -top littlesoc -json $out/$tag.json; stat" ) \
        > "$out/$tag.synth.log" 2>&1 || { tail -30 "$out/$tag.synth.log"; exit 1; }
      ;;
    *) echo "usage: $0 {up5k|ecp5} [seed ...]" >&2; exit 2 ;;
  esac

  for seed in "${seeds[@]}"; do
    case "$part" in
      up5k)
        asc="$out/$tag.$seed.asc"
        seed_arg=""
        [ "$seed" = 0 ] || seed_arg="--seed $seed"
        nextpnr-ice40 --up5k --package sg48 $seed_arg --json "$out/$tag.json" \
          --pcf soc/littlesoc.pcf --asc "$asc" > "$out/$tag.$seed.pnr.log" 2>&1 || true
        if [ ! -s "$asc" ]; then
          echo "*** $tag seed $seed produced no bitstream" >&2
          tail -20 "$out/$tag.$seed.pnr.log" >&2
          exit 1
        fi
        icetime -d up5k -P sg48 -p soc/littlesoc.pcf -t -r "$out/$tag.$seed.rpt" "$asc" \
          > "$out/$tag.$seed.icetime.log" 2>&1
        lc=$(sed -n 's/.*ICESTORM_LC: *\([0-9]*\)\/.*/\1/p' "$out/$tag.$seed.pnr.log" | tail -1)
        python3 soc/depth/row.py "$out/$tag.$seed.rpt" "$part" "$variant" "$seed" "$lc"
        ;;
      ecp5)
        cfg="$out/$tag.$seed.config"
        report="$out/$tag.$seed.report.json"
        seed_arg=""
        [ "$seed" = 0 ] || seed_arg="--seed $seed"
        nextpnr-ecp5 --25k --package CABGA381 --speed 6 --json "$out/$tag.json" \
          --lpf soc/littlesoc.lpf --lpf-allow-unconstrained --freq 200.0 $seed_arg \
          --textcfg "$cfg" --report "$report" > "$out/$tag.$seed.pnr.log" 2>&1 || true
        if [ ! -s "$cfg" ] || [ ! -s "$report" ]; then
          echo "*** $tag seed $seed produced no bitstream/report" >&2
          tail -20 "$out/$tag.$seed.pnr.log" >&2
          exit 1
        fi
        python3 soc/depth/row.py --ecp5 "$report" "$cfg" LFE5U-25F-6CABGA381 clk 200.0 \
          "$variant" "$seed"
        ;;
    esac
  done
done

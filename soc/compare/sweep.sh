#!/bin/sh
# One placement is a sample, not a verdict: twelve to sixteen seeds are the go/no-go.
set -eu

cd "$(dirname "$0")/../.."

seeds=${COMPARE_SEEDS:-"default 1 2 3"}
cores=${COMPARE_CORES:-"littlecpu vexriscv hazard3"}
part=${COMPARE_PART:-up5k}

case $part in
  up5k|ecp5) ;;
  *) echo "*** soc/compare/sweep.sh: COMPARE_PART is '$part'; this harness knows" >&2
     echo "*** up5k and ecp5. hx8k was removed, not renamed." >&2
     exit 2 ;;
esac

# The two arms answer different questions, so they read different lines. up5k's clock is
# a step function and `make compare-timing` grades it pass/fail; ECP5's is synthesised on
# a fine grid, so its frequency is a factor and gets a distribution.
case $part in
  up5k) figure='^critical path :' ;;
  ecp5) figure='^Fmax          :' ;;
esac

for core in $cores; do
  echo "== $core on $part"
  rows=""
  for seed in $seeds; do
    case $seed in
      default) arg="" ;;
      *)       arg=$seed ;;
    esac
    case $part in
      up5k) seed_var="COMPARE_SEED=$arg" ;;
      ecp5) seed_var="ECP5_SEED=$arg" ;;
    esac
    if ! out=$(make compare-timing COMPARE_PART="$part" COMPARE_CORE="$core" \
                 "$seed_var" "$@" 2>&1); then
      printf '%s\n' "$out" >&2
      echo "*** soc/compare/sweep.sh: $core seed '$seed' failed on $part; the sweep" >&2
      echo "*** stops here. On up5k a core under the 12 MHz step FAILS rather than" >&2
      echo "*** scoring a fraction, so read the output above before calling this a" >&2
      echo "*** broken run." >&2
      exit 1
    fi
    line=$(printf '%s\n' "$out" | grep "$figure") || {
      echo "*** soc/compare/sweep.sh: $core seed '$seed' exited 0 with no" >&2
      echo "*** '$figure' line, which the reader for $part is supposed to make" >&2
      echo "*** impossible." >&2
      exit 1
    }
    case $part in
      up5k)
        ns=$(printf '%s\n' "$line" | sed 's/^critical path : \([0-9.]*\) ns.*/\1/')
        mhz=$(printf '%s\n' "$line" | sed 's/.*(\([0-9.]*\) MHz).*/\1/')
        verdict=$(printf '%s\n' "$out" | grep '^STEP GATE:' || echo 'STEP GATE: not read')
        lc=$(printf '%s\n' "$out" | grep 'placed ICESTORM_LC against')
        printf '  seed %-8s %8s ns  %6s MHz   %s\n' "$seed" "$ns" "$mhz" "$verdict"
        ;;
      ecp5)
        mhz=$(printf '%s\n' "$line" | sed 's/^Fmax  *: *\([0-9.]*\) MHz.*/\1/')
        ns=$(printf '%s\n' "$line" | sed 's/.*(\([0-9.]*\) ns).*/\1/')
        lc=$(printf '%s\n' "$out" | grep 'placed TRELLIS_COMB against')
        printf '  seed %-8s %8s ns  %6s MHz\n' "$seed" "$ns" "$mhz"
        ;;
    esac
    printf '           %s\n' "$lc"
    rows="$rows$ns "
  done
  echo "  sorted ns: $(printf '%s\n' $rows | sort -n | tr '\n' ' ')"
  python3 soc/compare/spread.py --part "$part" --core "$core" $rows
  echo
done

echo "Read the WORST placement of each. No core's number here is its own project's"
echo "published figure, and the ISAs are not the same -- ADR-0086."
echo
case $part in
  up5k)
    echo "up5k's clock is a STEP FUNCTION -- the board crystal, or SB_HFOSC's"
    echo "48/24/12/6. Every core that clears the step runs at exactly 12 MHz, so the"
    echo "margin above it is not a factor and the comparison is CYCLES ALONE. The"
    echo "spread column above says how wide the sample was; it is not a delta, and"
    echo "up5k's own band was derived on littlesoc rather than on this bench, so it"
    echo "is deliberately not quoted here."
    ;;
  ecp5)
    echo "EHXPLLL synthesises ref x M / N / D on a fine grid, so Fmax IS a factor"
    echo "here and the product is Fmax times cycles. It publishes and does not"
    echo "ratchet:"
    python3 soc/bands.py ecp5 --note
    ;;
esac

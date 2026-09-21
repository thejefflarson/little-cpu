#!/bin/sh
# Forces nano_tt_tb.v's uio_oe X check red: a mutant top that leaves one uio_oe bit
# undriven must fail there, or the check that pad-tristate control is always defined has
# never actually fired.
set -eu

CFLAGS=$1
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WORKDIR="$HERE/nano-uio-oe-probe"
TOP="$REPO/nano/tt/src/tt_um_thejefflarson_nanocpu.v"

rm -rf "$WORKDIR"
mkdir -p "$WORKDIR"

if ! grep -qF "assign uio_oe  = {{4{sio_oe}}, 4'b1111};" "$TOP"; then
  echo "error: $TOP no longer spells uio_oe's assignment the way this probe mutates." \
       "Re-anchor the sed pattern on the new spelling -- left alone this would run the" \
       "shipping top twice and prove nothing about an undriven pad." >&2
  exit 2
fi

mutant="$WORKDIR/tt_um_mutant.v"
sed "s/assign uio_oe  = {{4{sio_oe}}, 4'b1111};/logic uio_oe_undriven; assign uio_oe = {{3{sio_oe}}, uio_oe_undriven, 4'b1111};/" \
  "$TOP" > "$mutant"
if cmp -s "$TOP" "$mutant"; then
  echo "error: the mutant top is identical to the shipping one." >&2
  exit 2
fi

echo "control: the shipping top"
if ! out=$(NANO_TT_TOP="$TOP" "$HERE/../tb/run_nano_tt_test.sh" "$CFLAGS" 2>&1); then
  echo "$out"
  echo "*** the shipping top does not pass its own GPIO/UART test, so a mutant" \
       "failing the same way would prove nothing." >&2
  exit 1
fi
echo "$out"

echo
echo "mutant: uio_oe[4] left undriven"
if out=$(NANO_TT_TOP="$mutant" "$HERE/../tb/run_nano_tt_test.sh" "$CFLAGS" 2>&1); then
  echo "$out"
  echo "*** a top that leaves one uio_oe bit undriven still passes." >&2
  exit 1
fi
echo "$out"
if ! printf '%s\n' "$out" | grep -q 'uio_oe is X'; then
  echo "*** the mutant failed, but not for the reason this probe expects -- it should" \
       "report uio_oe reading X." >&2
  exit 1
fi

echo
echo "nano_tt_tb.v's uio_oe check passes the shipping top and catches an undriven pad bit."

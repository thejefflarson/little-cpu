#!/bin/sh
# Forces tt_gpio_uart.S's timer section red through the top's pins: a top that leaves the core's irq_mtip tied low must time out waiting for the interrupt, and a bus that never answers the timer's addresses must read mtimecmp back as zero.
set -eu

CFLAGS=$1
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WORKDIR="$HERE/nano-tt-timer-probe"
TOP="$REPO/nano/tt/src/tt_um_thejefflarson_nanocpu.v"
BUS="$REPO/nano/bus.v"

rm -rf "$WORKDIR"
mkdir -p "$WORKDIR"

mutate() {  # $1 = source, $2 = sed program, $3 = mutant path, $4 = what it breaks
  sed "$2" "$1" > "$3"
  if cmp -s "$1" "$3"; then
    echo "error: $1 no longer spells the line the $4 mutant rewrites. Re-anchor the sed" \
         "pattern -- left alone this would run the shipping top twice and prove nothing." >&2
    exit 2
  fi
}

mutate "$TOP" "s/\.irq_mtip(mtip),/.irq_mtip(1'b0),/" "$WORKDIR/tt_um_mutant.v" "irq_mtip"
mutate "$BUS" "s/timer_sel ? timer_rdata : 32'b0/32'b0/" "$WORKDIR/bus_mutant.v" "timer read"

echo "control: the shipping top and bus"
if ! out=$(NANO_TT_TOP="$TOP" NANO_TT_BUS="$BUS" "$HERE/run_nano_tt_test.sh" "$CFLAGS" 2>&1); then
  echo "$out"
  echo "*** the shipping top does not pass its own test, so a mutant failing the same" \
       "way would prove nothing." >&2
  exit 1
fi
echo "$out" | tail -2

expect_red() {  # $1 = what the mutant does, $2 = the verdict it must draw, $3 = top, $4 = bus
  echo
  echo "mutant: $1"
  if out=$(NANO_TT_TOP="$3" NANO_TT_BUS="$4" "$HERE/run_nano_tt_test.sh" "$CFLAGS" 2>&1); then
    echo "$out" | tail -3
    echo "*** the $1 mutant still passes." >&2
    exit 1
  fi
  if ! printf '%s\n' "$out" | grep -q "tohost verdict $2\$"; then
    echo "$out" | tail -5
    echo "*** the $1 mutant failed, but not at test $2 of tt_gpio_uart.S." >&2
    exit 1
  fi
  printf '%s\n' "$out" | grep "tohost verdict"
}

expect_red "the core's irq_mtip tied low" 4 "$WORKDIR/tt_um_mutant.v" "$BUS"
expect_red "the bus never answers the timer" 6 "$TOP" "$WORKDIR/bus_mutant.v"

echo
echo "tt_gpio_uart.S's timer section passes the shipping top and fails both wiring mutants at its own tests."

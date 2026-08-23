#!/bin/bash
# Measures the board's real clock through its own UART.
#
# rtl/uart.v divides the clock by a constant for 115200, so the rate on the wire
# moves with the oscillator:  wire_baud = 115200 * f / 12e6.  Move the HOST to
# meet it and the rate that decodes tells you f.
#
# READ THE PLATEAU, NOT A WINNER. 8N1 tolerates a few percent, so a band of
# rates decodes cleanly; its centre is the board's rate and its width is the
# uncertainty. A single best score is noise -- an early version of this sampled
# 900ms per rate against a board that printed once a second, and "won" at
# whichever window happened to catch a line. Hence the long windows and the
# minimum byte count below.
set -uo pipefail
FTREAD=${FTREAD:-$(cd "$(dirname "$0")" && pwd)/ftread}
MS=${MS:-6000}
[ -x "$FTREAD" ] || { echo "no ftread at $FTREAD -- build it with 'make ftread'" >&2; exit 1; }

echo "rate     bytes  clean%  digits eol  sample"
for baud in 105000 108000 110000 112000 114000 115200 116000 118000 120000 122000 125000; do
  out=$("$FTREAD" "$baud" "$MS" 2>/tmp/uartsweep.err)
  st=$(tr -d '\n' < /tmp/uartsweep.err)
  b=$(sed -E 's/.*bytes=([0-9]+).*/\1/' <<<"$st"); p=$(sed -E 's/.*printable=([0-9]+).*/\1/' <<<"$st")
  d=$(sed -E 's/.*digits=([0-9]+).*/\1/' <<<"$st"); e=$(sed -E 's/.*eol=([0-9]+).*/\1/' <<<"$st")
  for v in b p d e; do [ -z "${!v}" ] && eval "$v=0"; done
  if [ "$b" -lt 8 ]; then
    printf '%-8s %5s  %6s  %6s %4s  (too few bytes to judge)\n' "$baud" "$b" "-" "$d" "$e"; continue
  fi
  pct=$(python3 -c "print(f'{100.0*($p+$e)/$b:.0f}')")
  mhz=$(python3 -c "print(f'{12e6*$baud/115200/1e6:.2f}')")
  printf '%-8s %5s  %5s%%  %6s %4s  %-24s %s MHz\n' "$baud" "$b" "$pct" "$d" "$e" \
    "$(printf '%s' "$out" | tr -d '\r' | tr '\n' ' ' | cut -c1-24)" "$mhz"
done
echo
echo "The centre of the 100%-clean band is the board's baud; f = 12e6 * baud / 115200."

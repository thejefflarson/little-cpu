#!/bin/bash
# Requires the iverilog wrapper to write no waveform without --vcd and the named file with it.
# Runs real vvp on a one-word jal-to-self image in a scratch dir, so a stray dump lands there.
set -euo pipefail

if [ "$#" -ne 1 ]; then
  echo "usage: nano_vcd_probe.sh <nano_sim_icarus.sh>" >&2
  exit 2
fi
WRAPPER=$(cd "$(dirname "$1")" && pwd)/$(basename "$1")
HERE=$(cd "$(dirname "$0")" && pwd)
WORKDIR="$HERE/nano-vcd-probe"

if [ ! -x "$WRAPPER" ]; then
  echo "error: '$WRAPPER' is not an executable wrapper." >&2
  exit 2
fi

rm -rf "$WORKDIR"
mkdir -p "$WORKDIR"

printf '0000006f\n' > "$WORKDIR/loop.rom.hex"
printf '00000000\n' > "$WORKDIR/loop.ram.hex"

run_wrapper() {  # extra wrapper arguments follow; the exit code is deliberately ignored
  (cd "$WORKDIR" && "$WRAPPER" --rom loop.rom.hex --ram loop.ram.hex --cycles 50 "$@") \
    > "$WORKDIR/run.log" 2>&1 || true
}

red=()

run_wrapper
stray=$(find "$WORKDIR" -name '*.vcd')
if [ -n "$stray" ]; then
  red+=("a run without --vcd wrote a waveform: $stray")
fi

run_wrapper --vcd "$WORKDIR/named.vcd"
if [ ! -s "$WORKDIR/named.vcd" ]; then
  red+=("a run with --vcd $WORKDIR/named.vcd did not write that file. Wrapper output:
$(cat "$WORKDIR/run.log")")
fi

if [ "${#red[@]}" -ne 0 ]; then
  for why in "${red[@]}"; do
    echo "*** $why" >&2
  done
  exit 1
fi

echo "no --vcd writes no waveform; --vcd <path> writes exactly that file."

#!/bin/bash
# Wraps nano/tb/nano_icarus.vvp behind nano-sim's own CLI and 0-6 exits, plus this leg's
# own 7 for an X reaching a retire, so nano/asm/run_nano_tests.sh drives either leg unmodified.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
VVP_IMAGE="$HERE/nano_icarus.vvp"

rom=""
ram=""
cycles=""
while [ "$#" -gt 0 ]; do
  case "$1" in
    --rom) rom=$2; shift 2 ;;
    --ram) ram=$2; shift 2 ;;
    --cycles) cycles=$2; shift 2 ;;
    --vcd) shift 2 ;;
    --bench) echo "error: --bench is not wired up on the iverilog leg yet" >&2; exit 3 ;;
    *) echo "error: unrecognized argument '$1'" >&2; exit 3 ;;
  esac
done
if [ -z "$rom" ] || [ -z "$ram" ] || [ -z "$cycles" ]; then
  echo "usage: nano_sim_icarus.sh --rom <hex> --ram <hex> --cycles N [--vcd out.vcd]" >&2
  exit 3
fi
if [ ! -f "$VVP_IMAGE" ]; then
  echo "error: '$VVP_IMAGE' does not exist; build it with 'make nano/tb/nano_icarus.vvp'." >&2
  exit 3
fi

set +e
out=$(vvp "$VVP_IMAGE" "+ROM=$rom" "+RAM=$ram" "+CYCLES=$cycles" 2>&1)
set -e
printf '%s\n' "$out"

retires=$(awk '/^RETIRES /{print $2; exit}' <<< "$out")

if grep -q '^PASS$' <<< "$out"; then
  code=0
elif grep -q '^FAIL ' <<< "$out"; then
  code=1
elif grep -q '^TIMEOUT$' <<< "$out"; then
  code=2
elif grep -q 'RVFI monitor error' <<< "$out"; then
  code=4
elif grep -q '^trap taken' <<< "$out"; then
  code=5
elif grep -q "^X reached a retiring instruction" <<< "$out"; then
  code=7
else
  echo "error: unrecognized nano_icarus.vvp output" >&2
  exit 3
fi

if [ -z "$retires" ] || [ "$retires" -eq 0 ]; then
  echo "the RVFI monitor observed nothing this run: 0 retires. The per-retire oracle was" >&2
  echo "blind, so this run's verdict (exit $code) means nothing." >&2
  exit 6
fi
exit "$code"

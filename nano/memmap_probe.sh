#!/bin/sh
# Forces nano/memmap_test.sh red for its own reason, the way nano/tb/nano_x_probe.sh
# forces the X-leg check red: a grader that has never failed proves nothing.
set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/.." && pwd)
MM="$HERE/memmap_test.sh"

fixture() {
  d=$(mktemp -d "${TMPDIR:-/tmp}/nano-memmap-probe.XXXXXX")
  mkdir -p "$d/nano/tt/src"
  cp "$REPO/nano/nano.v" "$REPO/nano/bus.v" "$REPO/nano/uart.v" "$REPO/nano/gpio.v" "$d/nano/"
  cp "$REPO/nano/tt/src/tt_um_thejefflarson_nanocpu.v" "$d/nano/tt/src/"
  printf '%s' "$d"
}

red=0

d=$(fixture)
if ! out=$("$MM" "$d" 2>&1); then
  echo "*** control: the shipping map is red on its own tree:" >&2
  echo "$out" >&2
  red=1
fi

# The one that matters: GPIO's base moved onto the UART's own base -- still 8-byte
# aligned, so only the overlap check can catch it.
d=$(fixture)
sed -i.bak "s/32'h1080_0008/32'h1080_0000/" "$d/nano/gpio.v"
if out=$("$MM" "$d" 2>&1); then
  echo "*** GPIO's base landing inside the UART's span is not red:" >&2
  echo "$out" >&2
  red=1
elif ! printf '%s' "$out" | grep -q "GPIO starts at"; then
  echo "*** GPIO overlapping the UART fails for the wrong reason:" >&2
  echo "$out" >&2
  red=1
fi

# The UART's base moved off PSRAM's own top, opening a gap the OR-of-selects design
# cannot cover as "PSRAM."
d=$(fixture)
sed -i.bak "s/32'h1080_0000/32'h1080_1000/" "$d/nano/uart.v"
if out=$("$MM" "$d" 2>&1); then
  echo "*** the UART drifting off PSRAM's top is not red:" >&2
  echo "$out" >&2
  red=1
elif ! printf '%s' "$out" | grep -q "PSRAM ends at"; then
  echo "*** the UART drifting off PSRAM's top fails for the wrong reason:" >&2
  echo "$out" >&2
  red=1
fi

# A misaligned GPIO base: a range test on the bits above an 8-byte window admits any
# address at a different alignment.
d=$(fixture)
sed -i.bak "s/32'h1080_0008/32'h1080_0009/" "$d/nano/gpio.v"
if out=$("$MM" "$d" 2>&1); then
  echo "*** a misaligned GPIO base is not red:" >&2
  echo "$out" >&2
  red=1
elif ! printf '%s' "$out" | grep -q "8-byte aligned"; then
  echo "*** a misaligned GPIO base fails for the wrong reason:" >&2
  echo "$out" >&2
  red=1
fi

if [ "$red" -ne 0 ]; then
  exit 1
fi

echo "nano/memmap_test.sh: the shipping map passes and three overlap/alignment mutants fail for their own reasons."

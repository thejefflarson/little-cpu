#!/bin/bash
# Builds soc/boot_loader.c and two program images, then runs the loader on rtl/littlesoc.v against a flash model
# holding each image (and damaged ones, and none) and compares what the UART says.
# SOC_SRCS names the RTL, as the Makefile's variable does. BOOT_LOADER_SRC replaces the loader (a probe's mutant);
# BOOT_CASES, a space-separated list of case names, runs only those.
set -uo pipefail
cd "$(dirname "$0")/.." || exit 1
: "${SOC_SRCS:?SOC_SRCS must name the SoC RTL; run this through make boot-loader-test}"
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
fail=0
bad() { echo "*** boot-loader: $1"; fail=1; }

tool() { python3 soc/boot_image.py "$@"; }
OFFSET=$(tool layout BOOT_FLASH_OFFSET)
BASE=$(tool layout BOOT_IMAGE_BASE)
ROM_BYTES=$(tool layout BOOT_ROM_BYTES)
IMAGE_MAX=$((ROM_BYTES - BASE))

mkdir -p "$WORK/soc"
make -s --no-print-directory soc-rom SOC_PROG="${BOOT_LOADER_SRC:-soc/boot_loader.c}" SOC_IMAGE_BIN="$WORK/loader.bin" || { echo "*** boot-loader: the loader does not build"; exit 1; }
loader_bytes=$(wc -c < "$WORK/loader.bin" | tr -d ' ')
[ "$loader_bytes" -le "$BASE" ] || bad "the loader is $loader_bytes bytes and an image is linked at $BASE, so the image would overwrite the code copying it"
tool vhex "$WORK/loader.bin" "$WORK/loader.hex"
python3 soc/rom_banks.py "$WORK/loader.hex" "$WORK/soc/rom_even.hex" "$WORK/soc/rom_odd.hex" --rom-words $((ROM_BYTES / 4)) > /dev/null || exit 1

for n in a b; do
  make -s --no-print-directory boot-image BOOT_PROG="test/boot/image_$n.c" BOOT_IMAGE="$WORK/$n.img" > /dev/null || { echo "*** boot-loader: image $n does not build"; exit 1; }
done
python3 test/boot_damage.py "$WORK/a.img" "$WORK" "$IMAGE_MAX" || exit 1

# shellcheck disable=SC2086
iverilog -g2012 -I rtl -Pboot_loader_sim.FLASH_OFFSET="$OFFSET" -Pboot_loader_sim.IMAGE_BYTES="$ROM_BYTES" \
  -o "$WORK/sim.vvp" $SOC_SRCS test/spiflash_model.v test/boot_loader_sim.v || { echo "*** boot-loader: the bench does not elaborate"; exit 1; }

# run NAME IMAGE LINES: the UART text, with the numbers that differ run to run masked.
run() {
  local name=$1 img=$2 lines=$3
  tool bytehex "$WORK/$img" "$WORK/$name.hex" --pad "$ROM_BYTES"
  (cd "$WORK" && vvp -n sim.vvp +lines="$lines" +image="$name.hex") > "$WORK/$name.out" 2>&1
  sed -e 's/cycles=[0-9]*/cycles=N/' -e 's/BADSUM [0-9A-F]* [0-9A-F]*/BADSUM W G/' "$WORK/$name.out" | grep -v 'finish called'
}

# expect NAME IMAGE LINES: the lines on stdin are the whole UART text.
expect() {
  local name=$1 img=$2 lines=$3 want got
  want=$(cat)
  case " ${BOOT_CASES:-$name} " in *" $name "*) ;; *) return 0;; esac
  got=$(run "$name" "$img" "$lines" | grep -v '^MAXGAP')
  if [ "$got" != "$want" ]; then
    bad "$name: wanted
$want
but the UART said
$got"
  fi
}

LEN_A=$(( $(wc -c < "$WORK/a.img") - 12 ))
LEN_B=$(( $(wc -c < "$WORK/b.img") - 12 ))
LIMIT=$(printf '%08X' "$IMAGE_MAX")

expect good_a a.img 3 <<EOF
BOOT
BOOT OK len=$LEN_A cycles=N
IMAGE A sum=10
EOF
expect good_b b.img 3 <<EOF
BOOT
BOOT OK len=$LEN_B cycles=N
IMAGE B product=006480
EOF
expect flipped flipped.img 2 <<EOF
BOOT
BOOT BADSUM W G
EOF
expect long long.img 2 <<EOF
BOOT
BOOT BADLEN $(printf '%08X' $((IMAGE_MAX + 4))) $LIMIT
EOF
expect odd odd.img 2 <<EOF
BOOT
BOOT BADLEN 00000007 $LIMIT
EOF
expect erased erased.img 2 <<EOF
BOOT
BOOT NOIMAGE FFFFFFFF 00000000
EOF

if [ "$fail" = 0 ]; then
  echo "boot-loader: two different images load and run; a flipped byte, a bad length and a missing image are reported and not run."
fi
exit "$fail"

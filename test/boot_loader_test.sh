#!/bin/bash
# Builds soc/boot_loader.c and two program images, then runs the loader on rtl/littlesoc.v against a flash model
# holding each image (and damaged ones, and none) and compares what the UART says. BOOT_TARGET=spram runs the same
# loader and format as the block-RAM boot ROM of rtl/littlesoc_spram.v instead, where a third image larger than the
# default board's 8 KB ROM must load and run too.
set -uo pipefail
cd "$(dirname "$0")/.." || exit 1
: "${SOC_SRCS:?SOC_SRCS must name the SoC RTL; run this through make boot-loader-test}"
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
fail=0
bad() { echo "*** boot-loader: $1"; fail=1; }

TARGET=${BOOT_TARGET:-}
tool() { python3 soc/boot_image.py ${TARGET:+--target "$TARGET"} "$@"; }
OFFSET=$(tool layout BOOT_FLASH_OFFSET)
BASE=$(tool layout BOOT_IMAGE_BASE)
ROM_BYTES=$(tool layout BOOT_ROM_BYTES)
IMAGE_MAX=$((ROM_BYTES - BASE))

mkdir -p "$WORK/soc"
if [ -z "$TARGET" ]; then
  make -s --no-print-directory soc-rom SOC_PROG="${BOOT_LOADER_SRC:-soc/boot_loader.c}" SOC_IMAGE_BIN="$WORK/loader.bin" || { echo "*** boot-loader: the loader does not build"; exit 1; }
  loader_bytes=$(wc -c < "$WORK/loader.bin" | tr -d ' ')
  [ "$loader_bytes" -le "$BASE" ] || bad "the loader is $loader_bytes bytes and an image is linked at $BASE, so the image would overwrite the code copying it"
  tool vhex "$WORK/loader.bin" "$WORK/loader.hex"
  python3 soc/rom_banks.py "$WORK/loader.hex" "$WORK/soc/rom_even.hex" "$WORK/soc/rom_odd.hex" --rom-words $((ROM_BYTES / 4)) > /dev/null || exit 1
  IMAGES="a b"
  SIM_FLAGS=
else
  LOADER_BASE=$(tool layout BOOT_SPRAM_LOADER_BASE)
  RAM_BYTES=$(tool layout BOOT_RAM_BYTES)
  make -s --no-print-directory soc-rom SOC_PROG="${BOOT_LOADER_SRC:-soc/boot_loader.c}" SOC_LDS=soc/bootrom.lds \
    SOC_LDFLAGS="-DBOOT_SPRAM -Wl,--defsym=BOOT_SPRAM_LOADER_BASE=$LOADER_BASE -Wl,--defsym=BOOT_SPRAM_RAM_BYTES=$RAM_BYTES" \
    SOC_IMAGE_BIN="$WORK/loader.bin" || { echo "*** boot-loader: the loader does not build"; exit 1; }
  loader_bytes=$(wc -c < "$WORK/loader.bin" | tr -d ' ')
  [ "$loader_bytes" -le 1024 ] || bad "the loader is $loader_bytes bytes and the boot ROM holds 1024"
  tool vhex "$WORK/loader.bin" "$WORK/loader.hex"
  python3 soc/rom_banks.py "$WORK/loader.hex" "$WORK/soc/bootrom_even.hex" "$WORK/soc/bootrom_odd.hex" --rom-words 256 > /dev/null || exit 1
  IMAGES="a b big"
  SIM_FLAGS="-DBOOT_SPRAM"
fi

for n in $IMAGES; do
  make -s --no-print-directory boot-image BOOT_TARGET="$TARGET" BOOT_PROG="test/boot/image_$n.c" BOOT_IMAGE="$WORK/$n.img" > /dev/null || { echo "*** boot-loader: image $n does not build"; exit 1; }
done
python3 test/boot_damage.py "$WORK/a.img" "$WORK" "$IMAGE_MAX" || exit 1

# shellcheck disable=SC2086
iverilog -g2012 $SIM_FLAGS -I rtl -Pboot_loader_sim.FLASH_OFFSET="$OFFSET" -Pboot_loader_sim.IMAGE_BYTES="$ROM_BYTES" \
  -o "$WORK/sim.vvp" $SOC_SRCS test/spiflash_model.v test/boot_loader_sim.v || { echo "*** boot-loader: the bench does not elaborate"; exit 1; }

run() {
  local name=$1 img=$2 lines=$3
  tool bytehex "$WORK/$img" "$WORK/$name.hex" --pad "$ROM_BYTES"
  (cd "$WORK" && vvp -n sim.vvp +lines="$lines" +image="$name.hex") > "$WORK/$name.out" 2>&1
  sed -e 's/cycles=[0-9]*/cycles=N/' -e 's/BADSUM [0-9A-F]* [0-9A-F]*/BADSUM W G/' "$WORK/$name.out" | grep -v 'finish called'
}

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
LEN_BIG=0
[ -z "$TARGET" ] || LEN_BIG=$(( $(wc -c < "$WORK/big.img") - 12 ))
LIMIT=$(printf '%08X' "$IMAGE_MAX")

expect good_a a.img 4 <<EOF
BOOT
BOOT OK len=$LEN_A cycles=N
IMAGE A sum=10
LOAD cycles=N
EOF
expect good_b b.img 4 <<EOF
BOOT
BOOT OK len=$LEN_B cycles=N
IMAGE B product=006480
LOAD cycles=N
EOF
if [ -n "$TARGET" ]; then
  [ "$LEN_BIG" -gt 8192 ] || bad "image big is $LEN_BIG bytes, no larger than the default board's 8 KB ROM, so it proves nothing"
  expect good_big big.img 4 <<EOF
BOOT
BOOT OK len=$LEN_BIG cycles=N
IMAGE BIG ok
LOAD cycles=N
EOF
fi
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
  echo "boot-loader${TARGET:+ ($TARGET)}: two different images load and run${TARGET:+, and one larger than the default ROM}; a flipped byte, a bad length and a missing image are reported and not run."
fi
exit "$fail"

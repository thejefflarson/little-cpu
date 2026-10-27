#!/bin/bash
# Compiles soc/lc_flash.c and soc/lc_uart.c against stub iceprog and ftread, and drives each with arguments whose answer is known.
# Unprivileged: the privilege drop for a root caller (SUDO_UID) needs sudo, which this test does not use.
set -uo pipefail
ROOT=${1:-$(cd "$(dirname "$0")/.." && pwd)}
WORK=$(mktemp -d)
trap 'chmod -R u+rwx "$WORK" 2>/dev/null; rm -rf "$WORK"' EXIT
fail=0
bad() { echo "*** board-wrappers: $1"; fail=1; }

case $WORK in */build/*) echo "error: scratch $WORK sits under a build/ directory, which makes every out-of-tree case an in-tree one" >&2; exit 1;; esac

# The wrappers scrub the environment, so the stubs name their output directory literally.
STUB_OUT=$WORK/out
printf '#!/bin/bash\necho "$@" > "%s/iceprog.args"\nfor a; do case $a in /dev/fd/*) cat "$a" > "%s/iceprog.data";; esac; done\n/usr/bin/env > "%s/iceprog.env"\n' "$STUB_OUT" "$STUB_OUT" "$STUB_OUT" > "$WORK/iceprog"
printf '#!/bin/bash\necho "$@" > "%s/ftread.args"\n' "$STUB_OUT" > "$WORK/ftread"
chmod +x "$WORK/iceprog" "$WORK/ftread"
cc -O2 -Wall -Werror -DREAL_ICEPROG="\"$WORK/iceprog\"" -o "$WORK/lc-flash" "$ROOT/soc/lc_flash.c" || { echo "*** board-wrappers: lc_flash.c does not compile"; exit 1; }
cc -O2 -Wall -Werror -DREAL_FTREAD="\"$WORK/ftread\"" -o "$WORK/lc-uart" "$ROOT/soc/lc_uart.c" || { echo "*** board-wrappers: lc_uart.c does not compile"; exit 1; }

CK=$WORK/checkout
mkdir -p "$STUB_OUT" "$CK/build/sub" "$WORK/elsewhere"
printf 'bitstream' > "$CK/build/ok.bin"
printf 'secret' > "$WORK/elsewhere/secret.bin"
printf 'sealed' > "$CK/build/sealed.bin"
chmod 000 "$CK/build/sealed.bin"
: > "$CK/build/empty.bin"
ln -s ../../elsewhere/secret.bin "$CK/build/escape.bin"
ln -s ok.bin "$CK/build/inside.bin"

reached() { [ -e "$STUB_OUT/iceprog.args" ] || [ -e "$STUB_OUT/ftread.args" ]; }
accepts() {
  local what=$1; shift
  rm -f "$STUB_OUT"/*
  "$@" >"$WORK/log" 2>&1 || bad "$what: refused: $(cat "$WORK/log")"
}
refuses() {
  local what=$1; shift
  rm -f "$STUB_OUT"/*
  if "$@" >"$WORK/log" 2>&1 || reached; then bad "$what"; fi
}
iceprog_args() { cat "$STUB_OUT/iceprog.args" 2>/dev/null; }

accepts "a bitstream under build/ was refused" "$WORK/lc-flash" "$CK/build/ok.bin"
[ "$(cat "$STUB_OUT/iceprog.data" 2>/dev/null)" = bitstream ] || bad "the bitstream did not reach iceprog through the descriptor it was handed"
case $(iceprog_args) in /dev/fd/[0-9]*) ;; *) bad "iceprog was handed a path, not the opened descriptor: $(iceprog_args)";; esac
accepts "a relative path under build/ was refused" bash -c "cd '$CK' && '$WORK/lc-flash' build/ok.bin"
accepts "a symlink that stays under build/ was refused" "$WORK/lc-flash" "$CK/build/inside.bin"
accepts "a .. that returns to build/ was refused" "$WORK/lc-flash" "$CK/build/sub/../../build/ok.bin"
accepts "-d with a USB id was refused" "$WORK/lc-flash" -d i:0x0403:0x6014 "$CK/build/ok.bin"
case $(iceprog_args) in "-d i:0x0403:0x6014 /dev/fd/"*) ;; *) bad "-d did not reach iceprog as given: $(iceprog_args)";; esac
accepts "-t was refused" "$WORK/lc-flash" -t
[ "$(iceprog_args)" = "-t" ] || bad "-t reached iceprog as '$(iceprog_args)'"

accepts "the environment case was refused" env FOO_SECRET=1 LD_PRELOAD=/x "$WORK/lc-flash" "$CK/build/ok.bin"
grep -q '^PATH=' "$STUB_OUT/iceprog.env" || bad "the stub did not capture an environment, so the scrub case proves nothing"
if grep -qE '^(FOO_SECRET|LD_PRELOAD)=' "$STUB_OUT/iceprog.env"; then bad "iceprog inherited the caller's environment"; fi

boot() { python3 "$ROOT/soc/boot_image.py" "$@"; }
OFF=$(boot layout BOOT_FLASH_OFFSET)
ALIGN=$(boot layout BOOT_FLASH_ALIGN)
END=$(boot layout BOOT_FLASH_BYTES)
printf 'abcd' > "$WORK/payload"
boot pack "$WORK/payload" "$CK/build/boot.img"
head -c 12 "$CK/build/boot.img" > "$CK/build/header.img"
cp "$CK/build/boot.img" "$CK/build/lied.img"
printf '\010' | dd of="$CK/build/lied.img" bs=1 seek=4 conv=notrunc 2> /dev/null
# Headers that are right in every way but one, over bodies the wrapper must still refuse: past its size bound,
# not a whole number of words, and the wrong magic.
python3 - "$ROOT" "$CK/build" <<'EOF'
import struct, sys
sys.path.insert(0, sys.argv[1] + "/soc")
import boot_image
magic = boot_image.layout()["BOOT_MAGIC"]
for name, length, tag in (("big", 70000, magic), ("ragged", 7, magic), ("nomagic", 4, magic ^ 1)):
    open(f"{sys.argv[2]}/{name}.img", "wb").write(struct.pack("<III", tag, length, 0) + b"\0" * length)
EOF

accepts "a boot image at the boot offset was refused" "$WORK/lc-flash" -o "$OFF" "$CK/build/boot.img"
[ "$(iceprog_args | sed 's|/dev/fd/[0-9]*|FD|')" = "-o $OFF FD" ] || bad "-o did not reach iceprog as given: $(iceprog_args)"
cmp -s "$CK/build/boot.img" "$STUB_OUT/iceprog.data" || bad "the image did not reach iceprog through the descriptor it was handed"
accepts "a boot image at a higher block was refused" "$WORK/lc-flash" -o $((OFF + ALIGN)) "$CK/build/boot.img"
accepts "the last block of the flash was refused" "$WORK/lc-flash" -o $((END - ALIGN)) "$CK/build/boot.img"
accepts "-d then -o was refused" "$WORK/lc-flash" -d i:0x0403:0x6014 -o "$OFF" "$CK/build/boot.img"
case $(iceprog_args) in "-d i:0x0403:0x6014 -o $OFF /dev/fd/"*) ;; *) bad "-d and -o did not reach iceprog in order: $(iceprog_args)";; esac

refuses "an offset inside the bitstream was accepted" "$WORK/lc-flash" -o "$ALIGN" "$CK/build/boot.img"
refuses "an offset of zero was accepted" "$WORK/lc-flash" -o 0 "$CK/build/boot.img"
refuses "an offset below the boot region was accepted" "$WORK/lc-flash" -o $((OFF - ALIGN)) "$CK/build/boot.img"
refuses "an unaligned offset was accepted" "$WORK/lc-flash" -o $((OFF + 4096)) "$CK/build/boot.img"
refuses "an offset one byte in was accepted" "$WORK/lc-flash" -o $((OFF + 1)) "$CK/build/boot.img"
refuses "an offset at the end of the flash was accepted" "$WORK/lc-flash" -o "$END" "$CK/build/boot.img"
refuses "an offset past the flash was accepted" "$WORK/lc-flash" -o $((END + ALIGN)) "$CK/build/boot.img"
refuses "a hex offset was accepted" "$WORK/lc-flash" -o "$(printf '0x%x' "$OFF")" "$CK/build/boot.img"
refuses "an offset with a leading zero was accepted" "$WORK/lc-flash" -o "0$OFF" "$CK/build/boot.img"
refuses "a negative offset was accepted" "$WORK/lc-flash" -o -"$OFF" "$CK/build/boot.img"
refuses "an offset with a suffix was accepted" "$WORK/lc-flash" -o "${OFF}k" "$CK/build/boot.img"
refuses "an offset too long to be one was accepted" "$WORK/lc-flash" -o 99999999999 "$CK/build/boot.img"
refuses "an empty offset was accepted" "$WORK/lc-flash" -o "" "$CK/build/boot.img"
refuses "-o with no file was accepted" "$WORK/lc-flash" -o "$OFF"
refuses "-o twice was accepted" "$WORK/lc-flash" -o "$OFF" -o "$OFF" "$CK/build/boot.img"
refuses "-o with -t was accepted" "$WORK/lc-flash" -o "$OFF" -t
refuses "-t then -o was accepted" "$WORK/lc-flash" -t -o "$OFF"
refuses "an image over the boot image bound was accepted" "$WORK/lc-flash" -o "$OFF" "$CK/build/big.img"
refuses "an image that is not a whole number of words was accepted" "$WORK/lc-flash" -o "$OFF" "$CK/build/ragged.img"
refuses "a bare header with no payload was accepted" "$WORK/lc-flash" -o "$OFF" "$CK/build/header.img"
refuses "a file without the image magic was accepted" "$WORK/lc-flash" -o "$OFF" "$CK/build/nomagic.img"
refuses "an image whose length does not match its size was accepted" "$WORK/lc-flash" -o "$OFF" "$CK/build/lied.img"
refuses "a bitstream was accepted as a boot image" "$WORK/lc-flash" -o "$OFF" "$CK/build/ok.bin"
refuses "a boot image outside build/ was accepted" "$WORK/lc-flash" -o "$OFF" "$WORK/payload"

refuses "-r was accepted: iceprog -r writes its argument as root" "$WORK/lc-flash" -r "$CK/build/ok.bin"
refuses "-r with an output path was accepted" "$WORK/lc-flash" -r "$WORK/outfile"
for flag in -R -S -c -v -b -I; do
  refuses "$flag was accepted" "$WORK/lc-flash" "$flag" "$CK/build/ok.bin"
done
refuses "extra arguments after the bitstream were accepted" "$WORK/lc-flash" "$CK/build/ok.bin" -r
refuses "-t with a file was accepted" "$WORK/lc-flash" -t "$CK/build/ok.bin"
refuses "no arguments were accepted" "$WORK/lc-flash"
refuses "a -d value with a path in it was accepted" "$WORK/lc-flash" -d "i:../../x" "$CK/build/ok.bin"
refuses "a -d value that is not a USB id was accepted" "$WORK/lc-flash" -d /dev/null -t
refuses "a path outside build/ was accepted" "$WORK/lc-flash" "$WORK/elsewhere/secret.bin"
refuses "a .. that leaves build/ was accepted" "$WORK/lc-flash" "$CK/build/../../elsewhere/secret.bin"
refuses "a symlink in build/ that escapes it was accepted" "$WORK/lc-flash" "$CK/build/escape.bin"
refuses "a relative symlink escape was accepted" bash -c "cd '$CK/build' && '$WORK/lc-flash' escape.bin"
if [ "$(id -u)" != 0 ]; then
  refuses "a file the caller cannot read was accepted" "$WORK/lc-flash" "$CK/build/sealed.bin"
fi
refuses "a directory under build/ was accepted" "$WORK/lc-flash" "$CK/build/sub"
refuses "a missing file was accepted" "$WORK/lc-flash" "$CK/build/missing.bin"
refuses "an empty file was accepted" "$WORK/lc-flash" "$CK/build/empty.bin"

accepts "lc-uart refused a bounded read" "$WORK/lc-uart" 8000
[ "$(cat "$STUB_OUT/ftread.args" 2>/dev/null)" = "115200 8000" ] || bad "lc-uart did not read at the fixed baud: '$(cat "$STUB_OUT/ftread.args" 2>/dev/null)'"
refuses "lc-uart took a baud as well" "$WORK/lc-uart" 9600 8000
refuses "lc-uart took no arguments" "$WORK/lc-uart"
refuses "lc-uart took zero milliseconds" "$WORK/lc-uart" 0
refuses "lc-uart took a negative number" "$WORK/lc-uart" -5
refuses "lc-uart took an unbounded read" "$WORK/lc-uart" 99999999999
refuses "lc-uart took a read over an hour" "$WORK/lc-uart" 3600001
refuses "lc-uart took a non-number" "$WORK/lc-uart" 10x
refuses "lc-uart took an empty argument" "$WORK/lc-uart" ""

if [ "$fail" = 0 ]; then
  echo "board-wrappers: lc-flash takes only a build/ bitstream opened as the caller, a build/ boot image behind a 64 KiB-aligned offset past the bitstream, or -t, lc-uart only a bounded read; every other shape is refused."
fi
exit "$fail"

#!/bin/bash
# Two bitstreams packed from different BOARD_ROM values: make prog's pre-flash check must refuse a request naming the other one and accept a match.
# usage: board_stamp_test.sh [ROOT]. ROOT holds the soc/board_stamp.sh under test; the make-level cases always run the real Makefile with a stub in place of iceprog.
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/.." && pwd)
ROOT=${1:-$REPO}
STAMP=$ROOT/soc/board_stamp.sh
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
fail=0
bad() { echo "*** board-stamp: $1"; fail=1; }

printf 'rom image one\n' > "$WORK/one.hex"
printf 'rom image two\n' > "$WORK/two.hex"
mkdir -p "$WORK/a" "$WORK/b"
printf 'bitstream from X\n' > "$WORK/a/board.bin"
printf 'bitstream from Y\n' > "$WORK/b/board.bin"
"$STAMP" write "$WORK/a/board.bin" rom-x crystal "$WORK/one.hex" || bad "write failed for X"
"$STAMP" write "$WORK/b/board.bin" rom-y internal "$WORK/two.hex" || bad "write failed for Y"

out=$("$STAMP" check "$WORK/a/board.bin" rom-y '' 2>&1) && bad "a request for rom-y was accepted against an image built from rom-x"
[[ $out == *"BOARD_ROM=rom-x"* && $out == *"BOARD_ROM=rom-y"* ]] || bad "the refusal does not name both the stamp and the request: $out"
out=$("$STAMP" check "$WORK/a/board.bin" '' internal 2>&1) && bad "a request for another BOARD_OSC was accepted"
"$STAMP" check "$WORK/a/board.bin" rom-x crystal > /dev/null 2>&1 || bad "the matching request was refused"
"$STAMP" check "$WORK/a/board.bin" '' '' > /dev/null 2>&1 || bad "a bare request was refused"
"$STAMP" check "$WORK/b/board.bin" rom-y internal > /dev/null 2>&1 || bad "the matching request for Y was refused"

printf 'bitstream rebuilt behind the stamp\n' > "$WORK/a/board.bin"
out=$("$STAMP" check "$WORK/a/board.bin" '' '' 2>&1) && bad "a bitstream changed after it was stamped was accepted"
rm -f "$WORK/a/board.bin.stamp"
out=$("$STAMP" check "$WORK/a/board.bin" '' '' 2>&1) && bad "an unstamped bitstream was accepted"

# make prog end to end, flash command stubbed: a bare prog flashes what is built, a mismatched one flashes nothing.
printf 'bitstream from X\n' > "$WORK/a/board.bin"
"$STAMP" write "$WORK/a/board.bin" rom-x crystal "$WORK/one.hex"
printf '#!/bin/bash\necho "$@" >> "%s/flash.calls"\necho "VERIFY OK"\n' "$WORK" > "$WORK/iceprog"
chmod +x "$WORK/iceprog"
run_prog() { FLASH_LOG="$WORK/flash.log" PRIME_S=5 FLASH_S=10 make -s -C "$REPO" prog BUILD="$WORK/a" BOARD_FLASH="$WORK/iceprog" ICEPROG_SUDO= "$@" 2>&1; }

: > "$WORK/flash.calls"
out=$(run_prog BOARD_ROM=rom-y) && bad "make prog BOARD_ROM=rom-y flashed an image built from rom-x"
[ -s "$WORK/flash.calls" ] && bad "the flash command ran on a refused request"
[[ $out == *"rom-x"* && $out == *"rom-y"* ]] || bad "make prog's refusal does not name both: $out"
out=$(run_prog BOARD_ROM=rom-x) || bad "make prog BOARD_ROM=rom-x was refused: $out"
[ -s "$WORK/flash.calls" ] || bad "the matching request never reached the flash command"
: > "$WORK/flash.calls"
out=$(run_prog) || bad "a bare make prog was refused: $out"
[ -s "$WORK/flash.calls" ] || bad "a bare make prog never reached the flash command"

[ "$fail" = 0 ] && echo "board-stamp: ok"
exit "$fail"

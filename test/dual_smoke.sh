#!/bin/sh
# Builds test/dual/smoke.S once and runs it TWICE: with both harts, and with
# hart 1 held in reset. Both directions are graded, and the second is the point.
#
# A DUAL HARNESS THAT MEASURES ONE HART MEASURES NOTHING, and it does not look
# any different from a working one -- hart 0's program passes either way unless
# the answer depends on hart 1 having run. So the same program is run both ways
# and the shared count is required to MOVE: 2 * ITERS with two harts, ITERS with
# one. A harness whose second core was never clocked, never released from reset,
# or wired to the same instance as the first reports the same number twice and
# goes red here.
#
# The forward direction grades three things the reverse cannot: the program's
# own verdict, both harts' retire counts being non-zero, and the exit status
# being 0 rather than any of the runner's named failures.
#
# Usage: dual_smoke.sh <path-to-dual-sim>
set -eu

SIM=${1:-./dual-sim}
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/.." && pwd)
ASM_DIR="$REPO/test/asm"
CYCLES=${DUAL_CYCLES:-8000}
PROG="$HERE/dual/smoke.S"

# Both numbers this script grades come out of the program itself: the iteration
# count from its own `#define`, and the address of the counter from the linked
# ELF's symbol table. Neither is restated here, so adding a word to `.data` or
# changing the loop moves both without anything having to be kept in step -- and
# a copy of `ram`'s base is one restatement of the memory map that does not have
# to exist.
ITERS=$(sed -n 's/^#define[[:space:]][[:space:]]*ITERS[[:space:]][[:space:]]*\([0-9][0-9]*\).*/\1/p' \
          "$PROG" | head -1)

if [ ! -x "$SIM" ]; then
  echo "error: $SIM is not an executable -- run \`make dual-sim\` first." >&2
  exit 1
fi

CC=
for candidate in riscv64-elf-gcc riscv64-unknown-elf-gcc; do
  if command -v "$candidate" >/dev/null 2>&1; then CC=$candidate; break; fi
done
if [ -z "$CC" ]; then
  echo "error: no RISC-V cross compiler found; see \`make setup\`." >&2
  exit 1
fi
OBJCOPY=${CC%gcc}objcopy

tmp=$(mktemp -d "${TMPDIR:-/tmp}/dual-smoke.XXXXXX")
trap 'rm -rf "$tmp"' EXIT

# The same build an `.S` program in the suite gets: sections.lds, no crt0, and
# `.data` poked into RAM by the runner rather than copied by a startup.
"$CC" -march=rv32imac_zicsr_zifencei -mabi=ilp32 -nostdlib \
  -I "$ASM_DIR" -T "$ASM_DIR/sections.lds" \
  "$PROG" -o "$tmp/smoke.elf"
TOTAL_ADDR=$("${CC%gcc}nm" "$tmp/smoke.elf" | awk '$3 == "total" { print "0x" $1 }')

# Both reads have a silent failure mode -- an unmatched pattern is an empty
# string, and every comparison below would then compare against nothing and
# report a mismatch that names the wrong cause.
if [ -z "$ITERS" ]; then
  echo "error: no \`#define ITERS\` in $PROG, so this script does not know what to expect." >&2
  exit 1
fi
if [ -z "$TOTAL_ADDR" ]; then
  echo "error: $PROG links no \`total\` symbol, so there is no counter to read back." >&2
  exit 1
fi
"$OBJCOPY" -O verilog --verilog-data-width=4 --only-section=.text \
  "$tmp/smoke.elf" "$tmp/rom.hex"
"$OBJCOPY" -O verilog --verilog-data-width=4 --remove-section=.text \
  "$tmp/smoke.elf" "$tmp/ram.hex"

run() {
  # $1 = label, $2 = extra flag (may be empty)
  set +e
  # shellcheck disable=SC2086
  "$SIM" --rom "$tmp/rom.hex" --ram "$tmp/ram.hex" --cycles "$CYCLES" \
    --report-word "$TOTAL_ADDR" $2 > "$tmp/$1.out" 2> "$tmp/$1.err"
  status=$?
  set -e
  echo "== $1 (exit $status) =="
  cat "$tmp/$1.out" "$tmp/$1.err"
  return 0
}

fail() {
  echo "*** dual-smoke: $1" >&2
  exit 1
}

word_of() {
  awk '$1 == "WORD" { print $3 }' "$tmp/$1.out"
}

retires_of() {
  awk -v h="$1" '$1 == h && $2 == "RETIRES" { print $3 }' "$tmp/$2.out"
}

# ---- both harts -------------------------------------------------------------
run both ""
both_status=$status
[ "$both_status" -eq 0 ] || fail "the two-hart run exited $both_status, not 0"
grep -q '^PASS$' "$tmp/both.out" || fail "the two-hart run did not report PASS"

both_total=$(word_of both)
[ "$both_total" = "$((2 * ITERS))" ] || \
  fail "the two-hart run counted $both_total, not $((2 * ITERS))"

for h in HART0 HART1; do
  n=$(retires_of "$h" both)
  [ -n "$n" ] && [ "$n" -gt 0 ] || fail "$h retired nothing in the two-hart run"
done

# ---- hart 1 held in reset ---------------------------------------------------
run held "--hold-hart1"
held_status=$status

# 6 is the per-hart silence gate, and it is the RIGHT answer here: hart 1 really
# did observe nothing. Anything else means the gate did not fire -- including 0,
# which would mean a program that cannot tell one hart from two.
[ "$held_status" -eq 6 ] || \
  fail "the held run exited $held_status, not 6 (the per-hart silence gate)"

held_total=$(word_of held)
[ "$held_total" = "$ITERS" ] || \
  fail "the held run counted $held_total, not $ITERS -- the harness cannot tell one hart from two"

held_h1=$(retires_of HART1 held)
[ "$held_h1" = "0" ] || fail "hart 1 retired $held_h1 instructions while held in reset"

held_h0=$(retires_of HART0 held)
[ -n "$held_h0" ] && [ "$held_h0" -gt 0 ] || \
  fail "hart 0 retired nothing in the held run, so the run says nothing about hart 1"

echo
echo "dual-smoke: OK -- two harts counted $both_total, one hart counted $held_total"

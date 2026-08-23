#!/bin/bash
# Runs the .S suite on the FPGA, in batches, and grades the verdicts.
#
# WHY BATCHES. One program per bitstream is 71 flashes at about thirty seconds
# each. The ROM holds several at once, so test/board/board_suite.S runs a batch
# and reports each verdict over the UART; the flash count falls to the number of
# batches. Placement is re-run per batch rather than patched with icebram --
# nextpnr is ~40s and icebram would have to match two banks, so the simple thing
# is also the fast enough thing.
#
# WHAT IT DOES NOT ISOLATE. The programs share a machine: CSRs, mtvec and the
# reservation carry from one to the next, and only the registers and the stack
# are reset between them. Where a batched verdict disagrees with the simulated
# suite, the single-program build is the one to believe.
#
# NEEDS ROOT, for the reason `make prog` does: Apple's FTDI dext owns the
# FT232H's only interface and root takes it anyway.
set -uo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT"
export PATH="$HOME/.cache/little-cpu/oss-cad-suite/bin:$PATH"

BUDGET=${BUDGET:-7600}          # bytes of the 8192-byte ROM to fill per batch
READ_MS=${READ_MS:-8000}
FTREAD=${FTREAD:-/tmp/ftread}
OUT=$(mktemp -d "${TMPDIR:-/tmp}/suiteboard.XXXXXX")
trap 'rm -rf "$OUT"' EXIT

# rvc.S is 12256 bytes and does not fit an 8192-byte ROM even alone. That is a
# fact about the program and the part, not a thing to work around here.
SKIP="rvc.S"

echo "== sizing the programs"
sizes=""
for f in test/asm/*.S; do
  b=$(basename "$f")
  case " $SKIP " in *" $b "*) echo "   skip $b (larger than the ROM)"; continue;; esac
  # Sized as an OBJECT, not as a link: linking one program with the driver needs
  # board_table and board_count, which build_batch.sh generates per batch and
  # cannot exist here. The object's text+data is what the batch will carry.
  riscv64-elf-gcc -march=rv32imac_zicsr_zifencei -mabi=ilp32 -nostdlib -DBOARD_SUITE \
    -I test/asm -c -o "$OUT/one.o" "$f" 2>/dev/null || { echo "   skip $b (does not assemble)"; continue; }
  n=$(riscv64-elf-size "$OUT/one.o" | awk 'NR==2{print $1+$2}')
  sizes="$sizes$n $f"$'\n'
done

# Greedy pack, largest first, so a big program never strands a batch.
batches=0; cur=""; curn=0
: > "$OUT/plan"
while read -r n f; do
  [ -z "$f" ] && continue
  if [ $((curn + n)) -gt "$BUDGET" ] && [ -n "$cur" ]; then
    echo "$cur" >> "$OUT/plan"; batches=$((batches+1)); cur=""; curn=0
  fi
  cur="$cur $f"; curn=$((curn + n))
done < <(printf '%s' "$sizes" | sort -rn)
[ -n "$cur" ] && { echo "$cur" >> "$OUT/plan"; batches=$((batches+1)); }
echo "== $batches batches"

pass=0; fail=0; missing=0
: > "$OUT/results"
i=0
while read -r progs; do
  i=$((i+1))
  echo
  echo "== batch $i of $batches"
  ./test/board/build_batch.sh "$OUT/b$i" $progs || { echo "   BUILD FAILED"; continue; }
  rm -f board.json board.asc board.bin
  make board.bin BOARD_OSC=internal BOARD_ROM=noop-rom >"$OUT/build.log" 2>&1 || {
    tail -5 "$OUT/build.log"; echo "   PLACE FAILED"; continue; }
  iceprog board.bin >/dev/null 2>&1 || { echo "   FLASH FAILED"; continue; }
  raw=$("$FTREAD" 115200 "$READ_MS" 2>/dev/null)
  # The driver repeats; take the last complete block, the one between two dots.
  block=$(printf '%s' "$raw" | awk '/^\.$/{n++; next} {a[n]=a[n]$0"\n"} END{print a[n-1]}')
  [ -z "$block" ] && block=$(printf '%s' "$raw")
  j=0
  for p in $progs; do
    name=$(basename "$p")
    v=$(printf '%s' "$block" | awk -v k="$j" '$1==k{print $2; exit}')
    if [ -z "$v" ]; then
      echo "   $name: NO REPORT"; missing=$((missing+1)); echo "$name MISSING" >> "$OUT/results"
    elif [ "$v" = "1" ]; then
      pass=$((pass+1)); echo "$name PASS" >> "$OUT/results"
    else
      echo "   $name: FAIL verdict $v (test $(( v >> 1 )))"
      fail=$((fail+1)); echo "$name FAIL $(( v >> 1 ))" >> "$OUT/results"
    fi
    j=$((j+1))
  done
done < "$OUT/plan"

echo
echo "=================================================="
echo "on hardware: $pass passed, $fail failed, $missing no report"
echo "skipped (larger than the ROM): $SKIP"
echo
echo "baseline says these fail under simulation:"
grep -v '^#' test/EXPECTED_FAIL 2>/dev/null | grep -v '^$' || echo "(none)"
echo
sort "$OUT/results" > /tmp/suite_board_results.txt
echo "per-program results: /tmp/suite_board_results.txt"

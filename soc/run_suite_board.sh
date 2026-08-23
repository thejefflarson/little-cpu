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
# SHOW_RAW=1 dumps every byte the wire carried and the block that was parsed out
# of it, which is what to reach for when a verdict is missing: it separates "the
# board said nothing" from "the board said something this could not read".
#
# NEEDS ROOT, for the reason `make prog` does: Apple's FTDI dext owns the
# FT232H's only interface and root takes it anyway.
set -uo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT"
export PATH="$HOME/.cache/little-cpu/oss-cad-suite/bin:$PATH"

# How much of the 8192-byte ROM a batch's PROGRAMS may fill. Computed rather
# than guessed: the driver has to fit beside them, so does a four-byte table
# entry per program, and the per-program sizes below are measured on OBJECTS,
# which understate the linked result because the linker aligns each one. The
# margin covers that. build_batch.sh still refuses a batch that overflows, so
# this only decides how well packed the batches are, never whether they are
# correct.
DRIVER_BYTES=$(riscv64-elf-gcc -march=rv32imac_zicsr_zifencei -mabi=ilp32 -nostdlib \
                 -DBOARD_SUITE -I test/asm -c -o /tmp/.drv.$$.o test/board/board_suite.S 2>/dev/null \
               && riscv64-elf-size /tmp/.drv.$$.o | awk 'NR==2{print $1+$2}')
rm -f /tmp/.drv.$$.o
: "${DRIVER_BYTES:=512}"
BUDGET=${BUDGET:-$(( 8192 - DRIVER_BYTES - 600 ))}
READ_MS=${READ_MS:-8000}
FTREAD=${FTREAD:-$ROOT/ftread}
OUT=$(mktemp -d "${TMPDIR:-/tmp}/suiteboard.XXXXXX")
trap 'rm -rf "$OUT"' EXIT

# RESULTS ARE WRITTEN AS THEY ARRIVE, to a path that outlives this script. A
# ten-minute run that records nothing until its last line loses everything to
# any interruption -- which is exactly what happened the first time the whole
# suite ran, when the file was edited underneath a running bash and the
# interpreter resumed at a shifted offset. Sixty-seven verdicts went with it.
RESULTS=${RESULTS:-/tmp/suite_board_results.txt}
: > "$RESULTS"

# rvc.S is 12256 bytes and does not fit an 8192-byte ROM even alone. That is a
# fact about the program and the part, not a thing to work around here.
SKIP="rvc.S"

echo "== driver is ${DRIVER_BYTES} bytes; budgeting ${BUDGET} per batch of the 8192-byte ROM"
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
echo
i=0
while read -r progs; do
  i=$((i+1))
  n=$(printf '%s' "$progs" | wc -w | tr -d ' ')
  printf '   batch %d: %2d programs --' "$i" "$n"
  for p in $progs; do printf ' %s' "$(basename "$p" .S)"; done
  printf '\n'
done < "$OUT/plan"

pass=0; fail=0; missing=0
i=0
while read -r progs; do
  i=$((i+1))
  echo
  echo "== batch $i of $batches"
  echo "   programs:$(for p in $progs; do printf ' %s' "$(basename "$p" .S)"; done)"

  t0=$SECONDS
  if ! ./test/board/build_batch.sh "$OUT/b$i" $progs > "$OUT/link.log" 2>&1; then
    echo "   LINK FAILED:"; sed 's/^/      /' "$OUT/link.log" | tail -12; continue
  fi
  echo "   link:  $(tail -1 "$OUT/link.log")  [$((SECONDS-t0))s]"

  t0=$SECONDS
  rm -f board.json board.asc board.bin
  if ! make board.bin BOARD_OSC=internal BOARD_ROM=noop-rom >"$OUT/build.log" 2>&1; then
    echo "   PLACE FAILED:"; tail -12 "$OUT/build.log" | sed 's/^/      /'; continue
  fi
  echo "   place: $(grep -o 'board.bin: [0-9]* bytes.*' "$OUT/build.log" | head -1)  [$((SECONDS-t0))s]"

  # iceprog's own output, live and unfiltered. It takes about thirty seconds and
  # prints its progress as it goes, so hiding it makes a working flash
  # indistinguishable from a hung one -- which is exactly how it looked the first
  # time this ran quietly.
  t0=$SECONDS
  echo "   flashing (iceprog, ~30s):"
  if ! iceprog board.bin 2>&1 | tee "$OUT/flash.log" | sed 's/^/      | /'; then
    echo "   FLASH FAILED"; continue
  fi
  grep -q 'VERIFY OK' "$OUT/flash.log" || { echo "   FLASH DID NOT VERIFY"; continue; }
  echo "   flash: VERIFY OK  [$((SECONDS-t0))s]"

  t0=$SECONDS
  raw=$("$FTREAD" 115200 "$READ_MS" 2>"$OUT/read.err")
  nbytes=$(sed -E 's/.*bytes=([0-9]+).*/\1/' < "$OUT/read.err" | tr -d '\n')
  echo "   read:  ${nbytes:-0} bytes in $((SECONDS-t0))s"

  # The batch runs ONCE and then repeats a lone '.' forever, so everything before
  # the first dot is the run. Taking a later block was the bug that made four
  # programs look broken: the driver used to re-run the batch, and text and CSRs
  # do not survive a second pass.
  block=$(printf '%s' "$raw" | awk '/^\.$/{exit} {print}')
  if [ -z "$block" ]; then
    echo "   (nothing before a '.' marker -- parsing the whole capture)"
    block=$(printf '%s' "$raw")
  fi

  if [ -n "${SHOW_RAW:-}" ]; then
    echo "   ---- raw capture ----"; printf '%s' "$raw" | sed 's/^/      /'
    echo "   ---- parsed block ----"; printf '%s' "$block" | sed 's/^/      /'
  else
    echo "   verdicts: $(printf '%s' "$block" | tr '\n' ' ' | cut -c1-70)"
  fi

  j=0
  for p in $progs; do
    name=$(basename "$p")
    v=$(printf '%s' "$block" | awk -v k="$j" '$1==k{print $2; exit}')
    if [ -z "$v" ]; then
      printf '      %-18s NO REPORT\n' "$name"; missing=$((missing+1)); echo "$name MISSING" >> "$RESULTS"
    elif [ "$v" = "1" ]; then
      printf '      %-18s pass\n' "$name"; pass=$((pass+1)); echo "$name PASS" >> "$RESULTS"
    else
      printf '      %-18s FAIL at test %d (verdict %s)\n' "$name" "$(( v >> 1 ))" "$v"
      fail=$((fail+1)); echo "$name FAIL $(( v >> 1 ))" >> "$RESULTS"
    fi
    j=$((j+1))
  done
  echo "   running total: $pass pass, $fail fail, $missing no report"
done < "$OUT/plan"

echo
echo "=================================================="
echo "on hardware: $pass passed, $fail failed, $missing no report"
echo "skipped (larger than the ROM): $SKIP"
echo
echo "baseline says these fail under simulation:"
grep -v '^#' test/EXPECTED_FAIL 2>/dev/null | grep -v '^$' || echo "(none)"
echo
echo "per-program results, written as they arrived: $RESULTS"
echo
echo "failures:"
grep -v ' PASS$' "$RESULTS" | sed 's/^/   /' || echo "   (none)"

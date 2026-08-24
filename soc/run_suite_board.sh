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
# Every raw capture, kept. Diagnosing a missing verdict without the bytes means
# re-running the suite, and the suite takes minutes.
RAWDIR=${RAWDIR:-/tmp/suite_board_raw}
rm -rf "$RAWDIR"; mkdir -p "$RAWDIR"

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

# ONE PLACEMENT FOR THE WHOLE RUN. The design is identical across batches --
# only the ROM's contents differ -- so nextpnr runs once here and icebram
# rewrites the ROM per batch. Against `icebram -g` random data, so the pattern
# is unique enough to find again.
echo
echo "== placing once (the design does not change between batches)"
icebram -g 32 1024 > "$OUT/ph_even.hex"
icebram -g 32 1024 > "$OUT/ph_odd.hex"
cp "$OUT/ph_even.hex" soc/rom_even.hex
cp "$OUT/ph_odd.hex" soc/rom_odd.hex
rm -f board.json board.asc board.bin
if ! make board.asc BOARD_OSC=internal BOARD_ROM=noop-rom >"$OUT/place.log" 2>&1; then
  echo "PLACE FAILED:"; tail -15 "$OUT/place.log" | sed 's/^/   /'; exit 1
fi
cp board.asc "$OUT/base.asc"
echo "   placed [$SECONDS s]"
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

  # SWAPPED, NOT RE-PLACED. Placement is 60s and everything else in a batch is
  # seconds, so re-running nextpnr seven times was seven eighths of the runtime
  # for a design that never changes -- only its ROM does. icebram rewrites a
  # block RAM's contents inside an already-placed .asc.
  #
  # It needs the placed image to carry a UNIQUE pattern, which is why the
  # placement above is done against `icebram -g` random data rather than against
  # a real program: a zero-padded ROM appears identically in both banks and
  # icebram refuses it -- "Conflicting from pattern for bit slice" -- because it
  # cannot tell which bank it is being asked to rewrite.
  t0=$SECONDS
  if ! icebram "$OUT/ph_even.hex" soc/rom_even.hex < "$OUT/base.asc" > "$OUT/b0.asc" 2>"$OUT/ib.log" \
     || ! icebram "$OUT/ph_odd.hex" soc/rom_odd.hex < "$OUT/b0.asc" > "$OUT/b1.asc" 2>>"$OUT/ib.log"; then
    echo "   ROM SWAP FAILED:"; sed 's/^/      /' "$OUT/ib.log" | head -6; continue
  fi
  icepack "$OUT/b1.asc" board.bin || { echo "   PACK FAILED"; continue; }
  echo "   swap:  $(wc -c < board.bin | tr -d ' ') bytes, no re-placement  [$((SECONDS-t0))s]"

  # iceprog's own output, live and unfiltered. It takes about thirty seconds and
  # prints its progress as it goes, so hiding it makes a working flash
  # indistinguishable from a hung one -- which is exactly how it looked the first
  # time this ran quietly.
  # RETRIED, BECAUSE THE UART USED TO FIGHT THE FLASH FOR PIN 14.
  #
  # soc/upduino.pcf used to put uart_tx on pin 14, and the vendor's own
  # constraint file calls that pin spi_miso -- the flash's data line into the
  # FPGA. Once a batch
  # has run, the driver replays its report forever, so the FPGA is driving that
  # pin while iceprog is trying to read the flash through it. The symptom is
  # exact: `cdone: high` after reset instead of low, a flash ID that comes back
  # with a spurious leading 0xFF, and then a write error. The first flash of a
  # session works because the board is still quiet.
  #
  # A retry usually wins -- the contention is a race against the replay's duty
  # cycle, not a permanent conflict. The design should not need one any more:
  # soc/board_upduino.v releases pin 14 whenever the host asserts the flash's
  # chip select on pin 16, so a bitstream built since that landed does not drive
  # the wire while iceprog reads it. The loop stays until a board has actually
  # run it -- it costs nothing when the first attempt wins, and removing it on
  # an argument rather than a run is how this project has been wrong before.
  t0=$SECONDS
  flashed=""
  for attempt in 1 2 3 4; do
    echo "   flashing (iceprog, attempt $attempt):"
    if iceprog board.bin 2>&1 | tee "$OUT/flash.log" | sed 's/^/      | /' \
       && grep -q 'VERIFY OK' "$OUT/flash.log"; then
      flashed=yes; break
    fi
    if grep -q 'unexpected rx byte\|Write error' "$OUT/flash.log"; then
      echo "   (the running design is driving pin 14, the flash's MISO -- retrying)"
    fi
    sleep 2
  done
  [ -n "$flashed" ] || { echo "   FLASH FAILED after 4 attempts"; continue; }
  echo "   flash: VERIFY OK  [$((SECONDS-t0))s]"

  # RETRIED UNTIL THE BLOCK IS WHOLE, and every capture kept. A short read is
  # indistinguishable from a batch that did not run unless the bytes are on
  # disk to look at, and re-running the whole suite to see them costs minutes.
  # The board replays forever, so another read is nearly free.
  t0=$SECONDS
  want=$(printf '%s' "$progs" | wc -w | tr -d ' ')
  for attempt in 1 2 3; do
    raw=$("$FTREAD" 115200 "$READ_MS" 2>"$OUT/read.err")
    printf '%s' "$raw" > "$RAWDIR/batch$i.attempt$attempt.txt"
    nbytes=$(sed -E 's/.*bytes=([0-9]+).*/\1/' < "$OUT/read.err" | tr -d '\n')
    block=$(printf '%s' "$raw" | awk '/^\.$/{n++; next} {a[n]=a[n]$0"\n"} END{print a[n-1]}')
    got=$(printf '%s' "$block" | grep -c '^[0-9]' || true)
    echo "   read:  ${nbytes:-0} bytes, $got of $want verdicts (attempt $attempt) [$((SECONDS-t0))s]"
    [ "$got" -ge "$want" ] && break
  done

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
echo "raw UART captures, one file per batch and attempt: $RAWDIR"
echo
echo "failures:"
grep -v ' PASS$' "$RESULTS" | sed 's/^/   /' || echo "   (none)"

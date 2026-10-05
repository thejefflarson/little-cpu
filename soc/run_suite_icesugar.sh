#!/bin/bash
# Runs the .S suite on the iCESugar-Pro, in batches, and grades the verdicts: place once, then per batch swap the
# ROM contents into the placed configuration, load SRAM over JTAG (never the flash) and read the CDC UART.
set -uo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT"
. "$ROOT/soc/board_verdict.sh"
LOADER=${ICESUGAR_LOADER:-openFPGALoader}
VID=${ICESUGAR_VID:-0x1d50}
PID=${ICESUGAR_PID:-0x602b}
CACHE=${XDG_CACHE_HOME:-$HOME/.cache}/little-cpu
export PATH="$CACHE/riscv-gcc/bin:$CACHE/oss-cad-suite/bin:$PATH"

mkdir -p build
DRIVER_BYTES=$(riscv-none-elf-gcc -march=rv32imac_zicsr_zifencei_zkt_zkr -mabi=ilp32 -nostdlib \
                 -DBOARD_SUITE -I test/asm -c -o build/.drv.$$.o test/board/board_suite.S 2>/dev/null \
               && riscv-none-elf-size build/.drv.$$.o | awk 'NR==2{print $1+$2}')
rm -f build/.drv.$$.o
: "${DRIVER_BYTES:=512}"
BUDGET=${BUDGET:-$(( 8192 - DRIVER_BYTES - 600 ))}
READ_S=${READ_S:-3}
OUT=$(mktemp -d "${TMPDIR:-/tmp}/suiteice.XXXXXX")
trap 'rm -rf "$OUT"' EXIT

RESULTS=${RESULTS:-build/suite_icesugar_results.txt}
: > "$RESULTS"
RAWDIR=${RAWDIR:-build/suite_icesugar_raw}
rm -rf "$RAWDIR"; mkdir -p "$RAWDIR"

# rvc.S is 12256 bytes and does not fit an 8192-byte ROM even alone.
SKIP="rvc.S"

echo "== driver is ${DRIVER_BYTES} bytes; budgeting ${BUDGET} per batch of the 8192-byte ROM"
sizes=""
for f in test/asm/*.S; do
  b=$(basename "$f")
  case " $SKIP " in *" $b "*) echo "   skip $b (larger than the ROM)"; continue;; esac
  riscv-none-elf-gcc -march=rv32imac_zicsr_zifencei_zkt_zkr -mabi=ilp32 -nostdlib -DBOARD_SUITE \
    -I test/asm -c -o "$OUT/one.o" "$f" 2>/dev/null || { echo "   skip $b (does not assemble)"; continue; }
  n=$(riscv-none-elf-size "$OUT/one.o" | awk 'NR==2{print $1+$2}')
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
echo "== placing once (the design does not change between batches)"
ecpbram -g "$OUT/ph_even.hex" -w 32 -d 1024 -s 1
ecpbram -g "$OUT/ph_odd.hex" -w 32 -d 1024 -s 2
cp "$OUT/ph_even.hex" soc/rom_even.hex
cp "$OUT/ph_odd.hex" soc/rom_odd.hex
rm -f build/icesugar.json build/icesugar.config build/icesugar.bit
if ! make build/icesugar.config ICESUGAR_ROM=noop-rom >"$OUT/place.log" 2>&1; then
  echo "PLACE FAILED:"; tail -15 "$OUT/place.log" | sed 's/^/   /'; exit 1
fi
cp build/icesugar.config "$OUT/base.config"
echo "   placed [$SECONDS s]"

pass=0; fail=0; missing=0
i=0
while read -r progs; do
  i=$((i+1))
  want=$(printf '%s' "$progs" | wc -w | tr -d ' ')
  echo
  echo "== batch $i of $batches"
  echo "   programs:$(for p in $progs; do printf ' %s' "$(basename "$p" .S)"; done)"

  t0=$SECONDS
  if ! ./test/board/build_batch.sh "$OUT/b$i" $progs > "$OUT/link.log" 2>&1; then
    echo "   LINK FAILED:"; sed 's/^/      /' "$OUT/link.log" | tail -12
    for p in $progs; do echo "$(basename "$p") LINK-FAILED" >> "$RESULTS"; missing=$((missing+1)); done
    continue
  fi
  echo "   link:  $(tail -1 "$OUT/link.log")  [$((SECONDS-t0))s]"

  if ! ecpbram -i "$OUT/base.config" -o "$OUT/b0.config" -f "$OUT/ph_even.hex" -t soc/rom_even.hex >"$OUT/eb.log" 2>&1 \
     || ! ecpbram -i "$OUT/b0.config" -o "$OUT/b1.config" -f "$OUT/ph_odd.hex" -t soc/rom_odd.hex >>"$OUT/eb.log" 2>&1 \
     || ! ecppack "$OUT/b1.config" build/icesugar.bit >>"$OUT/eb.log" 2>&1; then
    echo "   ROM SWAP FAILED:"; sed 's/^/      /' "$OUT/eb.log" | head -6
    for p in $progs; do echo "$(basename "$p") SWAP-FAILED" >> "$RESULTS"; missing=$((missing+1)); done
    continue
  fi

  loaded=""
  for attempt in 1 2 3; do
    if "$LOADER" -c cmsisdap --vid "$VID" --pid "$PID" -m build/icesugar.bit >"$OUT/load.log" 2>&1; then loaded=yes; break; fi
    sleep 2
  done
  if [ -z "$loaded" ]; then
    echo "   LOAD FAILED:"; display_safe < "$OUT/load.log" | tail -6 | sed 's/^/      /'
    for p in $progs; do echo "$(basename "$p") LOAD-FAILED" >> "$RESULTS"; missing=$((missing+1)); done
    continue
  fi
  echo "   load:  SRAM over JTAG  [$((SECONDS-t0))s]"

  for attempt in 1 2 3; do
    python3 soc/board_read.py --seconds "$READ_S" --out "$RAWDIR/batch$i.attempt$attempt.txt" >/dev/null 2>&1
    raw=$(cat "$RAWDIR/batch$i.attempt$attempt.txt" 2>/dev/null)
    block=$(printf '%s' "$raw" | uart_last_block)
    got=$(printf '%s' "$block" | grep -c '^[0-9]' || true)
    echo "   read:  $got of $want verdicts (attempt $attempt)"
    [ "$got" -ge "$want" ] && break
  done

  if [ -n "${SHOW_RAW:-}" ]; then
    echo "   ---- raw capture ----"; printf '%s' "$raw" | display_safe | sed 's/^/      /'
    echo "   ---- parsed block ----"; printf '%s' "$block" | display_safe | sed 's/^/      /'
  else
    echo "   verdicts: $(printf '%s' "$block" | display_safe | tr '\n' ' ' | cut -c1-70)"
  fi

  j=0
  for p in $progs; do
    name=$(basename "$p")
    line=$(result_line "$name" "$(block_verdict "$block" "$j")")
    echo "$line" >> "$RESULTS"
    case $line in
      *" PASS") printf '      %-18s pass\n' "$name"; pass=$((pass+1));;
      *" FAIL "*) printf '      %-18s %s\n' "$name" "${line#"$name "}"; fail=$((fail+1));;
      *) printf '      %-18s %s\n' "$name" "${line#"$name "}"; missing=$((missing+1));;
    esac
    j=$((j+1))
  done
  echo "   running total: $pass pass, $fail fail, $missing no report"
done < "$OUT/plan"

echo
echo "=================================================="
echo "on hardware: $pass passed, $fail failed, $missing no report"
echo "skipped (larger than the ROM): $SKIP"

# Grade against the simulation baselines: a failure the baseline does not name, a baselined program that passed, and a
# floor program that never ran are each red.
expected=$(grep -v '^#' test/EXPECTED_FAIL 2>/dev/null | awk 'NF{print $1}' | sort)
unexpected=""; unexpected_pass=""
while read -r name status; do
  [ -z "$name" ] && continue
  baselined=$(printf '%s\n' "$expected" | grep -Fx -- "$name" || true)
  if [ "$status" = PASS ]; then
    [ -n "$baselined" ] && unexpected_pass="$unexpected_pass $name"
  elif [ -z "$baselined" ]; then
    unexpected="$unexpected $name"
  fi
done < "$RESULTS"
never_ran=""
for n in $(awk '/\.S[[:space:]]/{print $1}' test/OBSERVED_FLOOR); do
  case " $SKIP " in *" $n "*) continue;; esac
  grep -q "^$n " "$RESULTS" || never_ran="$never_ran $n"
done
echo "failed on the board and not baselined:${unexpected:- (none)}"
echo "baselined to fail and passed on the board:${unexpected_pass:- (none)}"
echo "in test/OBSERVED_FLOOR and never run:${never_ran:- (none)}"
echo "per-program results, written as they arrived: $RESULTS"
echo "raw UART captures, one file per batch and attempt: $RAWDIR"
[ -z "$unexpected$unexpected_pass$never_ran" ]

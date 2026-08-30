#!/bin/sh
# Runs BOTH factors of the cross-core throughput product for every benchmark
# pair this repo knows, and writes the result into soc/compare/product.json --
# collapsing "sweep the clock, run the cycle count, do the arithmetic by hand,
# edit CLAUDE.md" into one command.
#
#   make compare-product
#   COMPARE_PRODUCT_SEEDS='default 1 2 3' make compare-product   # a shorter look
#
# Twelve seeds a side is this file's own convention for a verdict (the same
# floor soc/baseline_summary.py's callers use for `make soc-timing`), so the
# default sweeps twelve placements for littlecpu and twelve more for each
# other core a pair needs -- roughly half an hour total for the Dhrystone
# pair alone. A shorter COMPARE_PRODUCT_SEEDS is a look, not a verdict, the same
# distinction soc/compare/sweep.sh's own header states.
#
# A BENCHMARK NAMES A LIST OF CORES, NOT A FIXED PAIR -- soc/compare/product_write.py's
# header has the reasoning. Dhrystone here still measures littlecpu against one
# other core (VexRiscv) and CoreMark against one other (Hazard3, because the
# pinned VexRiscv build has no M extension and cannot execute CoreMark's
# RV32IMA image), but the artifact does not encode "exactly two": a benchmark
# whose image later runs on three cores is one more `--clock-ns`/
# `--cycle-factor` pair to this script, not a schema change.
#
# THE COREMARK PAIR IS FEATURE-DETECTED, NOT ASSUMED. `make compare-coremark`
# and soc/compare/coremark_dmips.py do not exist on every tree this script
# runs on -- as of this writing they are still on a held pull request. Where
# they are absent this script records CoreMark as "not yet measured" and moves
# on; where they exist it attempts the same measurement Dhrystone gets, and
# falls back to "not yet measured" with a diagnostic rather than aborting the
# whole run if the attempt does not match the shape this script expects. That
# fallback is temporary scaffolding for one PR's worth of drift, not a
# permanent hedge: once `make compare-coremark` lands for real, a run that
# still falls back is a bug in measure_coremark() below to fix, not a
# steady-state outcome to keep tolerating.
#
# WHAT THIS DOES NOT DO. It does not touch CLAUDE.md or any ADR -- those stay a
# person's sentences about a tree they read, and this script's whole job is to
# give that person one place the numbers actually came from rather than a
# second place to keep in sync by hand. It is not on `make test`'s path and
# adds no ratchet, the same as every other `make compare-*` target.
set -eu

cd "$(dirname "$0")/../.."

SEEDS=${COMPARE_PRODUCT_SEEDS:-"default 1 2 3 4 5 6 7 8 9 10 11"}
if [ -z "$SEEDS" ]; then
  echo "*** run_product.sh: COMPARE_PRODUCT_SEEDS is empty, so nothing would be" >&2
  echo "*** placed. Name the seeds, or unset it for the default twelve." >&2
  exit 2
fi
OUT=${COMPARE_PRODUCT_OUT:-soc/compare/product.json}

BASE=$(git rev-parse HEAD)
if git diff --quiet HEAD --; then DIRTY=no; else DIRTY=yes; fi
DATE=$(date -u '+%Y-%m-%dT%H:%M:%SZ')
ROM_WORDS=$(make -s print-COMPARE_ROM_WORDS)
RAM_WORDS=$(make -s print-COMPARE_RAM_WORDS)

CC=""
for candidate in riscv64-elf-gcc riscv64-unknown-elf-gcc; do
  if command -v "$candidate" >/dev/null 2>&1; then CC=$candidate; break; fi
done
if [ -z "$CC" ]; then
  echo "*** run_product.sh: no RISC-V cross compiler found; see \`make setup\`." >&2
  exit 1
fi

TOOLS_BLOCK=$(soc/print_toolchain.sh yosys nextpnr-ice40 icetime iverilog "$CC")
TOOL_ARGS=""
while IFS= read -r line; do
  [ -z "$line" ] && continue
  name=${line#\# }; name=${name%%:*}
  value=${line#*: }
  # `set --` below re-splits on whitespace, so the value's own spaces (every
  # one of these carries a version string with one in it) travel quoted.
  TOOL_ARGS="$TOOL_ARGS --tool"
  TOOL_ARGS="$TOOL_ARGS '$name=$value'"
done <<TOOLS
$TOOLS_BLOCK
TOOLS

# The bare -march= value out of a CFLAGS string, for --isa: three cores sharing
# one image may share a narrower ISA than any one core implements alone, so
# this is stamped on its own rather than left for a reader to pick back out of
# the full flag string.
isa_from_cflags() {  # $1 = CFLAGS string
  printf '%s\n' "$1" | sed -n 's/.*-march=\([A-Za-z0-9_]*\).*/\1/p'
}

# `make compare-timing` places $1 at $2 (empty for nextpnr's own default seed)
# and reports the critical path on stdout -- soc/compare/sweep.sh's own
# extraction, repeated here rather than shared, because this script also needs
# `make` NOT in a pipeline for the reason both files already give: the default
# shell is errexit without pipefail, so a graded command piped into `grep`
# hands back grep's status and a failed placement would print nothing and exit
# 0.
sweep_clock() {  # $1 = core; prints comma-separated nanoseconds on stdout
  core=$1
  ns_csv=""
  for seed in $SEEDS; do
    case $seed in
      default) arg="" ;;
      *)       arg=$seed ;;
    esac
    if ! out=$(make compare-timing COMPARE_CORE="$core" COMPARE_SEED="$arg" 2>&1); then
      echo "*** run_product.sh: $core seed '$seed' failed to place; the run" >&2
      echo "*** stops here. That is a failed placement, not a fast design." >&2
      printf '%s\n' "$out" >&2
      exit 1
    fi
    line=$(printf '%s\n' "$out" | grep '^critical path :') || {
      echo "*** run_product.sh: $core seed '$seed' exited 0 with no critical" >&2
      echo "*** path line, which soc/timing_split.py is supposed to make" >&2
      echo "*** impossible." >&2
      exit 1
    }
    ns=$(printf '%s\n' "$line" | sed 's/^critical path : \([0-9.]*\) ns.*/\1/')
    ns_csv="${ns_csv:+$ns_csv,}$ns"
  done
  printf '%s' "$ns_csv"
}

echo "== compare-product: littlecpu clock ($SEEDS) =="
LC_NS=$(sweep_clock littlecpu)
echo "$LC_NS"

echo
echo "== compare-product: Dhrystone (littlecpu against VexRiscv) =="
echo "== VexRiscv clock =="
VEX_NS=$(sweep_clock vexriscv)
echo "$VEX_NS"

echo "== Dhrystone cycles =="
if ! DHRY_OUT=$(make compare-dhrystone 2>&1); then
  echo "*** run_product.sh: make compare-dhrystone failed." >&2
  printf '%s\n' "$DHRY_OUT" >&2
  exit 1
fi
printf '%s\n' "$DHRY_OUT"

LC_CYCLES=$(printf '%s\n' "$DHRY_OUT" | grep '^DHRY core=littlecpu' | sed -n 's/.*cycles=\([0-9]*\).*/\1/p')
VEX_CYCLES=$(printf '%s\n' "$DHRY_OUT" | grep '^DHRY core=vexriscv' | sed -n 's/.*cycles=\([0-9]*\).*/\1/p')
if [ -z "$LC_CYCLES" ] || [ -z "$VEX_CYCLES" ]; then
  echo "*** run_product.sh: could not find both cores' 'DHRY core=... cycles='" >&2
  echo "*** lines in make compare-dhrystone's output." >&2
  exit 1
fi
DHRY_RUNS=$(make -s print-COMPARE_DHRY_RUNS)
DHRY_CFLAGS=$(make -s print-COMPARE_DHRY_CFLAGS)
DHRY_ISA=$(isa_from_cflags "$DHRY_CFLAGS")

# DMIPS/MHz: runs*1e6/cycles, divided by the VAX 11/780 rate --
# soc/compare/dhry_dmips.py's own constant, imported rather than restated so
# the two files cannot state two different rates.
DHRY_VAX_RATE=$(python3 -c "import sys; sys.path.insert(0, 'soc/compare'); \
  from dhry_dmips import VAX_DHRYSTONES_PER_SEC; print(VAX_DHRYSTONES_PER_SEC)")
LC_DHRY_FACTOR=$(python3 -c "print($DHRY_RUNS * 1e6 / $LC_CYCLES / $DHRY_VAX_RATE)")
VEX_DHRY_FACTOR=$(python3 -c "print($DHRY_RUNS * 1e6 / $VEX_CYCLES / $DHRY_VAX_RATE)")

eval "set -- $TOOL_ARGS"
python3 soc/compare/product_write.py "$OUT" dhrystone --measured \
  --target-core littlecpu --base "$BASE" --dirty "$DIRTY" --date "$DATE" \
  --seeds "$SEEDS" --cflags "$DHRY_CFLAGS" --isa "$DHRY_ISA" \
  --rom-words "$ROM_WORDS" --ram-words "$RAM_WORDS" --unit 'DMIPS/MHz' "$@" \
  --clock-ns "littlecpu=$LC_NS" --clock-ns "vexriscv=$VEX_NS" \
  --cycle-factor "littlecpu=$LC_DHRY_FACTOR" --cycle-factor "vexriscv=$VEX_DHRY_FACTOR"

# Attempts the CoreMark pair exactly the way Dhrystone was measured above, and
# returns 1 -- never aborts the script -- if make compare-coremark's output does
# not match what this function expects, so a wrong guess about an interface this
# script cannot see yet degrades to "not yet measured" instead of failing the
# whole run. Once `make compare-coremark` is real, a run that still falls back
# here is a bug in this function to fix, not a steady state to keep tolerating.
measure_coremark() {
  if HZ_NS=$(sweep_clock hazard3) \
     && CM_OUT=$(make compare-coremark 2>&1) \
     && LC_CM_CYCLES=$(printf '%s\n' "$CM_OUT" | grep '^COREMARK core=littlecpu' | sed -n 's/.*cycles=\([0-9]*\).*/\1/p') \
     && HZ_CM_CYCLES=$(printf '%s\n' "$CM_OUT" | grep '^COREMARK core=hazard3' | sed -n 's/.*cycles=\([0-9]*\).*/\1/p') \
     && CM_ITERATIONS=$(printf '%s\n' "$CM_OUT" | grep '^COREMARK core=littlecpu' | sed -n 's/.*iterations=\([0-9]*\).*/\1/p') \
     && CM_CFLAGS=$(make -s print-COMPARE_COREMARK_CFLAGS) \
     && [ -n "$LC_CM_CYCLES" ] && [ -n "$HZ_CM_CYCLES" ] \
     && [ -n "$CM_ITERATIONS" ] && [ -n "$CM_CFLAGS" ]; then
    printf '%s\n' "$CM_OUT"
    LC_CM_FACTOR=$(python3 -c "print($CM_ITERATIONS * 1e6 / $LC_CM_CYCLES)")
    HZ_CM_FACTOR=$(python3 -c "print($CM_ITERATIONS * 1e6 / $HZ_CM_CYCLES)")
    CM_ISA=$(isa_from_cflags "$CM_CFLAGS")
    eval "set -- $TOOL_ARGS"
    python3 soc/compare/product_write.py "$OUT" coremark --measured \
      --target-core littlecpu --base "$BASE" --dirty "$DIRTY" --date "$DATE" \
      --seeds "$SEEDS" --cflags "$CM_CFLAGS" --isa "$CM_ISA" \
      --rom-words "$ROM_WORDS" --ram-words "$RAM_WORDS" --unit 'CoreMark/MHz' "$@" \
      --clock-ns "littlecpu=$LC_NS" --clock-ns "hazard3=$HZ_NS" \
      --cycle-factor "littlecpu=$LC_CM_FACTOR" --cycle-factor "hazard3=$HZ_CM_FACTOR"
    return 0
  fi
  echo "*** run_product.sh: make compare-coremark's output did not match the" >&2
  echo "*** 'COREMARK core=... cycles=... iterations=...' shape this script" >&2
  echo "*** expects, or COMPARE_COREMARK_CFLAGS is unset. Recording CoreMark" >&2
  echo "*** as not yet measured; update measure_coremark() in" >&2
  echo "*** soc/compare/run_product.sh to match what landed." >&2
  return 1
}

echo
echo "== compare-product: CoreMark (littlecpu against Hazard3) =="
COREMARK_OK=0
if grep -q '^compare-coremark:' Makefile && [ -f soc/compare/coremark_dmips.py ]; then
  echo "make compare-coremark is on this tree; attempting the measurement."
  measure_coremark && COREMARK_OK=1
fi
if [ "$COREMARK_OK" -eq 0 ]; then
  if grep -q '^compare-coremark:' Makefile; then
    REASON="make compare-coremark exists on this tree but its output did not match what run_product.sh expects; see the warning above"
  else
    REASON="make compare-coremark is not on this tree yet; run_product.sh will measure it once that lands"
  fi
  python3 soc/compare/product_write.py "$OUT" coremark --not-yet-measured \
    --target-core littlecpu --core hazard3 --reason "$REASON"
fi

echo
echo "== $OUT =="
python3 soc/compare/product_check.py "$OUT" dhrystone --repo . \
  --current "cflags=$DHRY_CFLAGS" --current "rom_words=$ROM_WORDS" \
  --current "ram_words=$RAM_WORDS"
python3 soc/compare/product_check.py "$OUT" coremark --repo .

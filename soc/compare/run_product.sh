#!/bin/sh
# Runs BOTH factors of the cross-core throughput product for every benchmark pair this
# repo knows, and writes the result into soc/compare/product.json --collapsing "sweep the
# clock, run the cycle count, do the arithmetic by hand, edit CLAUDE.md" into one
# command.
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
  # `set --` below re-splits on whitespace, so the value's own spaces (every one of these
  # carries a version string with one in it) travel quoted.
  TOOL_ARGS="$TOOL_ARGS --tool"
  TOOL_ARGS="$TOOL_ARGS '$name=$value'"
done <<TOOLS
$TOOLS_BLOCK
TOOLS

isa_from_cflags() {  # $1 = CFLAGS string
  printf '%s\n' "$1" | sed -n 's/.*-march=\([A-Za-z0-9_]*\).*/\1/p'
}

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

# FIRST match only. `make compare-dhrystone` prints a fourth row -- this core alone at
# its native ISA, so the shared subset's cost is a number.
LC_CYCLES=$(printf '%s\n' "$DHRY_OUT" | grep '^DHRY core=littlecpu' | sed -n 's/.*cycles=\([0-9]*\).*/\1/p' | head -1)
VEX_CYCLES=$(printf '%s\n' "$DHRY_OUT" | grep '^DHRY core=vexriscv' | sed -n 's/.*cycles=\([0-9]*\).*/\1/p' | head -1)
if [ -z "$LC_CYCLES" ] || [ -z "$VEX_CYCLES" ]; then
  echo "*** run_product.sh: could not find both cores' 'DHRY core=... cycles='" >&2
  echo "*** lines in make compare-dhrystone's output." >&2
  exit 1
fi
DHRY_RUNS=$(make -s print-COMPARE_DHRY_RUNS)
DHRY_CFLAGS=$(make -s print-COMPARE_DHRY_CFLAGS)
DHRY_ISA=$(isa_from_cflags "$DHRY_CFLAGS")

DHRY_VAX_RATE=$(python3 -c "import sys; sys.path.insert(0, 'soc/compare'); \
  from dhry_dmips import VAX_DHRYSTONES_PER_SEC; print(VAX_DHRYSTONES_PER_SEC)")

for pair in "DHRY_RUNS=$DHRY_RUNS" "LC_CYCLES=$LC_CYCLES" \
            "VEX_CYCLES=$VEX_CYCLES" "DHRY_VAX_RATE=$DHRY_VAX_RATE"; do
  name=${pair%%=*}; value=${pair#*=}
  case "$value" in
    ''|*[!0-9.]*)
      echo "*** run_product.sh: $name is '$value', which is not a number." >&2
      echo "*** It is interpolated into a python3 -c expression below, so a" >&2
      echo "*** blank or multi-line value there dies as a SyntaxError instead" >&2
      echo "*** of naming itself. Fix what produced it." >&2
      exit 1 ;;
  esac
done

LC_DHRY_FACTOR=$(python3 -c "print($DHRY_RUNS * 1e6 / $LC_CYCLES / $DHRY_VAX_RATE)")
VEX_DHRY_FACTOR=$(python3 -c "print($DHRY_RUNS * 1e6 / $VEX_CYCLES / $DHRY_VAX_RATE)")

eval "set -- $TOOL_ARGS"
python3 soc/compare/product_write.py "$OUT" dhrystone --measured \
  --target-core littlecpu --base "$BASE" --dirty "$DIRTY" --date "$DATE" \
  --seeds "$SEEDS" --cflags "$DHRY_CFLAGS" --isa "$DHRY_ISA" \
  --rom-words "$ROM_WORDS" --ram-words "$RAM_WORDS" --unit 'DMIPS/MHz' "$@" \
  --clock-ns "littlecpu=$LC_NS" --clock-ns "vexriscv=$VEX_NS" \
  --cycle-factor "littlecpu=$LC_DHRY_FACTOR" --cycle-factor "vexriscv=$VEX_DHRY_FACTOR"

measure_coremark() {
  if HZ_NS=$(sweep_clock hazard3) \
     && CM_OUT=$(make compare-coremark 2>&1) \
     && LC_CM_CYCLES=$(printf '%s\n' "$CM_OUT" | grep '^COREMARK core=littlecpu' | sed -n 's/.* cycles=\([0-9]*\).*/\1/p' | head -1) \
     && HZ_CM_CYCLES=$(printf '%s\n' "$CM_OUT" | grep '^COREMARK core=hazard3' | sed -n 's/.* cycles=\([0-9]*\).*/\1/p' | head -1) \
     && CM_ITERATIONS=$(make -s print-COMPARE_COREMARK_ITERATIONS) \
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
  echo "*** 'COREMARK core=... cycles=...' shape this script expects, or" >&2
  echo "*** COMPARE_COREMARK_CFLAGS/ITERATIONS is unset. Recording CoreMark" >&2
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

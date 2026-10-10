#!/bin/sh
# Runs both factors of the cross-core throughput product, on every part this design
# ships to, and writes the result into soc/compare/product.json.
set -eu
# SEEDS and PARTS are expanded unquoted below; a glob in either must stay a literal.
set -f

cd "$(dirname "$0")/../.."

SEEDS=${COMPARE_PRODUCT_SEEDS:-"default 1 2 3 4 5 6 7 8 9 10 11"}
if [ -z "$SEEDS" ]; then
  echo "*** run_product.sh: COMPARE_PRODUCT_SEEDS is empty, so nothing would be" >&2
  echo "*** placed. Name the seeds, or unset it for the default twelve." >&2
  exit 2
fi
for seed in $SEEDS; do
  case $seed in
    default) ;;
    *[!0-9]*)
      echo "*** run_product.sh: COMPARE_PRODUCT_SEEDS has the word '$seed'; each" >&2
      echo "*** word must be 'default' or digits." >&2
      exit 2 ;;
  esac
done
PARTS=${COMPARE_PRODUCT_PARTS:-"up5k ecp5"}
if [ -z "$PARTS" ]; then
  echo "*** run_product.sh: COMPARE_PRODUCT_PARTS is empty, so nothing would be" >&2
  echo "*** placed. Name the parts, or unset it for the default up5k and ecp5." >&2
  exit 2
fi
for part in $PARTS; do
  case $part in
    up5k|ecp5) ;;
    *) echo "*** run_product.sh: COMPARE_PRODUCT_PARTS names '$part'; this" >&2
       echo "*** harness knows up5k and ecp5. hx8k was removed, not renamed." >&2
       exit 2 ;;
  esac
done
OUT=${COMPARE_PRODUCT_OUT:-soc/compare/product.json}

# The artifact is rewritten mid-run, so it cannot count as the tree moving.
OUT_EXCLUDE=""
case $OUT in
  /*) rel=${OUT#"$PWD"/}; [ "$rel" != "$OUT" ] && OUT_EXCLUDE=$rel ;;
  *)  OUT_EXCLUDE=$OUT ;;
esac

# The opponents are gitignored clones the repo's own status cannot see.
tree_status() {
  if [ -n "$OUT_EXCLUDE" ]; then
    git status --porcelain --untracked-files=all -- . ":(exclude,literal)$OUT_EXCLUDE"
  else
    git status --porcelain --untracked-files=all
  fi
  for clone in soc/compare/hazard3 formal/riscv-formal; do
    [ -e "$clone/.git" ] || continue
    git -C "$clone" status --porcelain --untracked-files=all | sed "s|^|$clone: |"
  done
}

tree_state() {
  git rev-parse HEAD
  tree_status
  for clone in soc/compare/hazard3 formal/riscv-formal; do
    [ -e "$clone/.git" ] && echo "$clone $(git -C "$clone" rev-parse HEAD)"
  done
  python3 soc/compare/product_digest.py
}

# A measurement over a tree that moved mid-run describes neither tree.
assert_tree_unmoved() {
  if [ "$(tree_state)" != "$START_STATE" ]; then
    echo "*** run_product.sh: the tree or an opponent clone changed during the run," >&2
    echo "*** so the stamp would describe neither state. Re-run it." >&2
    exit 1
  fi
}

BASE=$(git rev-parse HEAD)
START_STATE=$(tree_state)
DIGEST=$(python3 soc/compare/product_digest.py)
if [ -z "$(tree_status)" ]; then DIRTY=no; else DIRTY=yes; fi
DATE=$(date -u '+%Y-%m-%dT%H:%M:%SZ')
ROM_WORDS=$(make -s print-COMPARE_ROM_WORDS)
RAM_WORDS=$(make -s print-COMPARE_RAM_WORDS)
# up5k's clock is a step function every swept core has already cleared by here.
STEP_MHZ=$(make -s print-COMPARE_STEP_MHZ)
ECP5_PART=$(make -s print-ECP5_PART)
ECP5_TARGET_MHZ=$(make -s print-ECP5_TARGET_MHZ)

CC=""
if command -v riscv-none-elf-gcc >/dev/null 2>&1; then CC=riscv-none-elf-gcc; fi
if [ -z "$CC" ]; then
  echo "*** run_product.sh: no RISC-V cross compiler found; see \`make riscv-gcc-setup\`." >&2
  exit 1
fi

# nextpnr-ecp5/its Trellis database: only asked for when ECP5 is actually placed.
toolchain_block() {
  set -- yosys nextpnr-ice40 icetime iverilog "$CC"
  case " $PARTS " in
    *" ecp5 "*) set -- "$@" nextpnr-ecp5 trellis-db ;;
  esac
  soc/print_toolchain.sh "$@"
}
TOOLS_BLOCK=$(toolchain_block)

isa_from_cflags() {  # $1 = CFLAGS string
  printf '%s\n' "$1" | sed -n 's/.*-march=\([A-Za-z0-9_]*\).*/\1/p'
}

pair_name() {  # $1 = benchmark, $2 = part -- ecp5 gets its own pair name, e.g. dhrystone_ecp5
  case $2 in
    up5k) printf '%s' "$1" ;;
    ecp5) printf '%s_ecp5' "$1" ;;
  esac
}

# up5k and ecp5 read different report lines here, the split soc/compare/sweep.sh uses.
sweep_clock() {  # $1 = part, $2 = core; prints comma-separated ns on stdout
  part=$1
  core=$2
  case $part in
    up5k) figure='^critical path :' ;;
    ecp5) figure='^Fmax          :' ;;
  esac
  ns_csv=""
  for seed in $SEEDS; do
    case $seed in
      default) arg="" ;;
      *)       arg=$seed ;;
    esac
    case $part in
      up5k) out=$(make compare-timing COMPARE_PART=up5k COMPARE_CORE="$core" \
                     COMPARE_SEED="$arg" 2>&1) && rc=0 || rc=$? ;;
      ecp5) out=$(make compare-timing COMPARE_PART=ecp5 COMPARE_CORE="$core" \
                     ECP5_SEED="$arg" 2>&1) && rc=0 || rc=$? ;;
    esac
    # A placement under up5k's 12 MHz step still has a measured clock, and the stamp keeps it.
    if [ "$rc" -ne 0 ] && [ "$part" = up5k ] \
       && printf '%s\n' "$out" | grep -q 'MHz is under the [0-9.]* MHz step'; then
      echo "-- $core seed '$seed' is under the up5k step; the clock is kept" >&2
    elif [ "$rc" -ne 0 ]; then
      echo "*** run_product.sh: $core seed '$seed' failed to place on $part; the" >&2
      echo "*** run stops here. That is a failed placement, not a fast design." >&2
      printf '%s\n' "$out" >&2
      exit 1
    fi
    line=$(printf '%s\n' "$out" | grep "$figure") || {
      echo "*** run_product.sh: $core seed '$seed' on $part exited 0 with no" >&2
      echo "*** '$figure' line, which the reader for $part is supposed to make" >&2
      echo "*** impossible." >&2
      exit 1
    }
    case $part in
      up5k) ns=$(printf '%s\n' "$line" | sed 's/^critical path : \([0-9.]*\) ns.*/\1/') ;;
      ecp5) ns=$(printf '%s\n' "$line" | sed 's/.*(\([0-9.]*\) ns).*/\1/') ;;
    esac
    ns_csv="${ns_csv:+$ns_csv,}$ns"
  done
  printf '%s' "$ns_csv"
}

# One sweep per (core, part) serves both benchmark pairs below.
for part in $PARTS; do
  echo "== compare-product: clock sweep on $part ($SEEDS) =="
  for core in littlecpu vexriscv hazard3 hazard3_perf vexriscv_lrsc hazard3_c; do
    echo "-- $core --"
    ns=$(sweep_clock "$part" "$core")
    echo "$ns"
    eval "NS_${part}_${core}=\$ns"
  done
  echo
done

echo "== compare-product: Dhrystone cycles (littlecpu against VexRiscv and both Hazard3 builds) =="
if ! DHRY_OUT=$(make compare-dhrystone 2>&1); then
  echo "*** run_product.sh: make compare-dhrystone failed." >&2
  printf '%s\n' "$DHRY_OUT" >&2
  exit 1
fi
printf '%s\n' "$DHRY_OUT"

# FIRST match: the three-way row, not the ISA-cost/pairwise rows also printed.
LC_CYCLES=$(printf '%s\n' "$DHRY_OUT" | grep '^DHRY core=littlecpu' | sed -n 's/.*cycles=\([0-9]*\).*/\1/p' | head -1)
VEX_CYCLES=$(printf '%s\n' "$DHRY_OUT" | grep '^DHRY core=vexriscv' | sed -n 's/.*cycles=\([0-9]*\).*/\1/p' | head -1)
# `core=hazard3 marks=` and not `core=hazard3`, which is also the prefix of hazard3_perf.
HZ_CYCLES=$(printf '%s\n' "$DHRY_OUT" | grep '^DHRY core=hazard3 marks=' | sed -n 's/.* cycles=\([0-9]*\).*/\1/p' | head -1)
HZP_CYCLES=$(printf '%s\n' "$DHRY_OUT" | grep '^DHRY core=hazard3_perf marks=' | sed -n 's/.* cycles=\([0-9]*\).*/\1/p' | head -1)
if [ -z "$LC_CYCLES" ] || [ -z "$VEX_CYCLES" ] || [ -z "$HZ_CYCLES" ] || [ -z "$HZP_CYCLES" ]; then
  echo "*** run_product.sh: could not find all four cores' 'DHRY core=... cycles='" >&2
  echo "*** lines in make compare-dhrystone's output." >&2
  exit 1
fi
DHRY_RUNS=$(make -s print-COMPARE_DHRY_RUNS)
DHRY_CFLAGS=$(make -s print-COMPARE_DHRY_CFLAGS)
DHRY_ISA=$(isa_from_cflags "$DHRY_CFLAGS")

DHRY_VAX_RATE=$(python3 -c "import sys; sys.path.insert(0, 'soc/compare'); \
  from dhry_dmips import VAX_DHRYSTONES_PER_SEC; print(VAX_DHRYSTONES_PER_SEC)")

for pair in "DHRY_RUNS=$DHRY_RUNS" "LC_CYCLES=$LC_CYCLES" \
            "VEX_CYCLES=$VEX_CYCLES" "HZ_CYCLES=$HZ_CYCLES" "HZP_CYCLES=$HZP_CYCLES" \
            "DHRY_VAX_RATE=$DHRY_VAX_RATE"; do
  name=${pair%%=*}; value=${pair#*=}
  case "$value" in
    ''|*[!0-9.]*)
      echo "*** run_product.sh: $name is '$value', which is not a number." >&2
      echo "*** Fix what produced it." >&2
      exit 1 ;;
  esac
done

# Values travel as arguments, never spliced into the program text.
cycle_factor() {  # $1 = runs, $2 = cycles, $3 = divisor (default 1)
  python3 -c 'import sys; a = [float(v) for v in sys.argv[1:]]; print(a[0] * 1e6 / a[1] / a[2])' \
    "$1" "$2" "${3:-1}"
}
LC_DHRY_FACTOR=$(cycle_factor "$DHRY_RUNS" "$LC_CYCLES" "$DHRY_VAX_RATE")
VEX_DHRY_FACTOR=$(cycle_factor "$DHRY_RUNS" "$VEX_CYCLES" "$DHRY_VAX_RATE")
HZ_DHRY_FACTOR=$(cycle_factor "$DHRY_RUNS" "$HZ_CYCLES" "$DHRY_VAX_RATE")
HZP_DHRY_FACTOR=$(cycle_factor "$DHRY_RUNS" "$HZP_CYCLES" "$DHRY_VAX_RATE")
CYCLE_TOOLS_BLOCK=$(toolchain_block)
assert_tree_unmoved

for part in $PARTS; do
  eval "lc_ns=\$NS_${part}_littlecpu"
  eval "vex_ns=\$NS_${part}_vexriscv"
  eval "hz_ns=\$NS_${part}_hazard3"
  eval "hzp_ns=\$NS_${part}_hazard3_perf"
  set --
  case $part in
    up5k) set -- "$@" --step-mhz "$STEP_MHZ" ;;
    ecp5) set -- "$@" --field "ecp5_part=$ECP5_PART" \
                     --field "ecp5_target_mhz=$ECP5_TARGET_MHZ" ;;
  esac
  python3 soc/compare/product_write.py "$OUT" "$(pair_name dhrystone "$part")" --measured \
    --target-core littlecpu --base "$BASE" --dirty "$DIRTY" --date "$DATE" \
    --seeds "$SEEDS" --cflags "$DHRY_CFLAGS" --isa "$DHRY_ISA" \
    --rom-words "$ROM_WORDS" --ram-words "$RAM_WORDS" --unit 'DMIPS/MHz' \
    --digest "$DIGEST" --tools-block "$TOOLS_BLOCK" --cycle-tools-block "$CYCLE_TOOLS_BLOCK" "$@" \
    --clock-ns "littlecpu=$lc_ns" --clock-ns "vexriscv=$vex_ns" \
    --clock-ns "hazard3=$hz_ns" --clock-ns "hazard3_perf=$hzp_ns" \
    --cycle-factor "littlecpu=$LC_DHRY_FACTOR" --cycle-factor "vexriscv=$VEX_DHRY_FACTOR" \
    --cycle-factor "hazard3=$HZ_DHRY_FACTOR" --cycle-factor "hazard3_perf=$HZP_DHRY_FACTOR"
done

measure_coremark() {
  if CM_OUT=$(make compare-coremark 2>&1) \
     && LC_CM_CYCLES=$(printf '%s\n' "$CM_OUT" | grep '^COREMARK core=littlecpu' | sed -n 's/.* cycles=\([0-9]*\).*/\1/p' | head -1) \
     && VEX_CM_CYCLES=$(printf '%s\n' "$CM_OUT" | grep '^COREMARK core=vexriscv' | sed -n 's/.* cycles=\([0-9]*\).*/\1/p' | head -1) \
     && HZ_CM_CYCLES=$(printf '%s\n' "$CM_OUT" | grep '^COREMARK core=hazard3 marks=' | sed -n 's/.* cycles=\([0-9]*\).*/\1/p' | head -1) \
     && HZP_CM_CYCLES=$(printf '%s\n' "$CM_OUT" | grep '^COREMARK core=hazard3_perf marks=' | sed -n 's/.* cycles=\([0-9]*\).*/\1/p' | head -1) \
     && CM_ITERATIONS=$(make -s print-COMPARE_COREMARK_ITERATIONS) \
     && CM_CFLAGS=$(make -s print-COMPARE_COREMARK_CFLAGS) \
     && [ -n "$LC_CM_CYCLES" ] && [ -n "$VEX_CM_CYCLES" ] && [ -n "$HZ_CM_CYCLES" ] && [ -n "$HZP_CM_CYCLES" ] \
     && [ -n "$CM_ITERATIONS" ] && [ -n "$CM_CFLAGS" ]; then
    printf '%s\n' "$CM_OUT"
    for value in "$CM_ITERATIONS" "$LC_CM_CYCLES" "$VEX_CM_CYCLES" "$HZ_CM_CYCLES" "$HZP_CM_CYCLES"; do
      case $value in
        ''|*[!0-9]*)
          echo "*** run_product.sh: CoreMark iterations or cycles read '$value'," >&2
          echo "*** which is not a count." >&2
          return 1 ;;
      esac
    done
    LC_CM_FACTOR=$(cycle_factor "$CM_ITERATIONS" "$LC_CM_CYCLES")
    VEX_CM_FACTOR=$(cycle_factor "$CM_ITERATIONS" "$VEX_CM_CYCLES")
    HZ_CM_FACTOR=$(cycle_factor "$CM_ITERATIONS" "$HZ_CM_CYCLES")
    HZP_CM_FACTOR=$(cycle_factor "$CM_ITERATIONS" "$HZP_CM_CYCLES")
    CYCLE_TOOLS_BLOCK=$(toolchain_block)
    assert_tree_unmoved
    CM_ISA=$(isa_from_cflags "$CM_CFLAGS")
    # `|| return 1`: `set -e` is suspended in this whole function, since it is
    # the left side of `measure_coremark && COREMARK_OK=1` at the call site.
    for part in $PARTS; do
      eval "lc_ns=\$NS_${part}_littlecpu"
      eval "vex_ns=\$NS_${part}_vexriscv"
      eval "hz_ns=\$NS_${part}_hazard3"
      eval "hzp_ns=\$NS_${part}_hazard3_perf"
      set --
      case $part in
        up5k) set -- "$@" --step-mhz "$STEP_MHZ" ;;
        ecp5) set -- "$@" --field "ecp5_part=$ECP5_PART" \
                         --field "ecp5_target_mhz=$ECP5_TARGET_MHZ" ;;
      esac
      python3 soc/compare/product_write.py "$OUT" "$(pair_name coremark "$part")" --measured \
        --target-core littlecpu --base "$BASE" --dirty "$DIRTY" --date "$DATE" \
        --seeds "$SEEDS" --cflags "$CM_CFLAGS" --isa "$CM_ISA" \
        --rom-words "$ROM_WORDS" --ram-words "$RAM_WORDS" --unit 'CoreMark/MHz' \
        --digest "$DIGEST" --tools-block "$TOOLS_BLOCK" --cycle-tools-block "$CYCLE_TOOLS_BLOCK" "$@" \
        --clock-ns "littlecpu=$lc_ns" --clock-ns "vexriscv=$vex_ns" --clock-ns "hazard3=$hz_ns" \
        --clock-ns "hazard3_perf=$hzp_ns" \
        --cycle-factor "littlecpu=$LC_CM_FACTOR" --cycle-factor "vexriscv=$VEX_CM_FACTOR" \
        --cycle-factor "hazard3=$HZ_CM_FACTOR" --cycle-factor "hazard3_perf=$HZP_CM_FACTOR" \
        || return 1
    done
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
echo "== compare-product: CoreMark (littlecpu against VexRiscv and both Hazard3 builds) =="
COREMARK_CAPABLE=0
if grep -q '^compare-coremark:' Makefile && [ -f soc/compare/coremark_dmips.py ]; then
  COREMARK_CAPABLE=1
fi
COREMARK_OK=0
if [ "$COREMARK_CAPABLE" -eq 1 ]; then
  echo "make compare-coremark is on this tree; attempting the measurement."
  if measure_coremark; then
    COREMARK_OK=1
  else
    echo "*** run_product.sh: CoreMark is on this tree but the measurement" >&2
    echo "*** failed (see above); stopping here rather than falling back to" >&2
    echo "*** not-yet-measured, which could overwrite a pair an earlier part" >&2
    echo "*** in this same run already wrote successfully." >&2
    exit 1
  fi
fi
if [ "$COREMARK_OK" -eq 0 ]; then
  REASON="make compare-coremark is not on this tree yet; run_product.sh will measure it once that lands"
  for part in $PARTS; do
    python3 soc/compare/product_write.py "$OUT" "$(pair_name coremark "$part")" \
      --not-yet-measured --target-core littlecpu --core hazard3 --core hazard3_perf --core vexriscv \
      --reason "$REASON"
  done
fi

under_step() {  # $1 = comma-separated ns: true when the worst placement misses the step
  python3 -c 'import sys; sys.exit(0 if 1000 / max(float(v) for v in sys.argv[2].split(",")) < float(sys.argv[1]) else 1)' \
    "$STEP_MHZ" "$1"
}

mhz_range() {  # $1 = comma-separated ns
  python3 -c 'import sys; ns = [float(v) for v in sys.argv[1].split(",")]; print("%.2f to %.2f MHz" % (1000 / max(ns), 1000 / min(ns)))' "$1"
}

# The feature-matched pairs reuse the clock sweeps above; only the cycle factors are new.
measure_matched() {  # $1 = dhrystone | coremark
  bench=$1
  case $bench in
    dhrystone)
      target=compare-dhrystone-matched; tag=DHRY; unit='DMIPS/MHz'
      count=$DHRY_RUNS; divisor=$DHRY_VAX_RATE
      cflags=$(make -s print-COMPARE_DHRY_IMAC_CFLAGS) ;;
    coremark)
      target=compare-coremark-matched; tag=COREMARK; unit='CoreMark/MHz'
      count=$(make -s print-COMPARE_COREMARK_ITERATIONS); divisor=1
      cflags=$(make -s print-COMPARE_COREMARK_IMAC_CFLAGS) ;;
  esac
  if ! M_OUT=$(make "$target" 2>&1); then
    echo "*** run_product.sh: make $target failed." >&2
    printf '%s\n' "$M_OUT" >&2
    return 1
  fi
  printf '%s\n' "$M_OUT"
  matched_cycles() {  # $1 = core
    printf '%s\n' "$M_OUT" | grep "^$tag core=$1 marks=" \
      | sed -n 's/.* cycles=\([0-9]*\).*/\1/p' | head -1
  }
  m_lc=$(matched_cycles littlecpu)
  m_vx=$(matched_cycles vexriscv_lrsc)
  m_hz=$(matched_cycles hazard3_c)
  for value in "$count" "$m_lc" "$m_vx" "$m_hz" "$divisor"; do
    case $value in
      ''|*[!0-9.]*)
        echo "*** run_product.sh: the $bench feature-matched run gave '$value'," >&2
        echo "*** which is not a count." >&2
        return 1 ;;
    esac
  done
  m_lc_factor=$(cycle_factor "$count" "$m_lc" "$divisor")
  m_vx_factor=$(cycle_factor "$count" "$m_vx" "$divisor")
  m_hz_factor=$(cycle_factor "$count" "$m_hz" "$divisor")
  CYCLE_TOOLS_BLOCK=$(toolchain_block)
  assert_tree_unmoved
  m_isa=$(isa_from_cflags "$cflags")
  for part in $PARTS; do
    set --
    case $part in
      up5k) set -- "$@" --step-mhz "$STEP_MHZ" ;;
      ecp5) set -- "$@" --field "ecp5_part=$ECP5_PART" \
                       --field "ecp5_target_mhz=$ECP5_TARGET_MHZ" ;;
    esac
    # A core under the up5k step is out of that comparison: it leaves the pair, which says so.
    out_of=""
    for entry in "littlecpu:$m_lc_factor" "vexriscv_lrsc:$m_vx_factor" "hazard3_c:$m_hz_factor"; do
      core=${entry%%:*}
      eval "ns=\$NS_${part}_${core}"
      if [ "$part" = up5k ] && under_step "$ns"; then
        out_of="${out_of:+$out_of; }$core $(mhz_range "$ns")"
        continue
      fi
      set -- "$@" --clock-ns "$core=$ns" --cycle-factor "$core=${entry#*:}"
    done
    [ -z "$out_of" ] || set -- "$@" --field "out_of_comparison=$out_of"
    python3 soc/compare/product_write.py "$OUT" "$(pair_name "${bench}_imac" "$part")" --measured \
      --target-core littlecpu --base "$BASE" --dirty "$DIRTY" --date "$DATE" \
      --seeds "$SEEDS" --cflags "$cflags" --isa "$m_isa" \
      --rom-words "$ROM_WORDS" --ram-words "$RAM_WORDS" --unit "$unit" \
      --digest "$DIGEST" --tools-block "$TOOLS_BLOCK" --cycle-tools-block "$CYCLE_TOOLS_BLOCK" \
      "$@" || return 1
  done
}

for matched_bench in dhrystone coremark; do
  echo
  echo "== compare-product: $matched_bench, feature-matched (RV32IMAC) =="
  if ! measure_matched "$matched_bench"; then
    echo "*** run_product.sh: the feature-matched $matched_bench measurement failed;" >&2
    echo "*** stopping rather than leaving the stamp half-written." >&2
    exit 1
  fi
done

echo
echo "== $OUT =="
set -- --current "compiler=$CC" --current "compiler_version=$(make -s print-RISCV_GCC_VERSION)"
for part in $PARTS; do
  python3 soc/compare/product_check.py "$OUT" "$(pair_name dhrystone "$part")" --repo . \
    --current "cflags=$DHRY_CFLAGS" --current "rom_words=$ROM_WORDS" \
    --current "ram_words=$RAM_WORDS" "$@"
  if [ "$COREMARK_OK" -eq 1 ]; then
    python3 soc/compare/product_check.py "$OUT" "$(pair_name coremark "$part")" --repo . \
      --current "cflags=$CM_CFLAGS" --current "rom_words=$ROM_WORDS" \
      --current "ram_words=$RAM_WORDS" "$@"
  else
    python3 soc/compare/product_check.py "$OUT" "$(pair_name coremark "$part")" --repo . "$@"
  fi
  python3 soc/compare/product_check.py "$OUT" "$(pair_name dhrystone_imac "$part")" --repo . \
    --current "cflags=$(make -s print-COMPARE_DHRY_IMAC_CFLAGS)" --current "rom_words=$ROM_WORDS" \
    --current "ram_words=$RAM_WORDS" "$@"
  python3 soc/compare/product_check.py "$OUT" "$(pair_name coremark_imac "$part")" --repo . \
    --current "cflags=$(make -s print-COMPARE_COREMARK_IMAC_CFLAGS)" \
    --current "rom_words=$ROM_WORDS" --current "ram_words=$RAM_WORDS" "$@"
done

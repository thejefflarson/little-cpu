#!/bin/bash
# Assembles every test/asm/*.S file, runs it under the cxxrtl runner (`sim`),
# and checks the resulting pass/fail table against test/EXPECTED_FAIL — the
# sprint-1 baseline (ADR-0007, ADR-0008, ADR-0014). Invoked by `make test`.
#
# Usage: run_tests.sh <sim-binary> <asm-dir> <expected-fail-file>
#
# This script is the merge gate, so the failure mode that matters is a FALSE
# GREEN — reporting success without having tested anything. Three properties
# below exist only for that (ADR-0035):
#
#   * every build step's exit status is checked, and the simulator runs only
#     when the image it would read was actually produced. An unchecked objcopy
#     that emits an empty RAM image makes `tohost` read zero, which every test
#     with no data dependency still turns into a PASS;
#   * the failure set records NAME AND STATUS, not just the name, so the
#     set-equality check against test/EXPECTED_FAIL pins the failure *mode*.
#     A baselined test that starts failing for a new reason — a broken
#     assembly, a crashing runner, a timeout — is a red gate, not a match;
#   * `set -e` is on and mktemp is fatal. Without it a failed mktemp leaves
#     $tmp empty, every artifact path collapses to the filesystem root, and a
#     later run can pick up the previous run's stale image.
#
# Any nonzero exit that is *expected* is handled at its own call site (`if !`,
# `|| status=$?`) so that error-exit stays on everywhere else.
set -euo pipefail

if [ "$#" -ne 3 ]; then
  echo "usage: run_tests.sh <sim-binary> <asm-dir> <expected-fail-file>" >&2
  exit 1
fi

SIM=$1
ASM_DIR=$2
EXPECTED_FAIL=$3
CYCLES=5000

# Read before running anything: a mistyped or missing baseline path used to
# yield an empty expected set, and with an all-passing suite the gate then
# printed "Failure list matches ... exactly" having compared nothing. Fail
# here, in a second, rather than after the whole suite has run.
if [ ! -f "$EXPECTED_FAIL" ] || [ ! -r "$EXPECTED_FAIL" ]; then
  echo "error: baseline '$EXPECTED_FAIL' does not exist or is not readable." >&2
  echo "The gate compares the failure set against it; without it there is no gate." >&2
  exit 1
fi

# Toolchain: brew's riscv64-elf-gcc on macOS, apt's gcc-riscv64-unknown-elf
# (binary name riscv64-unknown-elf-gcc) on Linux. See `make setup`.
CC=""
for candidate in riscv64-elf-gcc riscv64-unknown-elf-gcc; do
  if command -v "$candidate" >/dev/null 2>&1; then
    CC=$candidate
    break
  fi
done
if [ -z "$CC" ]; then
  echo "error: no RISC-V cross compiler found (tried riscv64-elf-gcc, riscv64-unknown-elf-gcc)." >&2
  echo "Run 'make setup' to install one." >&2
  exit 1
fi

# objcopy is derived from the compiler's name but probed like the compiler,
# rather than assumed to exist because gcc did. A half-installed toolchain is
# a setup problem and should say so once, up front, instead of surfacing as 52
# identical per-test build failures.
OBJCOPY=${CC%gcc}objcopy
if ! command -v "$OBJCOPY" >/dev/null 2>&1; then
  echo "error: found $CC but not its matching $OBJCOPY." >&2
  echo "Run 'make setup' to install a complete toolchain." >&2
  exit 1
fi

tmp=$(mktemp -d "${TMPDIR:-/tmp}/littlecpu-test.XXXXXX") || {
  echo "error: could not create a temporary directory under ${TMPDIR:-/tmp}." >&2
  exit 1
}
# Belt and braces: an mktemp that exits 0 with empty output would otherwise
# collapse every artifact path to /<base>.hex and defeat the trap below.
if [ -z "$tmp" ] || [ ! -d "$tmp" ]; then
  echo "error: mktemp -d produced no usable directory under ${TMPDIR:-/tmp}." >&2
  exit 1
fi
trap 'rm -rf "$tmp"' EXIT

declare -a failures=()
declare -a table=()
passed=0

for src in "$ASM_DIR"/*.S; do
  name=$(basename "$src")
  base=${name%.S}
  elf="$tmp/$base.elf"
  build_log="$tmp/$base.build.log"
  rom_hex="$tmp/$base.rom.hex"
  ram_hex="$tmp/$base.ram.hex"

  status="PASS"
  if ! "$CC" -march=rv32imc_zicsr -mabi=ilp32 -nostdlib -I "$ASM_DIR" \
       -T "$ASM_DIR/sections.lds" "$src" -o "$elf" > "$build_log" 2>&1; then
    status="ASSEMBLE-ERROR"
  elif [ -s "$build_log" ]; then
    # Any assembler output (warnings included) on an otherwise-successful
    # build is still a defect per criterion 7 — zero assembler warnings.
    status="ASSEMBLE-WARNING"
  fi

  # ADR-0008's two images. Both statuses are checked: a failing objcopy
  # (a binutils without --verilog-data-width, a full disk) used to leave the
  # previous file — or no file — in place and let the simulator run against
  # it, reporting its verdict as if it were about the CPU. The empty case is
  # the dangerous one and gets its own label: an empty RAM image parses fine,
  # `tohost` reads zero, and every test with no data dependency still reaches
  # RVTEST_PASS.
  if [ "$status" = "PASS" ]; then
    for region in rom ram; do
      if [ "$region" = rom ]; then
        section_flag="--only-section=.text"
        image=$rom_hex
      else
        section_flag="--remove-section=.text"
        image=$ram_hex
      fi
      rm -f "$image"
      if ! "$OBJCOPY" -O verilog --verilog-data-width=4 "$section_flag" \
           "$elf" "$image" >> "$build_log" 2>&1; then
        status="OBJCOPY-ERROR $region"
        break
      fi
      if [ ! -s "$image" ]; then
        status="OBJCOPY-EMPTY $region"
        break
      fi
    done
  fi

  if [ "$status" = "PASS" ]; then
    # The one place error-exit is lifted, and it is lifted rather than guarded
    # with `|| sim_status=$?` because bash 3.2 (macOS /bin/bash) rewrites a
    # 127 "command not found" to 1 while `set -e` is in force — which would
    # report an unstartable runner as a `FAIL`, i.e. as a verdict about the
    # CPU. Measured, not assumed. Restored immediately after.
    set +e
    "$SIM" --rom "$rom_hex" --ram "$ram_hex" --cycles "$CYCLES" \
      > "$tmp/$base.run.log" 2>&1
    sim_status=$?
    set -e
    # The runner's exit ladder (test/cxxrtl.cc): 0 pass, 1 tohost fail, 2 cycle
    # limit, 3 usage/setup, 4 RVFI monitor error. 4 gets its own label because
    # it means something entirely different from the others — the per-retire
    # oracle disagreed with the core mid-run (ADR-0019), which is a wrong-result
    # report, not a broken harness. Lumping it into RUNNER-ERROR made it
    # indistinguishable from "the sim could not be started at all".
    case $sim_status in
      0) status="PASS" ;;
      1) num=$(awk '/^FAIL/{print $2; exit}' "$tmp/$base.run.log")
         status="FAIL${num:+ $num}" ;;
      2) status="TIMEOUT" ;;
      4) code=$(awk '/RVFI monitor error/{print $4; exit}' "$tmp/$base.run.log")
         status="MONITOR-ERROR${code:+ $code}" ;;
      *) status="RUNNER-ERROR $sim_status" ;;
    esac
  fi

  if [ "$status" = "PASS" ]; then
    passed=$((passed + 1))
  else
    failures+=("$name $status")
    if [ -s "$build_log" ]; then
      echo "--- $name build output ---" >&2
      cat "$build_log" >&2
    fi
  fi
  table+=("$(printf '%-16s %s' "$name" "$status")")
done

printf '%s\n' "${table[@]}"
echo
echo "$passed/${#table[@]} passed"

# Both sides are name-and-status pairs, whitespace-normalised so the baseline
# can stay readable. `NF` must gate the rebuild rather than follow it: awk
# forces NF to 1 when $1 is assigned, so `{$1=$1} NF` would resurrect every
# blank and comment-only line as an empty entry.
# ADR-0035 amends ADR-0014: matching on the name alone let a baselined test
# change failure mode — ASSEMBLE-ERROR, TIMEOUT, RUNNER-ERROR instead of the
# baselined MONITOR-ERROR — and still satisfy set equality.
actual_sorted=$(printf '%s\n' "${failures[@]:-}" | awk 'NF { $1=$1; print }' | sort)
expected_sorted=$(sed -e 's/#.*//' "$EXPECTED_FAIL" | awk 'NF { $1=$1; print }' | sort)

# A one-field line is the pre-ADR-0035 format. Silently accepting it would
# make every baselined entry unmatchable for a reason that reads like a
# regression, so name it instead.
malformed=$(printf '%s\n' "$expected_sorted" | awk 'NF == 1 {print}')
if [ -n "$malformed" ]; then
  echo "error: $EXPECTED_FAIL has entries with no status (ADR-0035 format is" >&2
  echo "'<test>.S <STATUS>', e.g. 'trap.S MONITOR-ERROR 101'):" >&2
  printf '  %s\n' "$malformed" >&2
  exit 1
fi

if [ "$actual_sorted" = "$expected_sorted" ]; then
  echo "Failure list matches $EXPECTED_FAIL exactly (name and status)."
  exit 0
fi

echo
echo "Failure list does NOT match $EXPECTED_FAIL:" >&2
diff <(echo "$expected_sorted") <(echo "$actual_sorted") \
  --label expected --label actual >&2 || true
exit 1

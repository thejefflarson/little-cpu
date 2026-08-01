#!/bin/bash
# Assembles every test/asm/*.S file, runs it under the cxxrtl runner (`sim`),
# and checks the resulting pass/fail table against test/EXPECTED_FAIL — the
# sprint-1 baseline (ADR-0007, ADR-0008, ADR-0014). Invoked by `make test`.
#
# Usage: run_tests.sh <sim-binary> <asm-dir> <expected-fail-file> <floor-file>
#
# This script is the merge gate, so the failure mode that matters is a FALSE
# GREEN — reporting success without having tested anything. Four properties
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
#     later run can pick up the previous run's stale image;
#   * THE SUITE'S SHAPE IS ASSERTED BEFORE ANYTHING RUNS. This gate verified
#     that every program it FOUND passed; it had no idea how many it should
#     find, and with both baselines empty there was no red entry whose
#     disappearance would say the suite had shrunk. test/OBSERVED_FLOOR is the
#     manifest — it already names every program — and test/check_suite_shape.sh
#     compares its name set against test/asm in BOTH directions up front. The
#     orphan check that used to do half of that at the END of the run is gone;
#     doing it after 52 assemblies and 52 simulations was strictly worse;
#   * OBSERVATION IS COUNTED, NOT INFERRED. The RVFI monitor is this gate's
#     per-retire oracle and nothing measured whether it ever fired. A monitor
#     that never saw a retire — an under-sensitivity defect of the ADR-0037
#     kind, an `ifdef` that dropped the shadow payload, a `write_cxxrtl` that
#     optimised the instance away — left every program reporting PASS off
#     `tohost` alone, and with test/EXPECTED_FAIL empty there was no red entry
#     whose disappearance would say so. ADR-0032 measured that end-state
#     checking on its own is blind to real architectural corruption. So the
#     runner now prints how many retires the monitor examined and how many of
#     those its spec model checked, and this script grades both: zero of either
#     is MONITOR-SILENT (the runner's own exit 6), and a drop below
#     test/OBSERVED_FLOOR is BELOW-FLOOR. Both are failing statuses in the same
#     name-and-status set as everything else.
#
# Any nonzero exit that is *expected* is handled at its own call site (`if !`,
# `|| status=$?`) so that error-exit stays on everywhere else.
set -euo pipefail

if [ "$#" -ne 4 ]; then
  echo "usage: run_tests.sh <sim-binary> <asm-dir> <expected-fail-file> <floor-file>" >&2
  exit 1
fi

SIM=$1
ASM_DIR=$2
EXPECTED_FAIL=$3
OBSERVED_FLOOR=$4
CYCLES=5000
HERE=$(cd "$(dirname "$0")" && pwd)

# Read before running anything: a mistyped or missing baseline path used to
# yield an empty expected set, and with an all-passing suite the gate then
# printed "Failure list matches ... exactly" having compared nothing. Fail
# here, in a second, rather than after the whole suite has run.
if [ ! -f "$EXPECTED_FAIL" ] || [ ! -r "$EXPECTED_FAIL" ]; then
  echo "error: baseline '$EXPECTED_FAIL' does not exist or is not readable." >&2
  echo "The gate compares the failure set against it; without it there is no gate." >&2
  exit 1
fi

# Same treatment, and for the same reason: a missing floor file would make
# every per-program floor lookup miss, and a lookup that always misses is a
# check that never runs.
if [ ! -f "$OBSERVED_FLOOR" ] || [ ! -r "$OBSERVED_FLOOR" ]; then
  echo "error: floor file '$OBSERVED_FLOOR' does not exist or is not readable." >&2
  echo "It records how much each program was observed to retire; without it" >&2
  echo "there is nothing to compare this run's observation against." >&2
  exit 1
fi

# THE SUITE'S SHAPE, before a single program is assembled. OBSERVED_FLOOR names
# every program the suite must contain, and this compares that name set against
# $ASM_DIR in both directions. It is a separate script because test/run_cosim.sh
# runs the identical check against the identical file: two legs that disagreed
# about what the suite is would be worse than neither checking.
if ! "$HERE/check_suite_shape.sh" "$ASM_DIR" "$OBSERVED_FLOOR"; then
  echo "error: the .S suite does not match its manifest; nothing was run." >&2
  exit 1
fi

# Read once, comments stripped, whitespace normalised — the same treatment the
# baseline gets below. Three fields, not two: name, retire floor, spec-checked
# floor. A malformed line is named rather than silently skipped, because a line
# this loop cannot parse is a floor that is not enforced.
floors=$(sed -e 's/#.*//' "$OBSERVED_FLOOR" | awk 'NF { $1=$1; print }')
malformed_floor=$(printf '%s\n' "$floors" | awk 'NF && (NF != 3 || $2 !~ /^[0-9]+$/ || $3 !~ /^[0-9]+$/) { print }')
if [ -n "$malformed_floor" ]; then
  echo "error: $OBSERVED_FLOOR has lines that are not '<test>.S <retires> <spec-checked>':" >&2
  printf '  %s\n' "$malformed_floor" >&2
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
  # Cleared per iteration, not merely assigned when the runner produces them: a
  # carried-over value from the previous program would let a run that printed no
  # counts be graded against its predecessor's.
  retires=""
  spec_retires=""
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
    # limit, 3 usage/setup, 4 RVFI monitor error, 5 trap-to-zero. 4 and 5 get
    # their own labels because each means something entirely different from
    # "the test failed": 4 is the per-retire oracle disagreeing with the core
    # mid-run (ADR-0019), a wrong-result report rather than a broken harness,
    # and 5 is a trap taken before a handler was installed (ADR-0029), which
    # would otherwise surface as a TIMEOUT naming neither the fault nor its
    # cause. Lumping either into RUNNER-ERROR made it indistinguishable from
    # "the sim could not be started at all".
    case $sim_status in
      0) status="PASS" ;;
      1) num=$(awk '/^FAIL/{print $2; exit}' "$tmp/$base.run.log")
         status="FAIL${num:+ $num}" ;;
      2) status="TIMEOUT" ;;
      4) code=$(awk '/RVFI monitor error/{print $4; exit}' "$tmp/$base.run.log")
         status="MONITOR-ERROR${code:+ $code}" ;;
      5) status="TRAP-TO-ZERO" ;;
      6) status="MONITOR-SILENT" ;;
      *) status="RUNNER-ERROR $sim_status" ;;
    esac

    # The runner prints exactly one `RETIRES <n> SPEC-CHECKED <m>` line on every
    # path that reached the simulation loop, whatever its verdict, so these are
    # available for the table even when the program failed.
    set -- $(awk '/^RETIRES /{print $2, $4; exit}' "$tmp/$base.run.log")
    retires=${1:-}
    spec_retires=${2:-}
  fi

  # A PASS with no counts line is not a pass. It means the binary this gate ran
  # is not the runner this script grades — a stale `sim` from before the
  # counters existed, or one whose output was truncated — and every one of the
  # observation checks below would then silently not run. Naming it is the
  # difference between a gate and a formality.
  if [ "$status" = "PASS" ] && { [ -z "$retires" ] || [ -z "$spec_retires" ]; }; then
    status="NO-COUNTS"
  fi

  # THE FLOOR, graded with `>=` and not set equality. The counts move for
  # legitimate reasons — the assembler is free to compress differently, a test
  # gains an instruction — and an exact ratchet on 52 numbers is one nobody
  # would keep. What must not happen is a program going quiet: retiring nothing,
  # or having its retires stop being spec-checked. `>=` catches that and
  # tolerates the rest. Only PASS programs are graded; anything already failing
  # has a more specific status, and overwriting it would lose the real reason.
  if [ "$status" = "PASS" ]; then
    floor=$(printf '%s\n' "$floors" | awk -v n="$name" '$1 == n { print $2, $3; found = 1 } END { exit !found }') || floor=""
    if [ -z "$floor" ]; then
      # UNREACHABLE as long as check_suite_shape.sh runs above: it has already
      # required every program in $ASM_DIR to have a manifest line, and failed
      # the whole run naming this one if it did not. Kept because the lookup has
      # to handle the not-found case regardless, and a silent empty `$floor`
      # would make the two comparisons below compare against nothing. If this
      # ever fires, the two checks have gone out of step with each other, which
      # is worth a named status rather than an arithmetic error.
      status="NO-FLOOR"
    else
      set -- $floor
      floor_retires=$1
      floor_spec=$2
      if [ "$retires" -lt "$floor_retires" ]; then
        status="BELOW-FLOOR retires"
        echo "$name: $retires retires, floor is $floor_retires ($OBSERVED_FLOOR)" >&2
      elif [ "$spec_retires" -lt "$floor_spec" ]; then
        status="BELOW-FLOOR spec-checked"
        echo "$name: $spec_retires spec-checked retires, floor is $floor_spec ($OBSERVED_FLOOR)" >&2
      fi
    fi
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
  # The observation counts are a THIRD column, deliberately outside the
  # name-and-status pair the baseline matches on (ADR-0035): they are a measured
  # quantity that moves, not a verdict, and putting them in the failure set
  # would make every baseline entry unmatchable the first time an instruction
  # count changed. The verdict they imply — MONITOR-SILENT, BELOW-FLOOR,
  # NO-COUNTS, NO-FLOOR — is in the status, where the gate can pin it.
  table+=("$(printf '%-16s %-22s %s' "$name" "$status" \
    "retires=${retires:--} spec-checked=${spec_retires:--}")")
done

printf '%s\n' "${table[@]}"
echo
echo "$passed/${#table[@]} passed"

# Both directions of the floor file's name set (ADR-0014's contract, the same
# one EXPECTED_FAIL and formal/EXPECTED_CHECKS are under) were checked HERE,
# after the whole suite had run. They now run before it, in
# check_suite_shape.sh, called at the top of this script — a manifest mismatch
# means the run was never going to be gradeable, so spending 52 assemblies and
# 52 simulations to discover it was the wrong order. Nothing is checked twice.

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

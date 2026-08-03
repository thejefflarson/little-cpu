#!/bin/bash
# Assembles every test/asm/*.S, runs it under the cxxrtl runner (`sim`), and
# grades the pass/fail table against test/EXPECTED_FAIL. Invoked by `make test`.
#
# Usage: run_tests.sh <sim-binary> <asm-dir> <expected-fail-file> <floor-file>
#
# This is the merge gate, so the failure mode that matters is a FALSE GREEN —
# reporting success without having tested anything. That is what every guard
# below is for, and each one is probed from test/probe_gates.sh.
#
# `set -e` is on; a nonzero exit that is expected is handled at its own call
# site so error-exit stays on everywhere else.
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

# A missing or unreadable baseline yields an EMPTY expected set, which against
# an all-passing suite prints "Failure list matches ... exactly" having compared
# nothing.
if [ ! -f "$EXPECTED_FAIL" ] || [ ! -r "$EXPECTED_FAIL" ]; then
  echo "error: baseline '$EXPECTED_FAIL' does not exist or is not readable." >&2
  echo "The gate compares the failure set against it; without it there is no gate." >&2
  exit 1
fi

if [ ! -f "$OBSERVED_FLOOR" ] || [ ! -r "$OBSERVED_FLOOR" ]; then
  echo "error: floor file '$OBSERVED_FLOOR' does not exist or is not readable." >&2
  echo "It records how much each program was observed to retire; without it" >&2
  echo "there is nothing to compare this run's observation against." >&2
  exit 1
fi

# Before anything is assembled, because a suite that shrank otherwise matches an
# empty baseline exactly. A separate script because test/run_cosim.sh runs the
# identical check against the identical file, and two legs that disagreed about
# what the suite is would be worse than neither of them checking.
if ! "$HERE/check_suite_shape.sh" "$ASM_DIR" "$OBSERVED_FLOOR"; then
  echo "error: the .S suite does not match its manifest; nothing was run." >&2
  exit 1
fi

# A malformed line is named rather than skipped: a line this cannot parse is a
# floor that is not enforced.
floors=$(sed -e 's/#.*//' "$OBSERVED_FLOOR" | awk 'NF { $1=$1; print }')
malformed_floor=$(printf '%s\n' "$floors" | awk 'NF && (NF != 3 || $2 !~ /^[0-9]+$/ || $3 !~ /^[0-9]+$/) { print }')
if [ -n "$malformed_floor" ]; then
  echo "error: $OBSERVED_FLOOR has lines that are not '<test>.S <retires> <spec-checked>':" >&2
  printf '  %s\n' "$malformed_floor" >&2
  exit 1
fi

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
# mktemp is fatal here: an mktemp that exits 0 with empty output would collapse
# every artifact path to /<base>.hex and defeat the trap below.
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
  # Cleared per iteration: a carried-over value would let a run that printed no
  # counts be graded against its predecessor's.
  retires=""
  spec_retires=""
  if ! "$CC" -march=rv32imc_zicsr_zifencei -mabi=ilp32 -nostdlib -I "$ASM_DIR" \
       -T "$ASM_DIR/sections.lds" "$src" -o "$elf" > "$build_log" 2>&1; then
    status="ASSEMBLE-ERROR"
  elif [ -s "$build_log" ]; then
    status="ASSEMBLE-WARNING"
  fi

  # The empty case is the dangerous one and gets its own label: an empty RAM
  # image parses fine, `tohost` reads zero, and every test with no data
  # dependency still reaches RVTEST_PASS (ADR-0035).
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
    # Lifted rather than guarded with `|| sim_status=$?` because bash 3.2
    # (macOS /bin/bash) rewrites a 127 to 1 under errexit, which would report an
    # unstartable runner as a FAIL — a verdict about the CPU.
    set +e
    "$SIM" --rom "$rom_hex" --ram "$ram_hex" --cycles "$CYCLES" \
      > "$tmp/$base.run.log" 2>&1
    sim_status=$?
    set -e
    # The exit ladder is test/cxxrtl.cc's. 4, 5 and 6 each get their own label
    # because lumping any into RUNNER-ERROR reads as "the sim would not start"
    # when it means the oracle disagreed, a trap preceded its handler, or the
    # oracle never looked.
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

    set -- $(awk '/^RETIRES /{print $2, $4; exit}' "$tmp/$base.run.log")
    retires=${1:-}
    spec_retires=${2:-}
  fi

  # A PASS with no counts line means the binary this gate ran is not the runner
  # it grades, and every observation check below would then silently not run.
  if [ "$status" = "PASS" ] && { [ -z "$retires" ] || [ -z "$spec_retires" ]; }; then
    status="NO-COUNTS"
  fi

  # `>=` and not set equality: the counts move for legitimate reasons, and the
  # thing that must not happen is a program going quiet. Only PASS programs are
  # graded — anything already failing has a more specific status.
  if [ "$status" = "PASS" ]; then
    floor=$(printf '%s\n' "$floors" | awk -v n="$name" '$1 == n { print $2, $3; found = 1 } END { exit !found }') || floor=""
    if [ -z "$floor" ]; then
      # Unreachable while check_suite_shape.sh runs above, and kept anyway: a
      # silent empty `$floor` would make the comparisons below compare against
      # nothing. If it fires, the two checks have gone out of step.
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
  # The counts stay outside the name-and-status pair the baseline matches on:
  # putting a quantity that legitimately moves into the failure set would make
  # every baseline entry unmatchable the first time it moved.
  table+=("$(printf '%-16s %-22s %s' "$name" "$status" \
    "retires=${retires:--} spec-checked=${spec_retires:--}")")
done

printf '%s\n' "${table[@]}"
echo
echo "$passed/${#table[@]} passed"

# `NF` must gate the rebuild rather than follow it: awk forces NF to 1 when $1
# is assigned, so `{$1=$1} NF` would resurrect every blank and comment-only line
# as an empty entry.
actual_sorted=$(printf '%s\n' "${failures[@]:-}" | awk 'NF { $1=$1; print }' | sort)
expected_sorted=$(sed -e 's/#.*//' "$EXPECTED_FAIL" | awk 'NF { $1=$1; print }' | sort)

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

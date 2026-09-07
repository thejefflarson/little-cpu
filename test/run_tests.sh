#!/bin/bash
# Builds every program in test/asm, runs it under the cxxrtl runner (`sim`), and grades
# the pass/fail table against test/EXPECTED_FAIL. Invoked by `make test`.
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
STALL_REPORT=${STALL_REPORT:-0}

# A baseline that is missing or cannot be read gives an empty expected set.
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

# Runs before anything is assembled.
if ! "$HERE/check_suite_shape.sh" "$ASM_DIR" "$OBSERVED_FLOOR"; then
  echo "error: the suite does not match its manifest; nothing was run." >&2
  exit 1
fi

# A line this cannot parse is a floor nothing enforces, so say so rather than skipping
# it.
floors=$(sed -e 's/#.*//' "$OBSERVED_FLOOR" | awk 'NF { $1=$1; print }')
malformed_floor=$(printf '%s\n' "$floors" | awk 'NF && (NF != 3 || $2 !~ /^[0-9]+$/ || $3 !~ /^[0-9]+$/) { print }')
if [ -n "$malformed_floor" ]; then
  echo "error: $OBSERVED_FLOOR has lines that are not '<program> <retires> <spec-checked>':" >&2
  printf '  %s\n' "$malformed_floor" >&2
  exit 1
fi

# A C program's floor is a silence bound, not an observation: its instruction count is
# whatever that gcc chose to inline, and the two toolchains this repo builds with
# disagree by about 1% on identical source.
C_FLOOR_MAX=64
overspecified=$(printf '%s\n' "$floors" \
  | awk -v max="$C_FLOOR_MAX" '$1 ~ /\.c$/ && ($2 > max || $3 > max) { print }')
if [ -n "$overspecified" ]; then
  echo "error: $OBSERVED_FLOOR gives a C program a floor above $C_FLOOR_MAX:" >&2
  printf '  %s\n' "$overspecified" >&2
  echo "That is a measurement of one compiler's output, not a bound on the" >&2
  echo "monitor going quiet, and it goes red when the other toolchain builds" >&2
  echo "the same source. Read the section on C programs in that file." >&2
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
# Stop if mktemp gave us nothing.
if [ -z "$tmp" ] || [ ! -d "$tmp" ]; then
  echo "error: mktemp -d produced no usable directory under ${TMPDIR:-/tmp}." >&2
  exit 1
fi
trap 'rm -rf "$tmp"' EXIT

declare -a failures=()
declare -a table=()
passed=0
stall_counts="$tmp/stall_counts"
: > "$stall_counts"
# A bare word with no spaces, so it is expanded unquoted below.
sim_stall_flag=""
if [ "$STALL_REPORT" = "1" ]; then
  sim_stall_flag="--stalls"
fi

shopt -s nullglob
programs=("$ASM_DIR"/*.S "$ASM_DIR"/*.c)
shopt -u nullglob

for src in "${programs[@]}"; do
  name=$(basename "$src")
  base=${name%.*}
  elf="$tmp/$base.elf"
  build_log="$tmp/$base.build.log"
  rom_hex="$tmp/$base.rom.hex"
  ram_hex="$tmp/$base.ram.hex"

  case $name in
    *.c)
      build=("$CC" -march=rv32imac_zicsr_zifencei_zkt -mabi=ilp32 -nostdlib
             -Os -std=c11 -ffreestanding -fno-tree-loop-distribute-patterns
             -Wall -Wextra -Werror -I "$ASM_DIR"
             -T "$ASM_DIR/boot.lds" "$HERE/crt0.S" "$src" -o "$elf")
      rom_flags=(--only-section=.text --only-section=.data)
      ram_flags=(--only-section=.tohost)
      ;;
    *)
      build=("$CC" -march=rv32imac_zicsr_zifencei_zkt -mabi=ilp32 -nostdlib
             -I "$ASM_DIR" -T "$ASM_DIR/sections.lds" "$src" -o "$elf")
      rom_flags=(--only-section=.text)
      ram_flags=(--remove-section=.text)
      ;;
  esac

  status="PASS"
  retires=""
  spec_retires=""
  if ! "${build[@]}" > "$build_log" 2>&1; then
    status="ASSEMBLE-ERROR"
  elif [ -s "$build_log" ]; then
    status="ASSEMBLE-WARNING"
  fi

  if [ "$status" = "PASS" ]; then
    for region in rom ram; do
      if [ "$region" = rom ]; then
        section_flags=("${rom_flags[@]}")
        image=$rom_hex
      else
        section_flags=("${ram_flags[@]}")
        image=$ram_hex
      fi
      rm -f "$image"
      if ! "$OBJCOPY" -O verilog --verilog-data-width=4 "${section_flags[@]}" \
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
    set +e
    "$SIM" --rom "$rom_hex" --ram "$ram_hex" --cycles "$CYCLES" $sim_stall_flag \
      > "$tmp/$base.run.log" 2>&1
    sim_status=$?
    set -e
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

    if [ "$STALL_REPORT" = "1" ]; then
      stall_line=$(awk '/^STALLS /{$1=""; print; exit}' "$tmp/$base.run.log")
      if [ -n "$stall_line" ]; then
        echo "$name $stall_line retires=${retires:-0}" >> "$stall_counts"
      fi
    fi
  fi

  if [ "$status" = "PASS" ] && { [ -z "$retires" ] || [ -z "$spec_retires" ]; }; then
    status="NO-COUNTS"
  fi

  if [ "$status" = "PASS" ]; then
    floor=$(printf '%s\n' "$floors" | awk -v n="$name" '$1 == n { print $2, $3; found = 1 } END { exit !found }') || floor=""
    if [ -z "$floor" ]; then
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
  table+=("$(printf '%-16s %-22s %s' "$name" "$status" \
    "retires=${retires:--} spec-checked=${spec_retires:--}")")
done

printf '%s\n' "${table[@]}"
echo
echo "$passed/${#table[@]} passed"

stall_report_status=0
if [ "$STALL_REPORT" = "1" ]; then
  set +e
  python3 "$HERE/stall_report.py" "$stall_counts"
  stall_report_status=$?
  set -e
fi

actual_sorted=$(printf '%s\n' "${failures[@]:-}" | awk 'NF { $1=$1; print }' | sort)
expected_sorted=$(sed -e 's/#.*//' "$EXPECTED_FAIL" | awk 'NF { $1=$1; print }' | sort)

malformed=$(printf '%s\n' "$expected_sorted" | awk 'NF == 1 {print}')
if [ -n "$malformed" ]; then
  echo "error: $EXPECTED_FAIL has entries with no status (the format is" >&2
  echo "'<test>.S <STATUS>', e.g. 'trap.S MONITOR-ERROR 101'):" >&2
  printf '  %s\n' "$malformed" >&2
  exit 1
fi

if [ "$actual_sorted" = "$expected_sorted" ]; then
  echo "Failure list matches $EXPECTED_FAIL exactly (name and status)."
  exit $stall_report_status
fi

echo
echo "Failure list does NOT match $EXPECTED_FAIL:" >&2
diff <(echo "$expected_sorted") <(echo "$actual_sorted") \
  --label expected --label actual >&2 || true
exit 1

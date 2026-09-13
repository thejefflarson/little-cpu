#!/bin/bash
# Builds every program in nano/asm, runs it under nano's cxxrtl runner (nano-sim), and
# grades the pass/fail table against nano/asm/EXPECTED_FAIL -- the same shape as
# test/run_tests.sh, over nano's own march/mabi/linker rather than littlecpu's, since the
# two cores share no register file width, bus, or crt0 to build against in common.
set -euo pipefail

if [ "$#" -ne 5 ]; then
  echo "usage: run_nano_tests.sh <sim-binary> <asm-dir> <expected-fail-file> <floor-file> <cflags>" >&2
  exit 1
fi

SIM=$1
ASM_DIR=$2
EXPECTED_FAIL=$3
OBSERVED_FLOOR=$4
CFLAGS=$5
CYCLES=5000
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)

if [ ! -f "$EXPECTED_FAIL" ] || [ ! -r "$EXPECTED_FAIL" ]; then
  echo "error: baseline '$EXPECTED_FAIL' does not exist or is not readable." >&2
  exit 1
fi

if [ ! -f "$OBSERVED_FLOOR" ] || [ ! -r "$OBSERVED_FLOOR" ]; then
  echo "error: floor file '$OBSERVED_FLOOR' does not exist or is not readable." >&2
  exit 1
fi

if ! "$REPO/test/check_suite_shape.sh" "$ASM_DIR" "$OBSERVED_FLOOR"; then
  echo "error: nano's suite does not match its manifest; nothing was run." >&2
  exit 1
fi

floors=$(sed -e 's/#.*//' "$OBSERVED_FLOOR" | awk 'NF { $1=$1; print }')
malformed_floor=$(printf '%s\n' "$floors" | awk 'NF && (NF != 2 || $2 !~ /^[0-9]+$/) { print }')
if [ -n "$malformed_floor" ]; then
  echo "error: $OBSERVED_FLOOR has lines that are not '<program> <retires>':" >&2
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
  exit 1
fi

if [ ! -x "$SIM" ]; then
  echo "error: '$SIM' is not an executable runner; build it with 'make nano-sim'." >&2
  exit 1
fi

tmp=$(mktemp -d "${TMPDIR:-/tmp}/nanocpu-test.XXXXXX") || {
  echo "error: could not create a temporary directory under ${TMPDIR:-/tmp}." >&2
  exit 1
}
trap 'rm -rf "$tmp"' EXIT

declare -a failures=()
declare -a table=()
passed=0

shopt -s nullglob
programs=("$ASM_DIR"/*.S)
shopt -u nullglob

for src in "${programs[@]}"; do
  name=$(basename "$src")
  base=${name%.*}
  elf="$tmp/$base.elf"
  build_log="$tmp/$base.build.log"
  rom_hex="$tmp/$base.rom.hex"
  ram_hex="$tmp/$base.ram.hex"

  status="PASS"
  retires=""
  # shellcheck disable=SC2086
  if ! "$CC" $CFLAGS -nostdlib -I "$ASM_DIR" -I "$REPO/test/asm" \
       -T "$ASM_DIR/nano.lds" "$src" -o "$elf" > "$build_log" 2>&1; then
    status="ASSEMBLE-ERROR"
  elif [ -s "$build_log" ]; then
    status="ASSEMBLE-WARNING"
  fi

  if [ "$status" = "PASS" ]; then
    if ! "$OBJCOPY" -O verilog --verilog-data-width=4 --only-section=.text \
         "$elf" "$rom_hex" >> "$build_log" 2>&1 || [ ! -s "$rom_hex" ]; then
      status="OBJCOPY-ERROR rom"
    elif ! "$OBJCOPY" -O verilog --verilog-data-width=4 --remove-section=.text \
         "$elf" "$ram_hex" >> "$build_log" 2>&1 || [ ! -s "$ram_hex" ]; then
      status="OBJCOPY-ERROR ram"
    fi
  fi

  if [ "$status" = "PASS" ]; then
    set +e
    "$SIM" --rom "$rom_hex" --ram "$ram_hex" --cycles "$CYCLES" > "$tmp/$base.run.log" 2>&1
    sim_status=$?
    set -e
    case $sim_status in
      0) status="PASS" ;;
      1) num=$(awk '/^FAIL/{print $2; exit}' "$tmp/$base.run.log")
         status="FAIL${num:+ $num}" ;;
      2) status="TIMEOUT" ;;
      4) code=$(awk '/RVFI monitor error/{print $4; exit}' "$tmp/$base.run.log")
         status="MONITOR-ERROR${code:+ $code}" ;;
      5) status="TRAP" ;;
      6) status="MONITOR-SILENT" ;;
      *) status="RUNNER-ERROR $sim_status" ;;
    esac
    retires=$(awk '/^RETIRES /{print $2; exit}' "$tmp/$base.run.log")
  fi

  if [ "$status" = "PASS" ] && [ -z "$retires" ]; then
    status="NO-COUNTS"
  fi

  if [ "$status" = "PASS" ]; then
    floor=$(printf '%s\n' "$floors" | awk -v n="$name" '$1 == n { print $2; found = 1 } END { exit !found }') || floor=""
    if [ -z "$floor" ]; then
      status="NO-FLOOR"
    elif [ "$retires" -lt "$floor" ]; then
      status="BELOW-FLOOR retires"
      echo "$name: $retires retires, floor is $floor ($OBSERVED_FLOOR)" >&2
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
  table+=("$(printf '%-16s %-22s retires=%s' "$name" "$status" "${retires:--}")")
done

printf '%s\n' "${table[@]}"
echo
echo "$passed/${#table[@]} passed"

actual_sorted=$(printf '%s\n' "${failures[@]:-}" | awk 'NF { $1=$1; print }' | sort)
expected_sorted=$(sed -e 's/#.*//' "$EXPECTED_FAIL" | awk 'NF { $1=$1; print }' | sort)

malformed=$(printf '%s\n' "$expected_sorted" | awk 'NF == 1 {print}')
if [ -n "$malformed" ]; then
  echo "error: $EXPECTED_FAIL has entries with no status (the format is" >&2
  echo "'<test>.S <STATUS>', e.g. 'divide.S MONITOR-ERROR 105'):" >&2
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

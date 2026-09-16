#!/bin/bash
# Builds every program in ASM_DIR under nano's cxxrtl runner and grades it against
# EXPECTED_FAIL. Nano's own six-program suite and littlecpu's portable subset both use
# this script, the latter with the linker script and cycle budget given explicitly.
set -euo pipefail

if [ "$#" -lt 5 ] || [ "$#" -gt 7 ]; then
  echo "usage: run_nano_tests.sh <sim-binary> <asm-dir> <expected-fail-file>" \
       "<floor-file> <cflags> [lds-path] [cycles]" >&2
  exit 1
fi

SIM=$1
ASM_DIR=$2
EXPECTED_FAIL=$3
OBSERVED_FLOOR=$4
CFLAGS=$5
LDS=${6:-$ASM_DIR/nano.lds}
CYCLES=${7:-5000}
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

if [ ! -f "$LDS" ]; then
  echo "error: linker script '$LDS' does not exist." >&2
  exit 1
fi

if ! "$REPO/test/check_suite_shape.sh" "$ASM_DIR" "$OBSERVED_FLOOR"; then
  echo "error: nano's suite does not match its manifest; nothing was run." >&2
  exit 1
fi

# EXCLUDED, with a reason after it, is the one non-numeric status a second field may
# hold. A numeric floor is capped at 10 digits, so the later `-ge` is always a plain
# integer compare, never a parse error `set -u` could read as "not below the floor".
floors=$(sed -e 's/#.*//' "$OBSERVED_FLOOR" | awk 'NF { $1=$1; print }')
malformed_floor=$(printf '%s\n' "$floors" | awk '
  $2 == "EXCLUDED" { if (NF < 3) print; next }
  NF != 2 || $2 !~ /^[0-9]{1,10}$/ { print }
')
if [ -n "$malformed_floor" ]; then
  echo "error: $OBSERVED_FLOOR has lines that are not '<program> <retires>' or" \
       "'<program> EXCLUDED <reason>':" >&2
  printf '  %s\n' "$malformed_floor" >&2
  exit 1
fi

# This runner globs *.S only, so a non-.S name can only ever be EXCLUDED.
non_s_attempted=$(printf '%s\n' "$floors" | awk '$2 != "EXCLUDED" && $1 !~ /\.S$/ { print $1 }')
if [ -n "$non_s_attempted" ]; then
  echo "error: $OBSERVED_FLOOR gives a non-.S program a real floor; this runner only" >&2
  echo "globs *.S, so it would never be attempted:" >&2
  printf '  %s\n' "$non_s_attempted" >&2
  exit 1
fi

CC=""
if command -v riscv-none-elf-gcc >/dev/null 2>&1; then
  CC=riscv-none-elf-gcc
fi
if [ -z "$CC" ]; then
  echo "error: no RISC-V cross compiler found (want riscv-none-elf-gcc)." >&2
  echo "Run 'make riscv-gcc-setup' to install the pinned one." >&2
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

# Shared by the main loop and the exclusion recheck below: build, objcopy, simulate.
build_and_run() {  # $1 = src, $2 = base name for this attempt's tmp files
  local src=$1 base=$2 status retires elf build_log rom_hex ram_hex sim_status num code
  elf="$tmp/$base.elf"
  build_log="$tmp/$base.build.log"
  rom_hex="$tmp/$base.rom.hex"
  ram_hex="$tmp/$base.ram.hex"
  status="PASS"
  retires=""

  # shellcheck disable=SC2086
  if ! "$CC" $CFLAGS -nostdlib -I "$ASM_DIR" -I "$REPO/test/asm" \
       -T "$LDS" "$src" -o "$elf" > "$build_log" 2>&1; then
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
      7) num=$(awk '/^X reached a retiring instruction.*cycle/{print $NF; exit}' "$tmp/$base.run.log")
         status="X-REACHED${num:+ $num}" ;;
      *) status="RUNNER-ERROR $sim_status" ;;
    esac
    retires=$(awk '/^RETIRES /{print $2; exit}' "$tmp/$base.run.log")
  fi

  if [ "$status" = "PASS" ] && ! printf '%s' "$retires" | grep -qE '^[0-9]{1,10}$'; then
    status="NO-COUNTS"
  fi

  printf '%s\t%s\t%s\n' "$status" "$retires" "$build_log"
}

declare -a failures=()
declare -a table=()
declare -a excluded_names=()
passed=0

shopt -s nullglob
programs=("$ASM_DIR"/*.S)
shopt -u nullglob

for src in "${programs[@]}"; do
  name=$(basename "$src")
  floor=$(printf '%s\n' "$floors" | awk -v n="$name" '$1 == n { print $2; found = 1 } END { exit !found }') || floor=""

  if [ "$floor" = "EXCLUDED" ]; then
    excluded_names+=("$name")
    continue
  fi

  base=${name%.*}
  attempt=$(build_and_run "$src" "$base")
  IFS=$'\t' read -r status retires build_log <<< "$attempt"

  if [ "$status" = "PASS" ]; then
    if [ -z "$floor" ]; then
      status="NO-FLOOR"
    elif [ "$retires" -ge "$floor" ]; then
      : # meets its floor
    else
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

if [ "${#table[@]}" -eq 0 ]; then
  echo "error: this run attempted zero programs. Every entry in $OBSERVED_FLOOR is" >&2
  echo "EXCLUDED (or the manifest is empty), so nothing here tests anything." >&2
  exit 1
fi

# An EXCLUDED program that still assembles and passes here is a stale exclusion.
excluded_but_passed=()
for name in "${excluded_names[@]:-}"; do
  [ -n "$name" ] || continue
  status=$(build_and_run "$ASM_DIR/$name" "excluded-$name" | cut -f1)
  if [ "$status" = "PASS" ]; then
    excluded_but_passed+=("$name")
  fi
done
if [ "${#excluded_but_passed[@]}" -gt 0 ]; then
  echo "error: these programs are marked EXCLUDED in $OBSERVED_FLOOR but assembled" >&2
  echo "and passed under this exact toolchain, so the exclusion is stale:" >&2
  printf '  %s\n' "${excluded_but_passed[@]}" >&2
  exit 1
fi

printf '%s\n' "${table[@]}"
echo
if [ "${#excluded_names[@]}" -gt 0 ]; then
  echo "$passed/${#table[@]} passed (${#excluded_names[@]} excluded)"
else
  echo "$passed/${#table[@]} passed"
fi

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

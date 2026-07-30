#!/bin/bash
# Assembles every test/asm/*.S file, runs it under the cxxrtl runner (`sim`),
# and checks the resulting pass/fail table against test/EXPECTED_FAIL — the
# sprint-1 baseline (ADR-0007, ADR-0008). Invoked by `make test`.
#
# Usage: run_tests.sh <sim-binary> <asm-dir> <expected-fail-file>
set -u

SIM=$1
ASM_DIR=$2
EXPECTED_FAIL=$3
CYCLES=5000

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
OBJCOPY=${CC%gcc}objcopy

tmp=$(mktemp -d "${TMPDIR:-/tmp}/littlecpu-test.XXXXXX")
trap 'rm -rf "$tmp"' EXIT

declare -a failed_tests=()
declare -a table=()
passed=0

for src in "$ASM_DIR"/*.S; do
  name=$(basename "$src")
  base=${name%.S}
  elf="$tmp/$base.elf"
  asm_log="$tmp/$base.asm.log"

  status="PASS"
  if ! "$CC" -march=rv32imc_zicsr -mabi=ilp32 -nostdlib -I "$ASM_DIR" \
       -T "$ASM_DIR/sections.lds" "$src" -o "$elf" > "$asm_log" 2>&1; then
    status="ASSEMBLE-ERROR"
  elif [ -s "$asm_log" ]; then
    # Any assembler output (warnings included) on an otherwise-successful
    # build is still a defect per criterion 7 — zero assembler warnings.
    status="ASSEMBLE-WARNING"
  fi

  if [ "$status" = "PASS" ]; then
    rom_hex="$tmp/$base.rom.hex"
    ram_hex="$tmp/$base.ram.hex"
    "$OBJCOPY" -O verilog --verilog-data-width=4 --only-section=.text \
      "$elf" "$rom_hex" 2>>"$asm_log"
    "$OBJCOPY" -O verilog --verilog-data-width=4 --remove-section=.text \
      "$elf" "$ram_hex" 2>>"$asm_log"

    "$SIM" --rom "$rom_hex" --ram "$ram_hex" --cycles "$CYCLES" \
      > "$tmp/$base.run.log" 2>&1
    case $? in
      0) status="PASS" ;;
      1) status="FAIL $(awk '/^FAIL/{print $2}' "$tmp/$base.run.log")" ;;
      2) status="TIMEOUT" ;;
      *) status="RUNNER-ERROR" ;;
    esac
  fi

  if [ "$status" = "PASS" ]; then
    passed=$((passed + 1))
  else
    failed_tests+=("$name")
    if [ -s "$asm_log" ]; then
      echo "--- $name assembler output ---" >&2
      cat "$asm_log" >&2
    fi
  fi
  table+=("$(printf '%-16s %s' "$name" "$status")")
done

printf '%s\n' "${table[@]}"
echo
echo "$passed/${#table[@]} passed"

actual_sorted=$(printf '%s\n' "${failed_tests[@]:-}" | sed '/^$/d' | sort)
expected_sorted=$(sed -e 's/#.*//' -e '/^[[:space:]]*$/d' "$EXPECTED_FAIL" | sort)

if [ "$actual_sorted" = "$expected_sorted" ]; then
  echo "Failure list matches $EXPECTED_FAIL exactly."
  exit 0
fi

echo
echo "Failure list does NOT match $EXPECTED_FAIL:" >&2
diff <(echo "$expected_sorted") <(echo "$actual_sorted") \
  --label expected --label actual >&2
exit 1

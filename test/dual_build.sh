#!/bin/bash
# Assembles and links every program in test/dual/ and checks it against
# test/dual/MUTATION_PAIRINGS, in both directions.
#
# WHAT THIS IS AND IS NOT. Nothing in this tree instantiates two harts, so no
# program in test/dual/ has ever executed and this script does not pretend
# otherwise: it does not run a simulator and it reports no pass/fail for any
# hardware property. What it grades is that the source still assembles at the
# suite's own ISA string, links into the suite's own map, produces both images a
# runner would need, and is still named by the pairing that claims it catches
# something. Four programs nothing builds would rot silently between now and the
# day the dual runner lands, and the pairings would rot with them.
#
# The build flags are test/run_tests.sh's `.S` arm verbatim, plus this
# directory on the include path. When that arm changes this has to change with
# it -- the same three-places rule the two program shapes already carry.
#
# Usage: dual_build.sh <dual-dir> <asm-dir> <pairings-file>
set -euo pipefail

if [ "$#" -ne 3 ]; then
  echo "usage: dual_build.sh <dual-dir> <asm-dir> <pairings-file>" >&2
  exit 1
fi

DUAL_DIR=$1
ASM_DIR=$2
PAIRINGS=$3

if [ ! -d "$DUAL_DIR" ]; then
  echo "error: '$DUAL_DIR' is not a directory. The two-hart programs live there" >&2
  echo "and a missing directory is the failure this whole script exists to see." >&2
  exit 1
fi

if [ ! -f "$PAIRINGS" ] || [ ! -r "$PAIRINGS" ]; then
  echo "error: pairings file '$PAIRINGS' does not exist or is not readable." >&2
  echo "Without it the programs below are claimed to catch nothing." >&2
  exit 1
fi

shopt -s nullglob
programs=("$DUAL_DIR"/*.S)
shopt -u nullglob

if [ "${#programs[@]}" -eq 0 ]; then
  echo "error: '$DUAL_DIR' contains no .S programs." >&2
  exit 1
fi

# The set check, before anything is built. `prog` legs name a program each; the
# two sets must match, so a program that grades nothing and a pairing that names
# a program nobody wrote are both red. A mutation with no `prog` leg is a real
# entry and not an omission -- starvation is one -- so only the `prog` lines
# take part.
claimed=$(sed -e 's/#.*//' "$PAIRINGS" | awk '$2 == "prog" { print $3 }' | sort -u)
present=$(printf '%s\n' "${programs[@]}" | xargs -n1 basename | sort -u)

missing=$(comm -23 <(printf '%s\n' "$claimed") <(printf '%s\n' "$present"))
unclaimed=$(comm -13 <(printf '%s\n' "$claimed") <(printf '%s\n' "$present"))

if [ -n "$missing" ]; then
  echo "error: $PAIRINGS names programs that do not exist in $DUAL_DIR:" >&2
  printf '  %s\n' $missing >&2
  exit 1
fi
if [ -n "$unclaimed" ]; then
  echo "error: $DUAL_DIR holds programs no pairing in $PAIRINGS claims:" >&2
  printf '  %s\n' $unclaimed >&2
  echo "A torture program nothing is paired against is a program nobody has" >&2
  echo "said what it catches. Add the pairing or delete the program." >&2
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

tmp=$(mktemp -d "${TMPDIR:-/tmp}/littlecpu-dual.XXXXXX") || {
  echo "error: could not create a temporary directory under ${TMPDIR:-/tmp}." >&2
  exit 1
}
if [ -z "$tmp" ] || [ ! -d "$tmp" ]; then
  echo "error: mktemp -d produced no usable directory under ${TMPDIR:-/tmp}." >&2
  exit 1
fi
trap 'rm -rf "$tmp"' EXIT

status=0
for src in "${programs[@]}"; do
  name=$(basename "$src")
  base=${name%.*}
  elf="$tmp/$base.elf"
  log="$tmp/$base.build.log"

  verdict="BUILT"
  if ! "$CC" -march=rv32imac_zicsr_zifencei -mabi=ilp32 -nostdlib \
       -I "$ASM_DIR" -I "$DUAL_DIR" -T "$ASM_DIR/sections.lds" \
       "$src" -o "$elf" > "$log" 2>&1; then
    verdict="ASSEMBLE-ERROR"
  elif [ -s "$log" ]; then
    verdict="ASSEMBLE-WARNING"
  fi

  if [ "$verdict" = "BUILT" ]; then
    for region in rom ram; do
      if [ "$region" = rom ]; then
        section_flags=(--only-section=.text)
      else
        section_flags=(--remove-section=.text)
      fi
      image="$tmp/$base.$region.hex"
      rm -f "$image"
      if ! "$OBJCOPY" -O verilog --verilog-data-width=4 "${section_flags[@]}" \
           "$elf" "$image" >> "$log" 2>&1; then
        verdict="OBJCOPY-ERROR $region"
        break
      fi
      # An empty image is the quiet failure: it still parses, so a runner would
      # start and every check that reads RAM would see zero.
      if [ ! -s "$image" ]; then
        verdict="OBJCOPY-EMPTY $region"
        break
      fi
    done
  fi

  printf '%-20s %s\n' "$name" "$verdict"
  if [ "$verdict" != "BUILT" ]; then
    sed -e 's/^/    /' "$log" >&2
    status=1
  fi
done

if [ "$status" -ne 0 ]; then
  echo "error: at least one two-hart program does not build." >&2
  exit 1
fi

echo "${#programs[@]} two-hart programs build and are paired. None of them has run:"
echo "nothing in this tree instantiates two harts."

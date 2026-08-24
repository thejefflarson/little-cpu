#!/bin/bash
# Assembles and links every program in test/dual/ and checks it against
# test/dual/MUTATION_PAIRINGS, in both directions.
#
# WHAT THIS IS AND IS NOT. It does not run a simulator and reports no pass/fail
# for any hardware property. What it grades is that the source still assembles
# at the suite's own ISA string, links into the suite's own map, produces both
# images a runner would need, and is still named by the pairing that claims it
# catches something. Four programs nothing builds would rot silently, and the
# pairings would rot with them.
#
# The dual top, its harness and its runner now exist, so `make dual-smoke` does
# execute one program here. That is the whole of what has run: no torture
# program has, and no mutation below has been applied. A program the runner
# grades directly is EXEMPT rather than paired -- see the exemption block below,
# which is checked in both directions so an exemption outliving its grader is
# red.
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
present=$(for p in "${programs[@]}"; do printf '%s\n' "${p##*/}"; done | sort -u)

# An EXEMPT line names a program the dual RUNNER grades directly, so no mutation
# pairing claims it and none should. It must name its grader, and the program
# must exist: an exemption for a program nobody wrote is how a deleted grader
# stops being noticed.
exempt=$(sed -e 's/#.*//' "$PAIRINGS" | awk '$1 == "EXEMPT" { print $2 }' | sort -u)
stale_exempt=$(comm -23 <(printf '%s\n' "$exempt") <(printf '%s\n' "$present"))
if [ -n "$stale_exempt" ]; then
  echo "error: $PAIRINGS exempts programs that do not exist in $DUAL_DIR:" >&2
  printf '%s\n' "$stale_exempt" | sed -e 's|^|  |' >&2
  echo "An exemption naming nothing is an exemption nobody can check. Delete it" >&2
  echo "with the program, or restore the program it was written for." >&2
  exit 1
fi
double=$(comm -12 <(printf '%s\n' "$exempt") <(printf '%s\n' "$claimed"))
if [ -n "$double" ]; then
  echo "error: $PAIRINGS both exempts and pairs:" >&2
  printf '%s\n' "$double" | sed -e 's|^|  |' >&2
  echo "A program is graded by the runner or by a mutation pairing, not by an" >&2
  echo "exemption AND a pairing -- the exemption would hide the pairing rotting." >&2
  exit 1
fi

missing=$(comm -23 <(printf '%s\n' "$claimed") <(printf '%s\n' "$present"))
unclaimed=$(comm -13 <(printf '%s\n' "$claimed") <(printf '%s\n' "$present") \
            | comm -23 - <(printf '%s\n' "$exempt"))

if [ -n "$missing" ]; then
  echo "error: $PAIRINGS names programs that do not exist in $DUAL_DIR:" >&2
  printf '%s\n' "$missing" | sed -e 's|^|  |' >&2
  exit 1
fi
if [ -n "$unclaimed" ]; then
  echo "error: $DUAL_DIR holds programs no pairing in $PAIRINGS claims:" >&2
  printf '%s\n' "$unclaimed" | sed -e 's|^|  |' >&2
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
  name=${src##*/}
  base=${name%.*}
  elf="$tmp/$base.elf"
  log="$tmp/$base.build.log"

  verdict="BUILT"
  if ! "$CC" -march=rv32imac_zicsr_zifencei_zkt -mabi=ilp32 -nostdlib \
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

n_exempt=$(printf '%s\n' "$exempt" | grep -c . || true)
echo "${#programs[@]} two-hart programs build; $((${#programs[@]} - n_exempt)) are paired against a"
echo "mutation and $n_exempt exempt to the runner. NO TORTURE PROGRAM HAS RUN and no"
echo "mutation below has been applied, so every pairing is still a prediction."

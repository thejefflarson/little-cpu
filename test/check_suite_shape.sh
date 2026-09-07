#!/bin/bash
# Asserts that the suite CONTAINS what it is supposed to contain, before either sim leg
# runs a single program.
set -euo pipefail

if [ "$#" -ne 2 ]; then
  echo "usage: check_suite_shape.sh <asm-dir> <manifest>" >&2
  exit 1
fi

ASM_DIR=$1
MANIFEST=$2

if [ ! -d "$ASM_DIR" ]; then
  echo "error: asm directory '$ASM_DIR' does not exist." >&2
  exit 1
fi

if [ ! -f "$MANIFEST" ] || [ ! -r "$MANIFEST" ]; then
  echo "error: suite manifest '$MANIFEST' does not exist or is not readable." >&2
  echo "It is the list of programs the suite must contain; without it there is" >&2
  echo "no way to tell a passing suite from a shrunken one." >&2
  exit 1
fi

listed=$(sed -e 's/#.*//' "$MANIFEST" | awk 'NF { print $1 }')

if [ -z "$listed" ]; then
  echo "error: suite manifest '$MANIFEST' names no programs." >&2
  echo "An empty manifest matches an empty suite and reports success." >&2
  exit 1
fi

not_a_program=$(printf '%s\n' "$listed" | awk '$1 !~ /\.[Sc]$/ { print }')
if [ -n "$not_a_program" ]; then
  echo "error: $MANIFEST has entries whose first field is not a '<test>.S' or" >&2
  echo "'<test>.c' name:" >&2
  printf '%s\n' "$not_a_program" | sed -e 's|^|  |' >&2
  exit 1
fi

duplicates=$(printf '%s\n' "$listed" | sort | uniq -d)
if [ -n "$duplicates" ]; then
  echo "error: $MANIFEST names the same program more than once:" >&2
  printf '%s\n' "$duplicates" | sed -e 's|^|  |' >&2
  exit 1
fi

shopt -s nullglob
programs=("$ASM_DIR"/*.S "$ASM_DIR"/*.c)
shopt -u nullglob
if [ "${#programs[@]}" -eq 0 ]; then
  echo "error: no programs found in '$ASM_DIR'." >&2
  echo "The manifest $MANIFEST names $(printf '%s\n' "$listed" | wc -l | tr -d ' ')." >&2
  exit 1
fi

present=$(for src in "${programs[@]}"; do basename "$src"; done | sort)
listed_sorted=$(printf '%s\n' "$listed" | sort)

missing=$(comm -23 <(printf '%s\n' "$listed_sorted") <(printf '%s\n' "$present"))
unlisted=$(comm -13 <(printf '%s\n' "$listed_sorted") <(printf '%s\n' "$present"))

rc=0

if [ -n "$missing" ]; then
  echo "error: $MANIFEST names programs that are not in $ASM_DIR:" >&2
  printf '%s\n' "$missing" | sed -e 's|^|  |' >&2
  echo "The suite has SHRUNK, or a program was renamed. Nothing was run." >&2
  echo "Remove the manifest line in the same commit that removes the program." >&2
  rc=1
fi

if [ -n "$unlisted" ]; then
  echo "error: $ASM_DIR has programs that $MANIFEST does not name:" >&2
  printf '%s\n' "$unlisted" | sed -e 's|^|  |' >&2
  echo "A program that lands without a manifest entry runs unmeasured. Add its" >&2
  echo "line — name, retire floor, spec-checked floor — in the same commit." >&2
  rc=1
fi

if [ "$rc" -ne 0 ]; then
  exit 1
fi

echo "Suite shape matches $MANIFEST: ${#programs[@]} programs."

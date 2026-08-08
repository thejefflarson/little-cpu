#!/bin/bash
# Asserts that the suite CONTAINS what it is supposed to contain, before either
# sim leg runs a single program.
#
# The suite is test/asm/*.S and test/asm/*.c. Both are programs and both are
# graded; they differ only in how test/run_tests.sh builds them, and nothing
# here needs to know which is which.
#
# Usage: check_suite_shape.sh <asm-dir> <manifest>
#
# WHY THIS EXISTS. `make test` verified that every program it FOUND passed. It
# had no idea how many it should find, and test/EXPECTED_FAIL and
# test/COSIM_EXPECTED_FAIL are both empty, so there was no red entry whose
# disappearance would have said the suite had shrunk. A bad rebase, a directory
# rename, a .gitignore change or a glob that stopped matching would leave the
# gate reporting `12/12 passed`, matching an empty baseline exactly, and exiting
# 0. That is the same hole formal/EXPECTED_CHECKS closes on the formal side: a
# verdict baseline cannot report whether a check stopped existing.
#
# THE MANIFEST IS test/OBSERVED_FLOOR, NOT A NEW FILE. That file already names
# every program in the suite — it has to, since it carries each one's observed
# retire floor — and its header already declares its name set to be a set
# equality in both directions. A second file naming the same programs would be a
# second surface to keep in sync, and the interesting failure would become "the
# two lists disagree", which is worse than the hole it closes. What was actually
# missing is enforcement, in three places, and that is what this script and its
# two call sites supply:
#
#   * test/run_tests.sh checked one direction per-program (NO-FLOOR, and only
#     for programs that had already PASSed) and the other direction AFTER the
#     whole suite had run. A missing program was caught, but only once every
#     surviving program had been assembled and simulated;
#   * test/run_cosim.sh did not read the manifest at all. Its own comment says
#     its program-count guard exists so an empty glob cannot match an empty
#     baseline — which is a guard against a suite of size ZERO, not against a
#     suite that lost most of its programs;
#   * neither leg rejected a manifest line that named a program twice, which
#     would make the set comparison below quietly non-symmetric.
#
# BOTH DIRECTIONS, and the second one is the one worth arguing for. A program
# present in the tree but absent from the manifest is red, so a program that lands
# without being wired into the floor file fails immediately rather than joining
# the suite unmeasured. That is why the ladder's version is bidirectional too.
#
# This script reads ONLY the first field of each manifest line. The remaining
# fields are the floor numbers, and grading those is test/run_tests.sh's job —
# it keeps its own format check for them, because a manifest is a name set and a
# floor is a measurement, and conflating the two would put co-simulation (which
# has no retire counts at all) in the business of validating them.
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

# Same treatment the baselines get, and for the same reason: a mistyped path
# would otherwise yield an empty manifest, and an empty manifest compared
# against a shrunken suite is a check that reports on nothing.
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

# A name that is neither a .S nor a .c is a typo or a stray field, and it would
# show up below as a phantom "missing from the tree" entry rather than as what
# it is.
not_a_program=$(printf '%s\n' "$listed" | awk '$1 !~ /\.[Sc]$/ { print }')
if [ -n "$not_a_program" ]; then
  echo "error: $MANIFEST has entries whose first field is not a '<test>.S' or" >&2
  echo "'<test>.c' name:" >&2
  printf '%s\n' "$not_a_program" | sed -e 's|^|  |' >&2
  exit 1
fi

# Duplicates would make `comm` below non-symmetric: a name listed twice and
# present once still produces an empty difference in both directions, so the
# manifest could name 53 lines covering 52 programs and read as a clean match.
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

#!/bin/bash
# Asserts that every place naming the ISA this suite builds for names the same
# one, and that the two places naming a DIFFERENT ISA on purpose still do.
#
# Usage: march_test.sh [repo-root]     # defaults to this script's parent
#
# WHY THIS EXISTS. The ISA string is stated at six sites and three of them are
# silent when they are wrong. test/run_tests.sh and test/cosim.py's assemble()
# are loud -- the assembler rejects `amoadd.w` outright -- but the Makefile's
# soc-rom target builds test/asm/datainit.c, which uses no atomics and would go
# on building for years at the old string; DHRY_CFLAGS builds a benchmark that
# uses none either; and DHRY_CFLAGS is duplicated VERBATIM into
# soc/depth/cycles.py with nothing anywhere comparing the two. So the failure
# this guards against is not "the build broke": it is two measurements taken of
# two different machines, reported in one table.
#
# WHY IT IS NOT A GREP-AND-REPLACE, IN BOTH DIRECTIONS. Two strings that look
# like the ones above must NOT move with them. formal/checks.cfg's `isa rv32imc`
# and the Makefile's `MONITOR_GEN -i rv32imc` name the instruction set
# riscv-formal GENERATES A SPEC MODEL FOR, and the pinned clone has no model for
# any A encoding -- so widening either produces nothing and makes the generated
# check set describe an ISA it is not checking. Two more are a different ISA on
# purpose: soc/compare/bench.S is deliberately rv32i so the cross-core harness
# can hand one image to a core with no M and no privileged architecture, and
# COMPARE_DHRY_CFLAGS is rv32ic because that is what VexRiscv implements. A
# sweep is wrong at four sites and incomplete at three others.
#
# So the table below states the allowed set and the comparison runs BOTH WAYS,
# like every other table in this repo: a site that stops carrying the string is
# as red as the string appearing where nothing allows it, and an exception whose
# site no longer carries the value it names is red too.
#
# Hermetic: git, grep, sed and awk. No cross compiler, no Sail, no yosys, so
# this runs inside `make test` anywhere.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=${1:-$(cd "$HERE/.." && pwd)}

if [ ! -d "$REPO" ]; then
  echo "error: '$REPO' is not a directory, so there is nothing to scan." >&2
  exit 1
fi

# THE ONE SOURCE. Everything below is graded against this string; changing it
# alone, with no site changed, is red at every site.
#
# `a` is here while `misa` still reads 0x4000_1104, which is deliberate and
# temporary: rtl/ decodes and executes the eleven A instructions, and claiming
# the misa bit is a separate change that has to move rtl/csrs.v and the Sail
# model's `A` key together. Nothing about a `-march` string is a claim about
# misa -- it is what the assembler will accept.
DECLARED_MARCH='rv32imac_zicsr_zifencei'

# The sites, with the exact number of times each states it. An exact count
# rather than "at least one", because two of these files state it twice for two
# program shapes and a deleted arm is the failure that leaves the other one
# looking fine.
required_sites() {
  sed -e 's/#.*//' -e 's/[[:space:]]*$//' -e '/^$/d' <<'SITES'
# The rulebook. The ISA target section quotes the string the suite builds at, so
# the prose and the flags cannot drift apart without this going red.
CLAUDE.md 1

# Two arms, one per program shape: a `.c` program linked through test/crt0.S and
# a freestanding `.S` one. Loud if wrong -- the assembler refuses an atomic.
test/run_tests.sh 2

# test/cosim.py's assemble(), the same two shapes for the reference model's side
# of the comparison. Loud for the same reason.
test/cosim.py 2

# THREE, and two of them are the silent ones: soc-rom's two program shapes build
# test/asm/datainit.c and the `.S` suite for the SoC's ROM image, neither of
# which executes an atomic, and DHRY_CFLAGS compiles a benchmark that does not
# either. A wrong string here changes what is measured, not whether it builds.
Makefile 3

# DHRY_CFLAGS again, copied verbatim into the depth sweep. The two full strings
# are compared below as well, because agreeing about `-march` and disagreeing
# about `-O` would be the same defect one flag over.
soc/depth/cycles.py 1

# The Sail reservation probe. It is not part of the graded suite and no core
# runs it, but it is assembled by the same cross compiler and asks the reference
# model about lr.w/sc.w -- so it is exactly the file that must not be left
# behind at a narrower string.
test/sail/reservation_probe.sh 1
SITES
}

# Every OTHER `-march=` in the tree, as `<path> <ISA>`. A path may be a
# directory ending in `/`, and the ISA may be `(any)` where the point of the
# entry is the file rather than one string in it; `(empty)` is a `-march=` with
# no ISA after it. A path alone teaches the next reader nothing and gets deleted
# by the next person tidying, so each entry says which ISA it names and why.
exception_sites() {
  sed -e 's/#.*//' -e 's/[[:space:]]*$//' -e '/^$/d' <<'EXCEPTIONS'
# Dated decision records, and dated proposals. Both are history, and several of
# them quote the flags a measurement was taken with -- which is the whole value
# of the record. Editing them would rewrite what was measured to agree with a
# later decision.
docs/adr/ (any)
docs/ideas/ (any)

# This file: the declaration above, the table, and the diagnostics. A check has
# to be able to name what it is looking for, and every occurrence here is
# either a shell variable or a pattern rather than a flag.
test/march_test.sh (any)

# The probes that force this check red plant wrong ISA strings in their
# fixtures and quote them in their labels. A probe that cannot name what it is
# planting is not a probe.
test/probe_gates.sh (any)

# soc/compare/bench.S, built for the cross-core harness. VexRiscv there is
# RV32IC with no privileged architecture, and the whole point of that harness is
# that ONE image runs on both cores -- so the image may use only what the
# narrower core has. rv32i, not even rv32ic, because the image is also this
# core's and nothing in it needs the density.
Makefile rv32i

# COMPARE_DHRY_CFLAGS: Dhrystone for the same harness, at the ISA VexRiscv
# actually implements. Quoting this core's number against theirs means compiling
# for each core's own ISA, which is why this one is not the string above.
Makefile rv32ic

# Not a flag at all: a grep pattern that finds the `-march=` in the command line
# the Dhrystone runner PRINTS, so the flags travel with the number. It has no
# ISA after it, which is what the `(empty)` says.
soc/compare/run_dhrystone.sh (empty)
EXCEPTIONS
}

# The look-alikes that must NOT move, as <file> <exact text> <why>. These name
# the ISA riscv-formal generates a spec model for, not the one gcc assembles.
FORMAL_ISA='rv32imc'

tmp=$(mktemp -d "${TMPDIR:-/tmp}/littlecpu-march.XXXXXX") || {
  echo "error: could not create a temporary directory under ${TMPDIR:-/tmp}." >&2
  exit 1
}
trap 'rm -rf "$tmp"' EXIT

required_sites  > "$tmp/required"
exception_sites > "$tmp/exceptions"

# Tracked files only, and git's index rather than a `find` with a prune list,
# for the reason test/retired_term_test.sh gives: what this guards against is a
# string arriving in a commit, and a checkout carries build artifacts,
# downloaded tools and whole agent worktrees a merge cannot bring anything
# through.
if ! git -C "$REPO" ls-files -z > "$tmp/files" 2>/dev/null || [ ! -s "$tmp/files" ]; then
  echo "error: cannot enumerate any tracked files under $REPO. This check reads" >&2
  echo "git's index, because what it guards is a build flag arriving in a" >&2
  echo "commit; a tree git cannot list is a scan of nothing reporting green." >&2
  exit 1
fi

# `/dev/null` first so grep always prefixes the filename, and `-o` so a line
# stating the flag twice is two hits rather than one. No match at all leaves
# grep at 1 and xargs at 123, which is not an error here.
hits=$( (cd "$REPO" && xargs -0 grep -noI -E -e '-march=[A-Za-z0-9_]*' -- /dev/null < "$tmp/files") || true)

rc=0

: > "$tmp/found"
: > "$tmp/unexpected"
: > "$tmp/covered"

# Which exception entry covers a path carrying a value, or nothing. A directory
# entry matches on the `/` it ends with, so `docs/adr/` cannot quietly cover a
# `docs/adrenaline.md` nobody meant to exempt.
match_exception() {  # $1 = path, $2 = value
  local path=$1 value=$2 entry epath evalue
  while IFS= read -r entry; do
    epath=${entry%% *}
    evalue=${entry#* }
    [ "$evalue" = "(any)" ] || [ "$evalue" = "$value" ] || continue
    case "$epath" in
      */) case "$path" in "$epath"*) printf '%s' "$entry"; return 0 ;; esac ;;
      *)  if [ "$path" = "$epath" ]; then printf '%s' "$entry"; return 0; fi ;;
    esac
  done < "$tmp/exceptions"
  return 0
}

while IFS= read -r hit; do
  [ -n "$hit" ] || continue
  path=${hit%%:*}
  rest=${hit#*:}
  line=${rest%%:*}
  value=${hit##*-march=}
  [ -n "$value" ] || value='(empty)'

  if [ "$value" = "$DECLARED_MARCH" ]; then
    printf '%s\n' "$path" >> "$tmp/found"
    continue
  fi
  entry=$(match_exception "$path" "$value")
  if [ -n "$entry" ]; then
    printf '%s\n' "$entry" >> "$tmp/covered"
    continue
  fi
  printf '%s:%s: -march=%s\n' "$path" "$line" "$value" >> "$tmp/unexpected"
done <<< "$hits"

if [ -s "$tmp/unexpected" ]; then
  rc=1
  echo "error: an -march= that is neither the declared ISA nor a named exception:" >&2
  sed -e 's|^|  |' "$tmp/unexpected" >&2
  echo >&2
  echo "This suite builds at -march=$DECLARED_MARCH, stated once in" >&2
  echo "test/march_test.sh and graded at every site from there. If the site" >&2
  echo "above deliberately names a DIFFERENT instruction set -- the cross-core" >&2
  echo "harness compiling for the narrower core, a pattern that is not a flag" >&2
  echo "-- add it to the exception list in this file, with which ISA it names" >&2
  echo "and why that one. An entry that is only a path is one the next person" >&2
  echo "tidying will delete." >&2
fi

while read -r path want; do
  got=$(grep -cxF -- "$path" "$tmp/found" || true)
  if [ "$got" -ne "$want" ]; then
    rc=1
    echo >&2
    echo "error: $path states -march=$DECLARED_MARCH $got time(s), not $want." >&2
    echo "Every site below builds something this suite measures, and three of" >&2
    echo "them build programs that use no atomic at all -- so a site left behind" >&2
    echo "goes on producing numbers for a machine nobody asked about rather" >&2
    echo "than failing. If the count moved for a good reason, move it here in" >&2
    echo "the same commit, with the reason above the line." >&2
  fi
done < "$tmp/required"

while IFS= read -r entry; do
  if ! grep -qxF -- "$entry" "$tmp/covered"; then
    rc=1
    echo >&2
    echo "error: the exception list names \`$entry\`, and it is not" >&2
    echo "there any more. Delete the entry -- or if that build moved, move the" >&2
    echo "entry with it. An exemption kept past its reason is how the next one" >&2
    echo "gets waved through, and it is why this comparison runs both ways." >&2
  fi
done < "$tmp/exceptions"

# ---- the two spellings that must NOT move ----------------------------------
#
# Read as their own patterns rather than as `-march=` hits, because that is how
# they are spelled: neither is a compiler flag.

if ! grep -qE "^[[:space:]]*isa[[:space:]]+$FORMAL_ISA[[:space:]]*$" "$REPO/formal/checks.cfg"; then
  rc=1
  echo >&2
  echo "error: formal/checks.cfg no longer says \`isa $FORMAL_ISA\`." >&2
  echo "That line picks which instructions riscv-formal GENERATES a spec model" >&2
  echo "for, and the pinned clone has no model for any A encoding: its" >&2
  echo "insn_amo generator has every call site commented out, it covers neither" >&2
  echo "min/max nor LR/SC, and there is no isa_rv32ia*.txt. Widening it" >&2
  echo "generates nothing and leaves the check set claiming an ISA it does not" >&2
  echo "check. formal/COMPLETE_EXCLUSIONS is where that boundary is recorded," >&2
  echo "and formal/check-complete-exclusions.py re-derives it from the clone." >&2
fi

if ! grep -qE -- "-i[[:space:]]+$FORMAL_ISA([[:space:]]|$)" "$REPO/Makefile"; then
  rc=1
  echo >&2
  echo "error: the Makefile's MONITOR_GEN no longer passes \`-i $FORMAL_ISA\`." >&2
  echo "It generates test/monitor.v, the per-retire oracle both sim legs read," >&2
  echo "from the same pinned clone as the line above and with the same gap in" >&2
  echo "it. Widening this regenerates the same file and changes nothing except" >&2
  echo "what the command line claims." >&2
fi

# ---- the Dhrystone flags, whole -------------------------------------------
#
# Not just the ISA: the two copies of DHRY_CFLAGS are one measurement's other
# half, and agreeing about -march while disagreeing about -O would be the same
# defect one flag over.

# One line, single-spaced and untrimmed at both ends, so the two copies are
# compared on what they SAY rather than on how they were wrapped: one is a make
# variable over three backslash-continued lines and the other is a Python
# implicit concatenation over two.
one_line() { tr -s '[:space:]' ' ' | sed -e 's/^ //' -e 's/ $//'; }

# A string this cannot read is fatal rather than empty. Comparing against
# something the script failed to parse is how a check goes on reporting green
# over a file it has stopped understanding.
need_flags() {  # $1 = file, $2 = what was read
  [ -n "$2" ] && return 0
  echo >&2
  echo "error: no Dhrystone flag string could be read out of $1." >&2
  echo "This check reads both copies and compares them; if the declaration" >&2
  echo "was respelled, teach this script the new spelling rather than" >&2
  echo "dropping the comparison." >&2
  exit 1
}

mk_flags=$(awk '
  /^DHRY_CFLAGS[[:space:]]*:=/ { collecting = 1 }
  collecting {
    line = $0
    sub(/^DHRY_CFLAGS[[:space:]]*:=[[:space:]]*/, "", line)
    cont = (line ~ /\\[[:space:]]*$/)
    sub(/\\[[:space:]]*$/, "", line)
    out = out " " line
    if (!cont) { print out; exit }
  }' "$REPO/Makefile" | one_line)
need_flags Makefile "$mk_flags"

py_flags=$(awk '
  index($0, "dhry_flags") && index($0, "=") { collecting = 1 }
  collecting {
    line = $0
    while (match(line, /"[^"]*"/)) {
      out = out substr(line, RSTART + 1, RLENGTH - 2)
      line = substr(line, RSTART + RLENGTH)
    }
    if (line ~ /\)/) { print out; exit }
  }' "$REPO/soc/depth/cycles.py" | one_line)
need_flags soc/depth/cycles.py "$py_flags"

if [ "$mk_flags" != "$py_flags" ]; then
  rc=1
  echo >&2
  echo "error: the Dhrystone flags are stated twice and they disagree." >&2
  echo "  Makefile           : $mk_flags" >&2
  echo "  soc/depth/cycles.py: $py_flags" >&2
  echo "The flags are the measurement's other half -- they are compiled INTO" >&2
  echo "the benchmark and printed beside the number, precisely so the two" >&2
  echo "cannot be quoted apart. Two copies that differ produce two numbers for" >&2
  echo "two machines under one heading." >&2
fi

if [ "$rc" -ne 0 ]; then
  exit 1
fi

sites=$(wc -l < "$tmp/required" | tr -d ' ')
excepted=$(wc -l < "$tmp/exceptions" | tr -d ' ')
files=$(tr -cd '\0' < "$tmp/files" | wc -c | tr -d ' ')
echo "-march=$DECLARED_MARCH at all $sites sites, $excepted named exceptions, over $files tracked files"

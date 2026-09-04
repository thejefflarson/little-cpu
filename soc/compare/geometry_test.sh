#!/bin/bash
# Asserts that every file describing the comparison harness's geometry describes
# the same one.
#
# Usage: geometry_test.sh [repo-root]     # defaults to this script's grandparent
#
# The harness's whole claim is that all three cores were measured in ONE geometry.
# The Makefile's variables, every comparison-harness top's parameter defaults,
# the linker script's two regions and the program's RAM base state part of it,
# and nothing but this compares them. A ROM that is 1024 words in the Makefile
# and 2048 in the RTL still synthesises, still places and still reports a
# critical path; it just reports it for a design the other cores were not
# measured against.
#
# THE FILE LIST IS DERIVED, NOT HAND-MAINTAINED. This script used to name the
# three comparison cores as a literal list, so a fourth core -- or any other
# soc/compare/bench_*.v declaring a chparam'able ROM_WORDS -- could land with no
# comparison to trip: adding a name here was a step nothing forced anyone to
# remember, which is why PR #243 adding soc/compare/coremark.lds and
# coremark_tb.v could not have tripped anything -- there was no comparison to
# trip. `chparam -set ROM_WORDS ... $(COMPARE_TOP)` in the Makefile is the
# actual mechanism that makes a top's ROM_WORDS the harness's ROM_WORDS, so this
# script reads its list of tops from the Makefile's own `COMPARE_TOP := ...`
# lines instead of keeping a second copy, and requires every
# soc/compare/bench_*.v that declares `parameter integer ROM_WORDS` -- the
# module-parameter form chparam can reach, as against a `localparam`, which it
# cannot -- to be one of those names. A file matching that glob with no such
# parameter (a testbench, an adapter) states no geometry of its own and is
# silently exempt: the exemption is by DECLARATION, not by a second hand-kept
# list this check would only ever be as complete as.
#
# The same argument covers the one *.lds file this geometry's numeric
# comparison actually reads: the Makefile links it directly
# (`-T soc/compare/<name>.lds`), which is the one thing that ties a linker
# script to THIS chparam'd geometry rather than to a differently-sized
# simulation of its own -- soc/compare/dhry.lds links a deliberately larger map
# that does not fit this one, and soc/compare/dhry_fit.py is the check that
# grades it against soc/compare/dhry_tb.v instead. Every OTHER soc/compare/*.lds
# must be named by some other soc/compare/*.sh or *.py script, so a linker
# script that is neither the Makefile's own nor read by anything else states a
# geometry nothing grades.
#
# Hermetic: grep, sed and shell arithmetic. No toolchain, no simulator, no yosys.
set -euo pipefail

# On once, for the whole script, rather than toggled around each glob: nothing
# here relies on the unexpanded-pattern behaviour nullglob turns off. /bin/bash
# on macOS is 3.2, whose `set -u` treats a zero-element array's `"${arr[@]}"`
# as an unbound variable, so every glob below is a plain `for` loop rather than
# an array -- that degenerates to zero iterations under nullglob instead of an
# error, with nothing to unset back.
shopt -s nullglob

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=${1:-$(cd "$HERE/../.." && pwd)}
SELF="$HERE/$(basename "$0")"

if [ ! -d "$REPO" ]; then
  echo "error: '$REPO' is not a directory, so there is nothing to compare." >&2
  exit 1
fi

rc=0

fail() {
  echo "error: $*" >&2
  rc=1
}

for f in Makefile soc/compare/bench.S rtl/memory.v; do
  if [ ! -f "$REPO/$f" ]; then
    echo "error: $f is missing, so its copy of the harness geometry cannot be" >&2
    echo "compared. If it moved, move this check with it." >&2
    exit 1
  fi
done

# A declaration this cannot read is fatal rather than empty: comparing against
# an empty string is how a check goes on reporting green over a file it has
# stopped understanding.
read_or_die() {  # $1 = label, $2 = file, $3 = sed program
  local value
  value=$(sed -n "$3" "$REPO/$2" | head -1)
  if [ -z "$value" ]; then
    echo "error: no $1 found in $2. This check compares the harness's stated" >&2
    echo "geometry; if the declaration was respelled, teach this script the new" >&2
    echo "spelling rather than dropping the comparison." >&2
    exit 1
  fi
  printf '%s' "$value"
}

# The Makefile's own list of comparison-harness tops: every `COMPARE_TOP := X`
# line names an X that `chparam -set ROM_WORDS ...` reaches, which is what
# makes a name here worth cross-checking at all.
mk_tops=$(sed -n 's/^COMPARE_TOP *:= *\(bench_[A-Za-z0-9_]*\) *$/\1/p' \
  "$REPO/Makefile" | sort -u)
if [ -z "$mk_tops" ]; then
  echo "error: no 'COMPARE_TOP := bench_*' line found in Makefile. This check" >&2
  echo "reads the comparison harness's own top list from there rather than" >&2
  echo "keeping a second copy; teach it the new spelling if it moved." >&2
  exit 1
fi

# The Makefile's own linker-script reference: `-T soc/compare/<name>.lds` is
# what ties a script to THIS chparam'd geometry, the same way COMPARE_TOP ties
# a .v file to it.
mk_lds=$(sed -n 's#.*-T soc/compare/\([A-Za-z0-9_]*\)\.lds.*#\1#p' \
  "$REPO/Makefile" | sort -u)
if [ -z "$mk_lds" ]; then
  echo "error: no '-T soc/compare/*.lds' line found in Makefile. This check" >&2
  echo "reads which linker script states the harness's own geometry from" >&2
  echo "there; teach it the new spelling if it moved." >&2
  exit 1
fi

mk_rom=$(read_or_die "COMPARE_ROM_WORDS" Makefile \
  's/^COMPARE_ROM_WORDS *:= *\([0-9]*\).*/\1/p')
mk_ram=$(read_or_die "COMPARE_RAM_WORDS" Makefile \
  's/^COMPARE_RAM_WORDS *:= *\([0-9]*\).*/\1/p')

# The program reaches RAM through a literal, because it has no .data for the
# linker to place there. rtl/memory.v's default base is what all three harnesses
# instantiate, so the literal has to be that number. Read here, ahead of the
# lds loop below, because neither depends on mk_lds and the lds loop's own
# ORIGIN check needs rtl_base already in hand.
prog_base=$(read_or_die "RAM base literal" soc/compare/bench.S \
  's/.*li *t0, *\(0x[0-9a-fA-F]*\).*/\1/p')
rtl_base=$(read_or_die "BASE parameter default" rtl/memory.v \
  "s/.*BASE *= *32'h\([0-9a-fA-F_]*\).*/\1/p")
rtl_base=0x${rtl_base//_/}
if [ "$((prog_base))" != "$((rtl_base))" ]; then
  fail "soc/compare/bench.S addresses RAM at $prog_base, rtl/memory.v's BASE is $rtl_base"
fi

for top in $mk_tops; do
  if [ ! -f "$REPO/soc/compare/$top.v" ]; then
    echo "error: the Makefile names comparison top '$top' (COMPARE_TOP := $top)" >&2
    echo "but soc/compare/$top.v does not exist." >&2
    exit 1
  fi
  v_rom=$(read_or_die "ROM_WORDS default" "soc/compare/$top.v" \
    's/.*parameter integer ROM_WORDS *= *\([0-9]*\).*/\1/p')
  v_ram=$(read_or_die "RAM_WORDS default" "soc/compare/$top.v" \
    's/.*parameter integer RAM_WORDS *= *\([0-9]*\).*/\1/p')
  [ "$v_rom" = "$mk_rom" ] || fail \
    "soc/compare/$top.v has ROM_WORDS=$v_rom, the Makefile has COMPARE_ROM_WORDS=$mk_rom"
  [ "$v_ram" = "$mk_ram" ] || fail \
    "soc/compare/$top.v has RAM_WORDS=$v_ram, the Makefile has COMPARE_RAM_WORDS=$mk_ram"
done

# Both ways: every soc/compare/bench_*.v that DECLARES a chparam'able ROM_WORDS
# must be one of the Makefile's own tops -- a file the Makefile's COMPARE_CORE
# selection cannot reach is measured by nothing, however faithfully its own
# ROM_WORDS agrees with a number nobody chparams it to. `parameter integer` is
# the distinguishing test rather than the filename: soc/compare/bench_tb.v
# matches the glob and states no geometry of its own, and a `localparam` (the
# shape soc/compare/dhry_tb.v uses) is fixed at authoring time and never
# reaches chparam at all, so neither is asked to agree with anything.
for f in "$REPO"/soc/compare/bench_*.v; do
  base=$(basename "$f" .v)
  if grep -q 'parameter integer ROM_WORDS' "$f" \
    && ! printf '%s\n' "$mk_tops" | grep -qx "$base"; then
    fail "soc/compare/$base.v declares parameter integer ROM_WORDS but no" \
         "'COMPARE_TOP := $base' line in the Makefile reaches it -- it is not" \
         "chparam'd and not compared"
  fi
done

# `LENGTH = 4K` -> bytes. Only K is accepted: a script that quietly read `4M` as
# 4 would compare two numbers that agree and mean different sizes.
lds_bytes() {  # $1 = region name, $2 = lds path relative to $REPO
  local raw
  raw=$(read_or_die "$1 region" "$2" \
    "s/^ *$1(.*LENGTH *= *\([0-9]*\)K *\$/\1/p")
  echo $((raw * 1024))
}

for name in $mk_lds; do
  if [ ! -f "$REPO/soc/compare/$name.lds" ]; then
    echo "error: soc/compare/$name.lds does not exist, so its copy of the" >&2
    echo "harness geometry cannot be compared. The Makefile links it; if it" >&2
    echo "moved, move this check with it." >&2
    exit 1
  fi
  lds_rom=$(lds_bytes rom "soc/compare/$name.lds")
  lds_ram=$(lds_bytes ram "soc/compare/$name.lds")
  [ "$lds_rom" = "$((mk_rom * 4))" ] || fail \
    "soc/compare/$name.lds' rom region is $lds_rom bytes, the harness ROM is $((mk_rom * 4))"
  [ "$lds_ram" = "$((mk_ram * 4))" ] || fail \
    "soc/compare/$name.lds' ram region is $lds_ram bytes, the harness RAM is $((mk_ram * 4))"
  lds_origin=$(read_or_die "ram ORIGIN" "soc/compare/$name.lds" \
    's/^ *ram(.*ORIGIN *= *\(0x[0-9a-fA-F]*\).*/\1/p')
  if [ "$((lds_origin))" != "$((rtl_base))" ]; then
    fail "soc/compare/$name.lds' ram ORIGIN is $lds_origin, rtl/memory.v's BASE is $rtl_base"
  fi
done

# Both ways again: every soc/compare/*.lds the Makefile does NOT link itself
# must be named by some other soc/compare/*.sh or *.py -- a linker script
# neither this check nor anything else reads states a geometry nobody grades.
# Every OTHER script's own *.lds references are collected once, as whole
# `name.lds` tokens rather than a bare substring test: a substring test would
# let soc/compare/not_orphan.lds' own name satisfy the check for
# soc/compare/orphan.lds, since the shorter name sits inside the longer one.
# SELF is excluded so a comment naming a file (like this one naming
# soc/compare/dhry.lds above) can never stand in for a real reference.
# `|| true`: a script naming no .lds at all is a normal outcome (most of
# soc/compare/ names none), not a failure -- without it, that grep's exit 1
# becomes the `for` loop's own exit status, which `pipefail` then blames on
# the whole assignment, aborting the script here with no message at all.
referenced_lds=$(for s in "$REPO"/soc/compare/*.sh "$REPO"/soc/compare/*.py; do
  [ "$s" = "$SELF" ] && continue
  grep -ohE '[A-Za-z0-9_]+\.lds' "$s" || true
done | sed 's/\.lds$//' | sort -u)

for f in "$REPO"/soc/compare/*.lds; do
  base=$(basename "$f" .lds)
  if printf '%s\n' "$mk_lds" | grep -qx "$base"; then
    continue
  fi
  if ! printf '%s\n' "$referenced_lds" | grep -qx "$base"; then
    fail "soc/compare/$base.lds is linked by nothing: not the Makefile's own" \
         "-T, and no soc/compare/*.sh or *.py names it -- its geometry is" \
         "graded by nothing"
  fi
done

if [ "$rc" -eq 0 ]; then
  echo "soc/compare geometry: ${mk_rom}-word ROM, ${mk_ram}-word RAM at $rtl_base," \
       "stated the same way everywhere it is declared"
fi
exit $rc

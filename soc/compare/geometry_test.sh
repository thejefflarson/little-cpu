#!/bin/bash
# Asserts that every file describing the comparison harness's geometry describes the same
# one.
set -euo pipefail

# On once, for the whole script, rather than toggled around each glob: nothing here
# relies on the unexpanded-pattern behaviour nullglob turns off.
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

# A declaration this cannot read is fatal rather than empty: comparing against an empty
# string is how a check goes on reporting green over a file it has stopped understanding.
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

mk_tops=$(sed -n 's/^COMPARE_TOP *:= *\(bench_[A-Za-z0-9_]*\) *$/\1/p' \
  "$REPO/Makefile" | sort -u)
if [ -z "$mk_tops" ]; then
  echo "error: no 'COMPARE_TOP := bench_*' line found in Makefile. This check" >&2
  echo "reads the comparison harness's own top list from there rather than" >&2
  echo "keeping a second copy; teach it the new spelling if it moved." >&2
  exit 1
fi

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

for f in "$REPO"/soc/compare/bench_*.v; do
  base=$(basename "$f" .v)
  if grep -q 'parameter integer ROM_WORDS' "$f" \
    && ! printf '%s\n' "$mk_tops" | grep -qx "$base"; then
    fail "soc/compare/$base.v declares parameter integer ROM_WORDS but no" \
         "'COMPARE_TOP := $base' line in the Makefile reaches it -- it is not" \
         "chparam'd and not compared"
  fi
done

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

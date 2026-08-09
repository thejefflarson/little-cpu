#!/bin/bash
# Asserts that every file describing the comparison harness's geometry describes
# the same one.
#
# Usage: geometry_test.sh [repo-root]     # defaults to this script's grandparent
#
# The harness's whole claim is that both cores were measured in ONE geometry. Six
# files state part of it -- the Makefile's variables, both harness tops' parameter
# defaults, the linker script's two regions and the program's RAM base -- and
# nothing but this compares them. A ROM that is 1024 words in the Makefile and
# 2048 in the RTL still synthesises, still places and still reports a critical
# path; it just reports it for a design the other core was not measured against.
#
# The Makefile's numbers reach synthesis through `chparam`, so they are what the
# placed netlist is built from. The tops' defaults are what iverilog uses in
# soc/compare/bench_tb.v, which has no chparam -- so a divergence would leave the
# simulation that says the harness works and the placement that says how fast it
# is describing different machines.
#
# Hermetic: grep, sed and shell arithmetic. No toolchain, no simulator, no yosys.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=${1:-$(cd "$HERE/../.." && pwd)}

if [ ! -d "$REPO" ]; then
  echo "error: '$REPO' is not a directory, so there is nothing to compare." >&2
  exit 1
fi

rc=0

fail() {
  echo "error: $*" >&2
  rc=1
}

for f in Makefile soc/compare/bench_littlecpu.v soc/compare/bench_vexriscv.v \
         soc/compare/bench.lds soc/compare/bench.S rtl/memory.v; do
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

mk_rom=$(read_or_die "COMPARE_ROM_WORDS" Makefile \
  's/^COMPARE_ROM_WORDS *:= *\([0-9]*\).*/\1/p')
mk_ram=$(read_or_die "COMPARE_RAM_WORDS" Makefile \
  's/^COMPARE_RAM_WORDS *:= *\([0-9]*\).*/\1/p')

for top in bench_littlecpu bench_vexriscv; do
  v_rom=$(read_or_die "ROM_WORDS default" "soc/compare/$top.v" \
    's/.*parameter integer ROM_WORDS *= *\([0-9]*\).*/\1/p')
  v_ram=$(read_or_die "RAM_WORDS default" "soc/compare/$top.v" \
    's/.*parameter integer RAM_WORDS *= *\([0-9]*\).*/\1/p')
  [ "$v_rom" = "$mk_rom" ] || fail \
    "soc/compare/$top.v has ROM_WORDS=$v_rom, the Makefile has COMPARE_ROM_WORDS=$mk_rom"
  [ "$v_ram" = "$mk_ram" ] || fail \
    "soc/compare/$top.v has RAM_WORDS=$v_ram, the Makefile has COMPARE_RAM_WORDS=$mk_ram"
done

# `LENGTH = 4K` -> bytes. Only K is accepted: a script that quietly read `4M` as
# 4 would compare two numbers that agree and mean different sizes.
lds_bytes() {  # $1 = region name
  local raw
  raw=$(read_or_die "$1 region" soc/compare/bench.lds \
    "s/^ *$1(.*LENGTH *= *\([0-9]*\)K *\$/\1/p")
  echo $((raw * 1024))
}

lds_rom=$(lds_bytes rom)
lds_ram=$(lds_bytes ram)
[ "$lds_rom" = "$((mk_rom * 4))" ] || fail \
  "soc/compare/bench.lds' rom region is $lds_rom bytes, the harness ROM is $((mk_rom * 4))"
[ "$lds_ram" = "$((mk_ram * 4))" ] || fail \
  "soc/compare/bench.lds' ram region is $lds_ram bytes, the harness RAM is $((mk_ram * 4))"

# The program reaches RAM through a literal, because it has no .data for the
# linker to place there. rtl/memory.v's default base is what both harnesses
# instantiate, so the literal has to be that number.
prog_base=$(read_or_die "RAM base literal" soc/compare/bench.S \
  's/.*li *t0, *\(0x[0-9a-fA-F]*\).*/\1/p')
rtl_base=$(read_or_die "BASE parameter default" rtl/memory.v \
  "s/.*BASE *= *32'h\([0-9a-fA-F_]*\).*/\1/p")
rtl_base=0x${rtl_base//_/}
if [ "$((prog_base))" != "$((rtl_base))" ]; then
  fail "soc/compare/bench.S addresses RAM at $prog_base, rtl/memory.v's BASE is $rtl_base"
fi

# The linker script's ram ORIGIN is the same number a third time.
lds_origin=$(read_or_die "ram ORIGIN" soc/compare/bench.lds \
  's/^ *ram(.*ORIGIN *= *\(0x[0-9a-fA-F]*\).*/\1/p')
if [ "$((lds_origin))" != "$((rtl_base))" ]; then
  fail "soc/compare/bench.lds' ram ORIGIN is $lds_origin, rtl/memory.v's BASE is $rtl_base"
fi

if [ "$rc" -eq 0 ]; then
  echo "soc/compare geometry: ${mk_rom}-word ROM, ${mk_ram}-word RAM at $rtl_base," \
       "stated the same way in all six places"
fi
exit $rc

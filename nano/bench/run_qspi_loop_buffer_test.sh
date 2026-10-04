#!/bin/bash
# Branch-free code must cost the same with the loop buffer on or off, a resident loop must pay
# exactly its zero-wait cost and count exactly its own hits, and every other loop must run to
# PASS under the model's fetch graders. Builds against the given nano_qspi_memory.v: the
# shipping one, or a mutated copy the probe hands it.
set -euo pipefail

if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
  echo "usage: run_qspi_loop_buffer_test.sh <nano-march-cflags> [path-to-nano_qspi_memory.v]" >&2
  exit 1
fi

CFLAGS=$1
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
QSPI_MEM_V=${2:-$REPO/nano/tb/nano_qspi_memory.v}

CC=""
if command -v riscv-none-elf-gcc >/dev/null 2>&1; then
  CC=riscv-none-elf-gcc
fi
if [ -z "$CC" ]; then
  echo "error: no RISC-V cross compiler found; see 'make riscv-gcc-setup'." >&2
  exit 1
fi
OBJCOPY=${CC%gcc}objcopy

tmp=$(mktemp -d "${TMPDIR:-/tmp}/nano-qspi-loop-test.XXXXXX")
trap 'rm -rf "$tmp"' EXIT

NANO_RISCV_FORMAL_MACROS="RISCV_FORMAL RISCV_FORMAL_COMPRESSED RISCV_FORMAL_ALIGNED_MEM RISCV_FORMAL_NRET=1 RISCV_FORMAL_XLEN=32 RISCV_FORMAL_ILEN=32"

build_sim() {  # tag, loop kind, loop window -> $tmp/nano-qspi-sim.<tag>
  local tag=$1 kind=$2 window=$3
  # shellcheck disable=SC2086
  yosys -p "read_verilog -sv $(printf -- '-D %s ' $NANO_RISCV_FORMAL_MACROS) \
    -D NANO_QSPI_TIMING -D NANO_QSPI_PREFETCH_DEPTH=2 -D NANO_QSPI_LOOP_KIND=$kind \
    -D NANO_QSPI_LOOP_WINDOW=$window -D NANO_QSPI_PREAMBLE_CYCLES=24 \
    \"$REPO/rvfi_macros.vh\" \"$REPO/nano/nano.v\" \"$QSPI_MEM_V\" \
    \"$REPO/soc/compare/dhry_monitor.v\" \"$REPO/nano/tb/nano_testbench.v\" \
    \"$REPO/test/monitor.sim.v\"; hierarchy -top nano_testbench; write_cxxrtl \"$tmp/$tag.rtl.cc\"" \
    > "$tmp/$tag.yosys.log" 2>&1 || { tail -60 "$tmp/$tag.yosys.log" >&2; exit 1; }
  clang++ -O2 -DNDEBUG -std=c++17 -Wall -Wextra -Werror \
    -DNANO_RTL_INCLUDE="\"$tmp/$tag.rtl.cc\"" \
    -isystem "$(yosys-config --datdir)/include/backends/cxxrtl/runtime" -I "$REPO/nano/tb" \
    "$REPO/nano/tb/nano_cxxrtl.cc" -o "$tmp/nano-qspi-sim.$tag" \
    > "$tmp/$tag.build.log" 2>&1 || { tail -60 "$tmp/$tag.build.log" >&2; exit 1; }
}

assemble() {  # $1 = elf name, $2.. = extra -D defines
  local elf=$1; shift
  # shellcheck disable=SC2086
  $CC $CFLAGS -nostdlib -I "$REPO/nano/asm" -I "$REPO/test/asm" "$@" \
    -T "$REPO/nano/asm/nano.lds" "$REPO/nano/bench/qspi_loop_micro.S" -o "$tmp/$elf"
  $OBJCOPY -O verilog --verilog-data-width=4 --only-section=.text "$tmp/$elf" "$tmp/$elf.rom.hex"
  $OBJCOPY -O verilog --verilog-data-width=4 --remove-section=.text "$tmp/$elf" "$tmp/$elf.ram.hex"
}

bucket() {  # $1 = BUCKETS log, $2 = field name
  local value
  value=$(grep '^BUCKETS ' "$1" | grep -oE "$2=[0-9]+" | cut -d= -f2) || value=""
  if [ -z "$value" ]; then
    echo "error: no '$2' field on a BUCKETS line of $1" >&2
    exit 1
  fi
  printf '%s' "$value"
}

run_sim() {  # $1 = sim tag, $2 = elf name -> writes $tmp/<tag>.<elf>.log
  local tag=$1 elf=$2
  "$tmp/nano-qspi-sim.$tag" --rom "$tmp/$elf.rom.hex" --ram "$tmp/$elf.ram.hex" --cycles 200000 \
    > "$tmp/$tag.$elf.log" 2>&1 || {
    echo "error: $tag against $elf did not PASS -- see below" >&2
    cat "$tmp/$tag.$elf.log" >&2
    exit 1
  }
}

build_sim none 0 0 & build_none=$!
build_sim tagged 1 8 & build_tagged=$!
build_sim cam 2 8 & build_cam=$!
built=1
for pid in "$build_none" "$build_tagged" "$build_cam"; do
  wait "$pid" || built=0
done
if [ "$built" -ne 1 ]; then
  exit 1
fi
assemble straight.elf -DKIND=0

for tag in none tagged cam; do
  run_sim "$tag" straight.elf
done
baseline=$(grep '^BUCKETS ' "$tmp/none.straight.elf.log")
for tag in tagged cam; do
  other=$(grep '^BUCKETS ' "$tmp/$tag.straight.elf.log")
  if [ "$baseline" != "$other" ]; then
    echo "FAIL: branch-free cycles differ with the loop buffer on ($tag)" >&2
    echo "  no loop buffer: $baseline" >&2
    echo "  $tag:            $other" >&2
    exit 1
  fi
done
echo "ok   branch-free cycles are identical with the loop buffer off, tagged-block, and CAM"

REPS_LO=200
REPS_HI=400
REPS_DELTA=$((REPS_HI - REPS_LO))
# A zero-wait memory answers a fetch in one cycle, so a resident loop's two instructions are two
# hits an iteration and every other bucket stays flat.
HITS_PER_ITERATION=2
# The tagged block's first back-branch re-tags into the next block, so that iteration misses.
HITS_LOST_TAGGED=2
HITS_LOST_CAM=1
assemble loop_lo.elf -DKIND=1 -DREPS=$REPS_LO
assemble loop_hi.elf -DKIND=1 -DREPS=$REPS_HI
for tag in tagged cam; do
  run_sim "$tag" loop_lo.elf
  run_sim "$tag" loop_hi.elf
  lo="$tmp/$tag.loop_lo.elf.log"
  hi="$tmp/$tag.loop_hi.elf.log"
  d_preamble=$(( $(bucket "$hi" redirect_preamble) - $(bucket "$lo" redirect_preamble) )) || exit 1
  d_parcel=$(( $(bucket "$hi" parcel_wait) - $(bucket "$lo" parcel_wait) )) || exit 1
  d_hit=$(( $(bucket "$hi" loop_hit) - $(bucket "$lo" loop_hit) )) || exit 1
  d_window=$(( $(bucket "$hi" window_cycles) - $(bucket "$lo" window_cycles) )) || exit 1
  if [ "$d_preamble" -ne 0 ] || [ "$d_parcel" -ne 0 ]; then
    echo "FAIL: $tag's resident loop still pays a marginal preamble/wait cost per iteration" \
      "(redirect_preamble +$d_preamble, parcel_wait +$d_parcel over $REPS_DELTA extra reps)" >&2
    exit 1
  fi
  if [ "$d_hit" -ne $((HITS_PER_ITERATION * REPS_DELTA)) ]; then
    echo "FAIL: $tag's resident loop reports +$d_hit loop-buffer hits over $REPS_DELTA extra reps," \
      "not exactly $((HITS_PER_ITERATION * REPS_DELTA))" >&2
    exit 1
  fi
  case $tag in tagged) lost=$HITS_LOST_TAGGED ;; *) lost=$HITS_LOST_CAM ;; esac
  hits_total=$(bucket "$hi" loop_hit) || exit 1
  want_total=$((HITS_PER_ITERATION * (REPS_HI - lost)))
  if [ "$hits_total" -ne "$want_total" ]; then
    echo "FAIL: $tag counts $hits_total loop-buffer hits over $REPS_HI reps, not exactly" \
      "$want_total: a fetch the flash queue serves must be a handshake, not a hit" >&2
    exit 1
  fi
  echo "ok   $tag's resident loop pays zero marginal preamble/wait, exactly +$d_hit hits and its" \
    "zero-wait +$d_window cycles over $REPS_DELTA reps, $hits_total hits in all"
done

assemble straddle_lo.elf -DKIND=2 -DREPS=$REPS_LO
assemble straddle_hi.elf -DKIND=2 -DREPS=$REPS_HI
for tag in tagged cam; do
  run_sim "$tag" straddle_lo.elf
  run_sim "$tag" straddle_hi.elf
done
echo "ok   a loop with a load and a block-straddling instruction runs to PASS in both shapes," \
  "every fetch served by the loop buffer or by parcels its flash run actually streamed"

REPS_LONG=50
assemble long_loop.elf -DKIND=3 -DREPS=$REPS_LONG
assemble wide_loop.elf -DKIND=4 -DREPS=$REPS_LONG
for tag in tagged cam; do
  run_sim "$tag" long_loop.elf
  run_sim "$tag" wide_loop.elf
done
echo "ok   a loop longer than the buffer, crossing the tagged block's edge, and a loop of only" \
  "32-bit instructions run to PASS in both shapes, no fetch re-served from a parcel the queue" \
  "handed over and no hit for a parcel that never entered the buffer"

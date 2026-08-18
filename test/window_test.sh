#!/bin/bash
# Asserts that the range decodes refuse to elaborate at a parameter shape they
# are not valid for -- in both frontends, and for the reason each is written
# for.
#
# Usage: window_test.sh          # every case; exit 0 only if all of them hold
#
# WHY THIS EXISTS. rtl/imemory.v, rtl/memory.v and rtl/timer.v each decide
# whether an address is inside their window. Each used to subtract the base and
# compare against the size, which is correct at any base and any size and costs
# a carry chain. Each now reads the address bits above the window instead, which
# is correct only while the window is a power of two sitting on a multiple of
# its own size. rtl/littlecpu.v copies all three windows for its load/store
# locality counters, and demands the same shapes of its copy.
#
# That is a different KIND of dependency from the one it replaced. The old
# spelling was slow at a bad parameter; the new one is silently wrong -- it
# admits addresses past the end of the memory, and those alias onto real words
# instead of reading zero. A comment saying "the window must be aligned" is not
# a check, so each file carries an elaboration `$fatal` beside the test it
# guards, and this asserts that each one fires.
#
# BOTH FRONTENDS, because a check only iverilog enforces would let the
# synthesised design be wrong while every simulation stayed green -- which is
# the shape of the defect it is guarding against. Needs iverilog and yosys, both
# of which `make test` already requires.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/.." && pwd)

for tool in iverilog yosys; do
  if ! command -v "$tool" >/dev/null 2>&1; then
    echo "error: $tool is not on PATH, so the elaboration checks in rtl/ cannot" >&2
    echo "be forced to fire. Skipping would report a green run for checks that" >&2
    echo "were never executed, which is what this file exists to prevent." >&2
    exit 1
  fi
done

tmp=$(mktemp -d "${TMPDIR:-/tmp}/littlecpu-window.XXXXXX")
test -n "$tmp" -a -d "$tmp"
trap 'rm -rf "$tmp"' EXIT

cases=0
failed=0

# Run from inside the fixture directory so the two frontends are handed bare
# filenames. yosys takes its script as one string, in which an interpolated
# path containing a space would read as two files.
cd "$tmp"

# What each frontend is handed, set by the two case runners below. The memories
# elaborate from their own file alone; the core needs the rest of rtl/ behind it.
SRCS=dut.v

# elaborate <frontend> -> prints the diagnostic, returns the frontend's status
elaborate() {
  case $1 in
    iverilog) iverilog -g2012 -o /dev/null $SRCS probe.v 2>&1 ;;
    yosys)    yosys -p "read_verilog -sv $SRCS probe.v; hierarchy -top window_probe" 2>&1 ;;
  esac
}

# run_case <label> <file> <module> <params> <expect: reject|accept> <text>
#
# The wrapper names the module under test and overrides one parameter, and is
# written per case rather than shared so a case cannot pass by elaborating a
# module other than the one it means to.
#
# A rejecting case pins a fragment of the DIAGNOSTIC as well as the status.
# Status alone would be satisfied by a wrapper that failed to compile for a
# typo, demonstrating nothing about the check it names.
run_case() {
  local label=$1 file=$2 module=$3 params=$4 expect=$5 want=$6
  printf '`default_nettype none\nmodule window_probe;\n  %s #(%s) dut ();\nendmodule\n' \
    "$module" "$params" > probe.v
  # A file that includes another one cannot elaborate on its own, and neither
  # can the stages it goes on to instantiate: those get the whole directory,
  # flat, so the include resolves with no search path. It also puts the shipping
  # rtl/littlesoc.v in front of the frontend beside the probe, which is a second
  # instantiation of the module under test at the parameters that place on the
  # part. The three memories include nothing and are handed their own file.
  if grep -q '^`include' "$REPO/$file"; then
    cp "$REPO"/rtl/*.v .
    SRCS=$(cd "$REPO/rtl" && printf '%s ' *.v)
  else
    cp "$REPO/$file" dut.v
    SRCS=dut.v
  fi
  local frontend out rc
  for frontend in iverilog yosys; do
    cases=$((cases + 1))
    set +e
    out=$(elaborate "$frontend")
    rc=$?
    set -e
    if [ "$expect" = reject ]; then
      if [ "$rc" -eq 0 ]; then
        echo "FAIL [$frontend] $label: elaborated cleanly; the check did not fire" >&2
        failed=$((failed + 1))
        continue
      fi
      # yosys reports an elaboration $fatal without the message arguments, so
      # the fragment is matched against iverilog's output only. On yosys the
      # pinned evidence is that the failure is a FATAL from the design rather
      # than a parse error in the wrapper.
      local pin=$want
      [ "$frontend" = yosys ] && pin='FATAL'
      case $out in
        *"$pin"*) ;;
        *)
          echo "FAIL [$frontend] $label: failed without \"$pin\":" >&2
          printf '%s\n' "$out" | sed 's/^/    /' >&2
          failed=$((failed + 1))
          continue ;;
      esac
      echo "ok   [$frontend] $label rejected"
    else
      if [ "$rc" -ne 0 ]; then
        echo "FAIL [$frontend] $label: the shipping parameters were rejected:" >&2
        printf '%s\n' "$out" | sed 's/^/    /' >&2
        failed=$((failed + 1))
        continue
      fi
      echo "ok   [$frontend] $label accepted"
    fi
  done
}

echo "== rtl/imemory.v: ROM_WORDS must be a power of two"
run_case "ROM_WORDS = 1536" rtl/imemory.v imemory ".ROM_WORDS(1536)" \
  reject "ROM_WORDS must be a power of two"
run_case "ROM_WORDS = 2048" rtl/imemory.v imemory ".ROM_WORDS(2048)" accept ""

echo
echo "== rtl/memory.v: an aligned power-of-two window"
run_case "RAM_WORDS = 12288" rtl/memory.v memory \
  ".BASE(32'h0001_0000), .RAM_WORDS(12288)" reject "RAM_WORDS must be a power of two"
run_case "BASE off the window" rtl/memory.v memory \
  ".BASE(32'h0001_0004), .RAM_WORDS(16384)" reject "BASE must be aligned"
run_case "the SoC's own" rtl/memory.v memory \
  ".BASE(32'h0001_0000), .RAM_WORDS(16384)" accept ""

echo
echo "== rtl/timer.v: a 16-byte aligned BASE"
run_case "BASE = 0x0002_0008" rtl/timer.v timer ".BASE(32'h0002_0008)" \
  reject "BASE must be 16-byte aligned"
run_case "the SoC's own" rtl/timer.v timer ".BASE(32'h0002_0000)" accept ""

# The core copies that map for its load/store locality counters, and its copy is
# read by nothing else -- so a shape no memory here could be built at would be
# counted against silently rather than refused. These are the same three checks,
# restated where the copy is.
echo
echo "== rtl/littlecpu.v: the copied map has the shape the memories demand"
# One override per case, so each names the parameter it is about and the other
# three stay at the values that ship; the accept below states all four.
run_case "LS_TEXT_WORDS = 3072" rtl/littlecpu.v littlecpu ".LS_TEXT_WORDS(3072)" \
  reject "LS_TEXT_WORDS must be a power of two"
run_case "LS_RAM_WORDS = 12288" rtl/littlecpu.v littlecpu ".LS_RAM_WORDS(12288)" \
  reject "LS_RAM_WORDS must be a power of two"
run_case "LS_RAM_BASE off the window" rtl/littlecpu.v littlecpu \
  ".LS_RAM_BASE(32'h0001_0004)" reject "LS_RAM_BASE must be aligned"
run_case "LS_TIMER_BASE = 0x0002_0008" rtl/littlecpu.v littlecpu \
  ".LS_TIMER_BASE(32'h0002_0008)" reject "LS_TIMER_BASE must be 16-byte aligned"
run_case "the SoC's own" rtl/littlecpu.v littlecpu \
  ".LS_TEXT_WORDS(2048), .LS_RAM_BASE(32'h0001_0000), .LS_RAM_WORDS(16384), .LS_TIMER_BASE(32'h0002_0000)" \
  accept ""

echo
if [ "$failed" -ne 0 ]; then
  echo "$failed of $cases elaborations did not behave as required." >&2
  exit 1
fi
echo "$cases elaborations, each rejected or accepted as required."

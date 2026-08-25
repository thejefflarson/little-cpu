#!/bin/bash
# Elaborates the board wrapper, and forces two ways of breaking it red.
#
# WHY THIS EXISTS. soc/board_upduino.v is the only file in this tree that no
# check on `make test` or on CI reads. `make bitstream` synthesises it and
# `make prog` flashes it, and both need a board plugged in. That was survivable
# while the file was ten lines of glue around `littlesoc`; it is not now that it
# arbitrates four shared pins behind `SB_IO` output enables, because the ways
# that goes wrong -- a port renamed on one side of the instantiation, an enable
# left unconnected, a net nothing drives -- are exactly what yosys reports for
# free and what a human with hardware would otherwise find.
#
# WARNINGS ARE ERRORS HERE, and the warning is the interesting half: yosys fails
# on a port that does not exist, and merely WARNS about a wire with no driver,
# which is what a mis-wired enable leaves behind.
#
# yosys prefixes a diagnostic tied to a source line with "file:line: " ahead of
# "ERROR: "/"Warning: " -- $readmemh's own "Can not open file" is one -- so a
# grep anchored at the start of the line finds a bare ERROR/Warning and misses
# every file-scoped one. That once left a FAIL here with nothing printed under
# it. `show_diagnostics` below matches both, and falls back to the log's tail
# so a diagnostic in a form neither of those covers still shows something.
#
# IT DOES NOT READ soc/upduino.pcf. Nothing in this repo parses a pcf, so a pin
# assignment naming a port that no longer exists is caught by nextpnr during
# `make bitstream` and by nothing here. That gap is real and is named rather
# than papered over.
#
# Usage: board_elaborate.sh <yosys> <top> <src>...
set -uo pipefail

if [ "$#" -lt 3 ]; then
  echo "usage: board_elaborate.sh <yosys> <top> <src>..." >&2
  exit 2
fi

YOSYS=$1; shift
TOP=$1; shift
SRCS=("$@")

TMP=$(mktemp -d "${TMPDIR:-/tmp}/boardelab.XXXXXX")
trap 'rm -rf "$TMP"' EXIT

failed=0
cases=0

# One elaboration. Prints nothing on success; the caller decides what a failure
# means, because both directions are wanted.
elaborate() {  # $1 = log path, then the sources
  local log=$1; shift
  "$YOSYS" -p "read_verilog -sv -lib +/ice40/cells_sim.v; read_verilog -sv $*; hierarchy -top $TOP -check; proc; opt_clean; check -assert" \
    > "$log" 2>&1
}

# Prints the log's ERROR/Warning lines, wherever in the line they start, or --
# if none matched, meaning the diagnostic took some third shape -- the log's
# own tail, so a FAIL here is never silent.
show_diagnostics() {  # $1 = log path
  local log=$1 matches
  matches=$(grep -E '(ERROR|Warning):' "$log")
  if [ -n "$matches" ]; then
    echo "$matches" | head -5 | sed 's/^/       /'
  else
    echo "       (no ERROR:/Warning: line matched -- last 10 lines of the log:)"
    tail -10 "$log" | sed 's/^/       /'
  fi
}

# `accept` wants exit 0 AND no warning; `reject` wants one or the other to go,
# with the given text in the log. A reject that failed for some unrelated reason
# is not a demonstration, which is why the text is compared and not just the
# status.
run_case() {  # $1 = what, $2 = accept|reject, $3 = expected text, then sources
  local what=$1 want=$2 text=$3; shift 3
  local log="$TMP/case.$cases.log"
  cases=$((cases + 1))
  elaborate "$log" "$@"
  local rc=$?
  local warned=no
  grep -qE 'Warning:' "$log" && warned=yes

  if [ "$want" = accept ]; then
    if [ "$rc" -eq 0 ] && [ "$warned" = no ]; then
      echo "ok   $what elaborated, warning-free"
    else
      echo "FAIL $what did not elaborate cleanly (rc=$rc, warnings=$warned):"
      show_diagnostics "$log"
      failed=$((failed + 1))
    fi
  else
    if { [ "$rc" -ne 0 ] || [ "$warned" = yes ]; } && grep -q "$text" "$log"; then
      echo "ok   $what refused, and for its own reason"
    else
      echo "FAIL $what was accepted, or refused for some other reason:"
      show_diagnostics "$log"
      failed=$((failed + 1))
    fi
  fi
}

echo "== soc/board_upduino.v: the wrapper as it ships"
run_case "the board wrapper" accept "" "${SRCS[@]}"

# The board file is the last source; every mutation below is a copy of it with
# the rest of the list unchanged.
# `${SRCS[-1]}` is a bash 4.3 spelling and macOS ships 3.2, which reads it as a
# subscript of minus one and refuses.
last=$(( ${#SRCS[@]} - 1 ))
board=${SRCS[$last]}
rest=("${SRCS[@]:0:$last}")

echo
echo "== and two ways of breaking it, each required to be caught"

# A mutation that did not apply is a red direction that is not being taken, and
# it looks exactly like one that is: the copy elaborates because it is the
# original. So each `sed` below is required to change something.
mutate() {  # $1 = output path, $2 = sed script
  sed "$2" "$board" > "$1"
  if cmp -s "$board" "$1"; then
    echo "FAIL the mutation \`$2\` changed nothing, so its case would pass on"
    echo "     the shipping file. The text it edits has moved."
    failed=$((failed + 1))
    return 1
  fi
}

# A port renamed on ONE side. This is what happens when rtl/littlesoc.v grows or
# loses a pin and the wrapper is not updated with it, which is the change this
# file was written for. yosys FAILS on it.
if mutate "$TMP/renamed.v" 's/\.uart_tx(uart_tx),/.uart_txx(uart_tx),/'; then
  run_case "a port the SoC does not have" reject "does not have a port named" \
    "${rest[@]}" "$TMP/renamed.v"
fi

# The pin whose level the enable on pin 14 reads, no longer wired back out of
# its `SB_IO`. yosys does NOT fail on this -- it warns that the wire has no
# driver -- so this is the case that says treating a warning as an error is
# load-bearing here rather than decorative.
if mutate "$TMP/undriven.v" 's/\.D_IN_0(ssn_in)/.D_IN_0()/'; then
  run_case "the enable reading a pin nothing drives" reject "has no driver" \
    "${rest[@]}" "$TMP/undriven.v"
fi

echo
if [ "$failed" -ne 0 ]; then
  echo "$failed of $cases elaborations did not behave as required." >&2
  exit 1
fi
echo "$cases elaborations: the wrapper reads, and both ways of breaking it are caught."

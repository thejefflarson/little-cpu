#!/bin/bash
# Elaborates the board wrapper, and forces two ways of breaking it red.
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

# One elaboration. Prints nothing on success; the caller decides what a failure means,
# because both directions are wanted.
elaborate() {  # $1 = log path, then the sources
  local log=$1; shift
  "$YOSYS" -p "read_verilog -sv -lib +/ice40/cells_sim.v; read_verilog -sv $*; hierarchy -top $TOP -check; proc; opt_clean; check -assert" \
    > "$log" 2>&1
}

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

last=$(( ${#SRCS[@]} - 1 ))
board=${SRCS[$last]}
rest=("${SRCS[@]:0:$last}")

echo
echo "== and two ways of breaking it, each required to be caught"

mutate() {  # $1 = output path, $2 = sed script
  sed "$2" "$board" > "$1"
  if cmp -s "$board" "$1"; then
    echo "FAIL the mutation \`$2\` changed nothing, so its case would pass on"
    echo "     the shipping file. The text it edits has moved."
    failed=$((failed + 1))
    return 1
  fi
}

# A port renamed on ONE side must not elaborate: an unconnected port is silent otherwise.
if mutate "$TMP/renamed.v" 's/\.uart_tx(uart_tx),/.uart_txx(uart_tx),/'; then
  run_case "a port the SoC does not have" reject "does not have a port named" \
    "${rest[@]}" "$TMP/renamed.v"
fi

if mutate "$TMP/undriven.v" 's/\.D_IN_0(ssn_pin)/.D_IN_0()/'; then
  run_case "the synchroniser reading a pin nothing drives" reject "has no driver" \
    "${rest[@]}" "$TMP/undriven.v"
fi

echo
if [ "$failed" -ne 0 ]; then
  echo "$failed of $cases elaborations did not behave as required." >&2
  exit 1
fi
echo "$cases elaborations: the wrapper reads, and both ways of breaking it are caught."

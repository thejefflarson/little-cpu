#!/bin/bash
# Sourced by soc/run_suite_board.sh. The verdict comes off the board's UART, so it is
# untrusted text: bash evaluates array subscripts inside $(( )), so an unvalidated
# `x[$(cmd)]` would run cmd.

# grade_verdict <text>: prints PASS, "FAIL <test>" or PARSE, and returns 0.
grade_verdict() {
  local v=$1
  case $v in
    ''|*[!0-9]*) echo PARSE; return 0;;
  esac
  [ "${#v}" -le 9 ] || { echo PARSE; return 0; }
  while [ "${#v}" -gt 1 ] && [ "${v#0}" != "$v" ]; do v=${v#0}; done
  if [ "$v" = 1 ]; then echo PASS; else echo "FAIL $(( v >> 1 ))"; fi
}

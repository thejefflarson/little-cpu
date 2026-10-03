#!/bin/bash
grade_verdict() {
  local v=$1
  case $v in
    ''|*[!0-9]*) echo PARSE; return 0;;
  esac
  [ "${#v}" -le 9 ] || { echo PARSE; return 0; }
  while [ "${#v}" -gt 1 ] && [ "${v#0}" != "$v" ]; do v=${v#0}; done
  if [ "$v" = 1 ]; then echo PASS; else echo "FAIL $(( v >> 1 ))"; fi
}

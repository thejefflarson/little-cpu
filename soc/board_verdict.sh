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

display_safe() { LC_ALL=C tr -cd '[:print:]\n'; }

# Refuses a symlink, or a file or parent directory not owned by OWNER_UID (root) or writable by group or world.
check_root_binary() {
  local bin=${1-} owner=${2-0} path st uid mode
  [ -n "$bin" ] || { echo "error: no binary named" >&2; return 1; }
  if [ -L "$bin" ] || [ ! -f "$bin" ]; then
    echo "error: $bin is not a regular file (a symlink or a missing path)" >&2; return 1
  fi
  for path in "$bin" "$(dirname "$bin")"; do
    st=$(stat -c '%u %a' "$path" 2>/dev/null || stat -f '%u %Lp' "$path" 2>/dev/null) \
      || { echo "error: cannot stat $path" >&2; return 1; }
    uid=${st% *}; mode=${st#* }
    case $uid$mode in *[!0-9]*) echo "error: unreadable owner or mode for $path" >&2; return 1;; esac
    if [ "$uid" != "$owner" ]; then
      echo "error: $path is owned by uid $uid, not $owner; run 'make install-board-tools'" >&2; return 1
    fi
    if [ $(( 8#$mode & 8#022 )) -ne 0 ]; then
      echo "error: $path is group- or world-writable (mode $mode); run 'make install-board-tools'" >&2; return 1
    fi
  done
}

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

native_magic() { LC_ALL=C od -An -tx1 -N4 "$1" 2>/dev/null | tr -d ' \n'; }

resolve_path() {
  local p=$1 dir
  if [ -d "$p" ]; then (cd -P "$p" && pwd -P); return; fi
  dir=$(cd -P "$(dirname "$p")" 2>/dev/null && pwd -P) || return 1
  echo "${dir%/}/$(basename "$p")"
}

owner_mode() {
  local st
  st=$(stat -c '%u %a' "$1" 2>/dev/null || stat -f '%u %Lp' "$1" 2>/dev/null) || return 1
  case ${st% *}${st#* } in ''|*[!0-9]*) return 1;; esac
  echo "$st"
}

acl_grants() {
  if [ "$(uname -s)" = Darwin ]; then
    ls -lde "$1" 2>/dev/null | tail -n +2 | grep -q ' allow '
  elif command -v getfacl >/dev/null 2>&1; then
    getfacl -cp "$1" 2>/dev/null | grep -Eq '^(user|group):[^:]+:.w'
  else
    return 1
  fi
}

# Prints $1's resolved path once it is owned by OWNER, each directory above by OWNER or root, none group-, world- or ACL-writable.
check_path_chain() {
  local want=${1-} owner=${2-0} real comp cur="" st uid mode
  local -a comps=()
  real=$(resolve_path "$want") || { echo "error: cannot resolve $want" >&2; return 1; }
  [ "$real" = / ] || IFS=/ read -r -a comps <<< "${real#/}"
  for comp in "" ${comps[@]+"${comps[@]}"}; do
    if [ -z "$comp" ]; then cur=/; else cur=${cur%/}/$comp; fi
    [ -L "$cur" ] && { echo "error: $cur is a symlink inside the resolved path of $want" >&2; return 1; }
    st=$(owner_mode "$cur") || { echo "error: cannot stat $cur" >&2; return 1; }
    uid=${st% *}; mode=${st#* }
    if [ "$cur" = "$real" ]; then
      [ "$uid" = "$owner" ] || { echo "error: $cur is owned by uid $uid, not $owner; run 'make install-board-tools'" >&2; return 1; }
    elif [ "$uid" != "$owner" ] && [ "$uid" != 0 ]; then
      echo "error: $cur is owned by uid $uid, not $owner or root; run 'make install-board-tools'" >&2; return 1
    fi
    if [ $(( 8#$mode & 8#022 )) -ne 0 ]; then
      echo "error: $cur is group- or world-writable (mode $mode); run 'make install-board-tools'" >&2; return 1
    fi
    if acl_grants "$cur"; then
      echo "error: $cur carries an ACL that grants access beyond its mode; run 'make install-board-tools'" >&2; return 1
    fi
  done
  printf '%s\n' "$real"
}

list_deps() {
  if command -v otool >/dev/null 2>&1; then
    otool -L "$1" | tail -n +2 | awk '{print $1}'
  elif command -v ldd >/dev/null 2>&1; then
    env -i "$(command -v ldd)" "$1" | awk '$2 == "=>" {print $3} $1 ~ /^\// && $2 !~ /=>/ {print $1}'
  else
    echo "error: neither otool nor ldd is available to list $1's libraries" >&2; return 1
  fi
}

elf_rpaths() {
  local f=$1 origin dir dynamic
  command -v readelf >/dev/null 2>&1 || { echo "error: readelf is needed to read $f's RUNPATH" >&2; return 1; }
  origin=$(dirname "$f")
  dynamic=$(readelf -d "$f") || { echo "error: readelf cannot read $f" >&2; return 1; }
  printf '%s\n' "$dynamic" | sed -n 's/.*(R[UN]*PATH).*\[\(.*\)\]$/\1/p' | tr ':' '\n' | while IFS= read -r dir; do
    dir=${dir//\$\{ORIGIN\}/$origin}; printf '%s\n' "${dir//\$ORIGIN/$origin}"
  done
}

# Checks every library and RUNPATH BIN reaches, transitively; only /usr/lib, /System/Library, /lib and /lib64 are trusted unread.
check_deps() {
  local bin=$1 owner=$2 exec_dir f dep real deps dir seen=" "
  local queue=("$bin")
  exec_dir=$(dirname "$bin")
  while [ "${#queue[@]}" -gt 0 ]; do
    f=${queue[0]}; queue=("${queue[@]:1}")
    if [ "$(native_magic "$f")" = 7f454c46 ]; then
      deps=$(elf_rpaths "$f") || return 1
      while IFS= read -r dir; do
        [ -n "$dir" ] || continue
        case $dir in /*) ;; *) echo "error: $f searches $dir, a relative RUNPATH" >&2; return 1;; esac
        check_path_chain "$dir" "$owner" >/dev/null || { echo "error: $f searches $dir for libraries" >&2; return 1; }
      done <<< "$deps"
    fi
    deps=$(list_deps "$f") || { echo "error: cannot list the libraries of $f" >&2; return 1; }
    while IFS= read -r dep; do
      [ -n "$dep" ] || continue
      case $dep in
        /System/Volumes/*) echo "error: $f loads $dep, on the writable data volume" >&2; return 1;;
        /usr/lib/*|/System/Library/*|/lib/*|/lib64/*)
          case $dep in *..*) echo "error: $f loads $dep, a path that climbs out of a system directory" >&2; return 1;; esac
          continue;;
        @executable_path/*) real=$exec_dir/${dep#@executable_path/};;
        /*) real=$dep;;
        *) echo "error: $f loads $dep, which is neither absolute nor @executable_path/" >&2; return 1;;
      esac
      case $seen in *" $real "*) continue;; esac
      seen="$seen$real "
      [ -f "$real" ] || { echo "error: $f loads $dep, but $real is not a file" >&2; return 1; }
      real=$(check_path_chain "$real" "$owner") || return 1
      queue+=("$real")
    done <<< "$deps"
  done
}

# Prints the resolved path it checked, the only one a caller may run: through an unresolved one, a symlink can be repointed.
check_root_binary() {
  local bin=${1-} owner=${2-0} real
  [ -n "$bin" ] || { echo "error: no binary named" >&2; return 1; }
  if [ -L "$bin" ] || [ ! -f "$bin" ]; then
    echo "error: $bin is not a regular file (a symlink or a missing path)" >&2; return 1
  fi
  real=$(check_path_chain "$bin" "$owner") || return 1
  case $(native_magic "$real") in
    7f454c46|cffaedfe|cefaedfe|feedfacf|feedface|cafebabe) ;;
    *) echo "error: $bin is not a Mach-O or ELF executable; a #! script resolves its interpreter through the caller's PATH" >&2; return 1;;
  esac
  check_deps "$real" "$owner" || return 1
  printf '%s\n' "$real"
}

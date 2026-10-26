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

# The last complete block of a UART replay: the driver ends each replay with a "." line, so the text before the final one.
uart_last_block() { awk '/^\.$/{n++; next} {a[n]=a[n]$0"\n"} END{printf "%s", a[n-1]}'; }

# Block $1 must open with the driver's "B<batch $2> <programs $3>" tag, so a buffered replay from an earlier batch is refused.
block_tag_ok() { [ "${1%%$'\n'*}" = "B$2 $3" ]; }

# Counts INTACT and DAMAGED blocks of capture $1 for batch $2 of $3 programs; intact is exactly "B<batch> <programs>", then "k <verdict>" in order (a digit flipped in place passes).
# BLOCK is the first intact block showing a failure, else the first intact one.
grade_blocks() {
  local out line cur="" first="" failing=""
  out=$(printf '%s' "$1" | awk -v b="$2" -v w="$3" '
    function finish(   i, ok, f) {
      if (nl == 0) return
      ok = (seg[0] == "B" b " " w && nl == w + 1)
      for (i = 1; ok && i <= w; i++) { f = seg[i]; sub(/ .*/, "", f); ok = (f == i - 1 && seg[i] ~ /^[0-9]+ [0-9]+$/) }
      if (ok) { print "I"; for (i = 0; i <= w; i++) print seg[i]; print "." } else print "D"
      nl = 0
    }
    /^\.$/ { finish(); next }
    { seg[nl++] = $0 }')
  BLOCK=""; INTACT=0; DAMAGED=0
  while IFS= read -r line; do
    case $line in
      D) DAMAGED=$((DAMAGED+1));;
      I) INTACT=$((INTACT+1)); cur="";;
      .) [ -n "$first" ] || first=$cur
         if [ -z "$failing" ] && ! block_all_pass "$cur"; then failing=$cur; fi;;
      *) cur="$cur$line"$'\n';;
    esac
  done <<< "$out"
  BLOCK=${failing:-$first}
}

block_all_pass() {
  local line
  while IFS= read -r line; do
    case $line in B*|"") continue;; esac
    [ "$(grade_verdict "${line#* }")" = PASS ] || return 1
  done <<< "$1"
}

# Flashes $2 with tool $1 into log $3, on device $4 if given: a bounded read-only -t first drains the FT232H's stray UART bytes, which hang the write.
flash_bitstream() {
  local bin=$1 file=$2 log=$3 dev=${4-}
  $BOUNDED "${PRIME_S:-30}" $ICEPROG_SUDO "$bin" ${dev:+-d "$dev"} -t > "$log.prime" 2>&1 || true
  $BOUNDED "${FLASH_S:-180}" $ICEPROG_SUDO "$bin" ${dev:+-d "$dev"} "$file" > "$log" 2>&1 || return 1
  grep -q 'VERIFY OK' "$log"
}

block_verdict() { printf '%s\n' "$1" | awk -v k="$2" '$1==k{print $2; exit}'; }

# ok, unexpected or unexpected-pass for program $1's result $2 against baseline names $3; a baselined program matches only a numbered FAIL.
baseline_class() {
  local name=$1 status=$2 baselined=$3 hit
  hit=$(printf '%s\n' "$baselined" | grep -Fx -- "$name" || true)
  case $status in
    PASS) if [ -n "$hit" ]; then echo unexpected-pass; else echo ok; fi;;
    "FAIL "[0-9]*) if [ -n "$hit" ]; then echo ok; else echo unexpected; fi;;
    *) echo unexpected;;
  esac
}

# One results line, "name PASS|FAIL <n>|PARSE-ERROR|MISSING", from a program's name and its verdict (empty if none arrived).
result_line() {
  local name=$1 v=${2-} g
  if [ -z "$v" ]; then echo "$name MISSING"; return 0; fi
  g=$(grade_verdict "$v")
  case $g in
    PASS) echo "$name PASS";;
    PARSE) echo "$name PARSE-ERROR";;
    *) echo "$name $g";;
  esac
}

# Grades a dual-hart UART capture $1: its last complete "DUAL" line must read c0=16 and c1=$2 (16 with hart 1 running, 0 held).
dual_verdict() {
  local last
  last=$(display_safe < "$1" | grep -E '^DUAL c0=[0-9]+ c1=[0-9]+$' | tail -n 1 || true)
  if [ "$last" = "DUAL c0=16 c1=$2" ]; then echo "dual: $last"; return 0; fi
  echo "dual: wanted 'DUAL c0=16 c1=$2', last complete line was '${last:-none}'" >&2
  return 1
}

flash_verdict() {
  local last
  last=$(display_safe < "$1" | grep -E '^FLASH id=[0-9A-F]{6} sfdp=.{4}$' | tail -n 1 || true)
  if [ "$last" = "FLASH id=$2 sfdp=SFDP" ]; then echo "flash: $last"; return 0; fi
  echo "flash: wanted 'FLASH id=$2 sfdp=SFDP', last complete line was '${last:-none}'" >&2
  return 1
}

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

ACL_PROBE_PY='
import ctypes, ctypes.util, errno, sys
libc = ctypes.CDLL(ctypes.util.find_library("c"), use_errno=True)
libc.acl_get_link_np.restype = ctypes.c_void_p
libc.acl_get_link_np.argtypes = [ctypes.c_char_p, ctypes.c_int]
libc.acl_to_text.restype = ctypes.c_void_p
libc.acl_to_text.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
acl = libc.acl_get_link_np(sys.argv[1].encode(), 0x100)
if not acl:
    sys.exit(1 if ctypes.get_errno() == errno.ENOENT else 2)
text = libc.acl_to_text(acl, None)
if not text:
    sys.exit(2)
sys.exit(0 if b":allow:" in ctypes.string_at(text) else 1)
'

acl_grants() {
  local out rc=0
  if [ "$(uname -s)" = Darwin ]; then
    python3 -I -c "$ACL_PROBE_PY" "$1" 2>/dev/null || rc=$?
    [ "$rc" -le 1 ] && return "$rc"
    return 2
  elif command -v getfacl >/dev/null 2>&1; then
    out=$(getfacl -cp "$1" 2>/dev/null) || return 2
    grep -q '^user::' <<< "$out" || return 2
    grep -Eq '^(user|group):[^:]+:.w' <<< "$out"
  else
    echo "error: getfacl is not installed, so the ACL on $1 cannot be read; install the acl package" >&2
    return 2
  fi
}

# Prints $1's resolved path once it is owned by OWNER, each directory above by OWNER or root, none group-, world- or ACL-writable.
check_path_chain() {
  local want=${1-} owner=${2-0} real comp cur="" st uid mode acl
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
    acl_grants "$cur" && acl=0 || acl=$?
    case $acl in
      0) echo "error: $cur carries an ACL that grants access beyond its mode; run 'make install-board-tools'" >&2; return 1;;
      1) ;;
      *) echo "error: cannot read the ACL on $cur" >&2; return 1;;
    esac
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

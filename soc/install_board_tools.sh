#!/bin/bash
# Installs iceprog and ftread root-owned under DEST/bin, with iceprog's bundled @executable_path/../lib dylibs under DEST/lib.
set -euo pipefail
. "$(dirname "$0")/board_verdict.sh"
DEST=${1:?usage: $0 DEST GROUP FTREAD}; GROUP=${2:?}; FTREAD=${3:?}

ICEPROG=$(command -v iceprog) || { echo "error: iceprog is not on PATH" >&2; exit 1; }
# The OSS CAD Suite's bin/iceprog is a bash wrapper around libexec/iceprog.
if [ "$(head -c 2 "$ICEPROG")" = '#!' ]; then ICEPROG=$(dirname "$ICEPROG")/../libexec/iceprog; fi
for b in "$ICEPROG" "$FTREAD"; do
  case $(native_magic "$b") in
    7f454c46|cffaedfe|cefaedfe|feedfacf|feedface|cafebabe) ;;
    *) echo "error: $b is not a Mach-O or ELF executable" >&2; exit 1;;
  esac
done

libs=()
if command -v otool >/dev/null; then
  srcdir=$(cd "$(dirname "$ICEPROG")/../lib" 2>/dev/null && pwd) || srcdir=""
  queue=("$ICEPROG")
  while [ "${#queue[@]}" -gt 0 ]; do
    f=${queue[0]}; queue=("${queue[@]:1}")
    while read -r dep; do
      name=${dep#@executable_path/../lib/}
      case " ${libs[*]-} " in *" $name "*) continue;; esac
      [ -n "$srcdir" ] && [ -f "$srcdir/$name" ] || { echo "error: $f needs $dep, not found under $srcdir" >&2; exit 1; }
      libs+=("$name"); queue+=("$srcdir/$name")
    done < <(otool -L "$f" | awk '$1 ~ /^@executable_path\/\.\.\/lib\// {print $1}')
  done
fi

sudo install -d -o root -g "$GROUP" -m 755 "$DEST" "$DEST/bin" "$DEST/lib"
sudo install -o root -g "$GROUP" -m 755 "$ICEPROG" "$DEST/bin/iceprog"
sudo install -o root -g "$GROUP" -m 755 "$FTREAD" "$DEST/bin/ftread"
for name in ${libs[@]+"${libs[@]}"}; do
  sudo install -o root -g "$GROUP" -m 755 "$srcdir/$name" "$DEST/lib/$name"
done
for f in "$DEST/bin/iceprog" "$DEST/bin/ftread" ${libs[@]+"${libs[@]/#/$DEST/lib/}"}; do check_root_binary "$f"; done
echo "installed iceprog, ftread and ${#libs[@]} bundled librar$( [ "${#libs[@]}" = 1 ] && echo y || echo ies), root-owned, in $DEST"

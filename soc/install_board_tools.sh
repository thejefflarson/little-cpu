#!/bin/bash
# Installs iceprog and ftread root-owned under DEST/bin, with iceprog's bundled @executable_path/../lib dylibs under DEST/lib
# and ftread's own (libftdi, libusb) under DEST/lib/ftread, repointed so nothing root runs loads from a user-owned prefix.
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

# ftread is repointed in a private staging copy, and its libraries live apart from iceprog's so two builds of one name cannot collide.
STAGE=$(mktemp -d)
trap 'rm -rf "$STAGE"' EXIT
cp "$FTREAD" "$STAGE/ftread"; chmod 755 "$STAGE/ftread"
FTLIBS=()
if command -v otool >/dev/null; then
  mkdir "$STAGE/lib"
  queue=("$STAGE/ftread")
  while [ "${#queue[@]}" -gt 0 ]; do
    f=${queue[0]}; queue=("${queue[@]:1}")
    while read -r dep; do
      case $dep in
        /usr/lib/*|/System/*|@executable_path/../lib/ftread/*) continue;;
        /*) ;;
        *) echo "error: $f loads $dep, which cannot be bundled" >&2; exit 1;;
      esac
      name=$(basename "$dep")
      if [ ! -f "$STAGE/lib/$name" ]; then
        [ -f "$dep" ] || { echo "error: $f needs $dep, which does not exist" >&2; exit 1; }
        cp "$dep" "$STAGE/lib/$name"; chmod 755 "$STAGE/lib/$name"
        install_name_tool -id "@executable_path/../lib/ftread/$name" "$STAGE/lib/$name"
        FTLIBS+=("$name"); queue+=("$STAGE/lib/$name")
      fi
      install_name_tool -change "$dep" "@executable_path/../lib/ftread/$name" "$f"
    done < <(otool -L "$f" | tail -n +2 | awk '{print $1}')
  done
  for f in "$STAGE/ftread" ${FTLIBS[@]+"${FTLIBS[@]/#/$STAGE/lib/}"}; do codesign --force -s - "$f"; done
fi

sudo install -d -o root -g "$GROUP" -m 755 "$DEST" "$DEST/bin" "$DEST/lib"
sudo install -o root -g "$GROUP" -m 755 "$ICEPROG" "$DEST/bin/iceprog"
sudo install -o root -g "$GROUP" -m 755 "$STAGE/ftread" "$DEST/bin/ftread"
[ "${#FTLIBS[@]}" -eq 0 ] || sudo install -d -o root -g "$GROUP" -m 755 "$DEST/lib/ftread"
for name in ${FTLIBS[@]+"${FTLIBS[@]}"}; do
  sudo install -o root -g "$GROUP" -m 755 "$STAGE/lib/$name" "$DEST/lib/ftread/$name"
done
for name in ${libs[@]+"${libs[@]}"}; do
  sudo install -o root -g "$GROUP" -m 755 "$srcdir/$name" "$DEST/lib/$name"
done
for f in "$DEST/bin/iceprog" "$DEST/bin/ftread" ${libs[@]+"${libs[@]/#/$DEST/lib/}"}; do check_root_binary "$f"; done
n=$(( ${#libs[@]} + ${#FTLIBS[@]} ))
echo "installed iceprog, ftread and $n bundled librar$( [ "$n" = 1 ] && echo y || echo ies), root-owned, in $DEST"

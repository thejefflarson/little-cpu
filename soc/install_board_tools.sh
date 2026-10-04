#!/bin/bash
# Installs iceprog and ftread root-owned under DEST/bin, iceprog's @executable_path/../lib dylibs under DEST/lib and ftread's
# own under DEST/lib/ftread, repointed; each is copied to one stage, checked there, and `sudo install`ed only from it.
set -euo pipefail
. "$(dirname "$0")/board_verdict.sh"
DEST=${1:?usage: $0 DEST GROUP FTREAD}; GROUP=${2:?}; FTREAD=${3:?}

ICEPROG=$(command -v iceprog) || { echo "error: iceprog is not on PATH" >&2; exit 1; }
# The OSS CAD Suite's bin/iceprog is a bash wrapper around libexec/iceprog.
if [ "$(head -c 2 "$ICEPROG")" = '#!' ]; then ICEPROG=$(dirname "$ICEPROG")/../libexec/iceprog; fi

STAGE=$(mktemp -d)
trap 'rm -rf "$STAGE"' EXIT
mkdir "$STAGE/bin" "$STAGE/lib" "$STAGE/lib/ftread"
cp "$ICEPROG" "$STAGE/bin/iceprog"; cp "$FTREAD" "$STAGE/bin/ftread"
chmod 755 "$STAGE/bin/iceprog" "$STAGE/bin/ftread"
for b in "$STAGE/bin/iceprog" "$STAGE/bin/ftread"; do
  case $(native_magic "$b") in
    7f454c46|cffaedfe|cefaedfe|feedfacf|feedface|cafebabe) ;;
    *) echo "error: $b is not a Mach-O or ELF executable" >&2; exit 1;;
  esac
done

libs=()
if command -v otool >/dev/null; then
  srcdir=$(cd "$(dirname "$ICEPROG")/../lib" 2>/dev/null && pwd) || srcdir=""
  queue=("$STAGE/bin/iceprog")
  while [ "${#queue[@]}" -gt 0 ]; do
    f=${queue[0]}; queue=("${queue[@]:1}")
    while read -r dep; do
      name=${dep#@executable_path/../lib/}
      case $name in */*|.*|ftread|'') echo "error: $f needs $dep, which names no file directly under lib/" >&2; exit 1;; esac
      case " ${libs[*]-} " in *" $name "*) continue;; esac
      [ -n "$srcdir" ] && [ -f "$srcdir/$name" ] || { echo "error: $f needs $dep, not found under $srcdir" >&2; exit 1; }
      cp "$srcdir/$name" "$STAGE/lib/$name"; chmod 755 "$STAGE/lib/$name"
      libs+=("$name"); queue+=("$STAGE/lib/$name")
    done < <(otool -L "$f" | awk '$1 ~ /^@executable_path\/\.\.\/lib\// {print $1}')
  done
fi

FTLIBS=()
if command -v otool >/dev/null; then
  queue=("$STAGE/bin/ftread")
  while [ "${#queue[@]}" -gt 0 ]; do
    f=${queue[0]}; queue=("${queue[@]:1}")
    while read -r dep; do
      case $dep in
        /System/Volumes/*) echo "error: $f loads $dep, on the writable data volume" >&2; exit 1;;
        /usr/lib/*|/System/Library/*|@executable_path/../lib/ftread/*) continue;;
        /*) ;;
        *) echo "error: $f loads $dep, which cannot be bundled" >&2; exit 1;;
      esac
      name=$(basename "$dep")
      if [ ! -f "$STAGE/lib/ftread/$name" ]; then
        [ -f "$dep" ] || { echo "error: $f needs $dep, which does not exist" >&2; exit 1; }
        cp "$dep" "$STAGE/lib/ftread/$name"; chmod 755 "$STAGE/lib/ftread/$name"
        install_name_tool -id "@executable_path/../lib/ftread/$name" "$STAGE/lib/ftread/$name"
        FTLIBS+=("$name"); queue+=("$STAGE/lib/ftread/$name")
      fi
      install_name_tool -change "$dep" "@executable_path/../lib/ftread/$name" "$f"
    done < <(otool -L "$f" | tail -n +2 | awk '{print $1}')
  done
  for f in "$STAGE/bin/ftread" ${FTLIBS[@]+"${FTLIBS[@]/#/$STAGE/lib/ftread/}"}; do codesign --force -s - "$f"; done
fi

sudo install -d -o root -g "$GROUP" -m 755 "$DEST" "$DEST/bin" "$DEST/lib"
sudo install -o root -g "$GROUP" -m 755 "$STAGE/bin/iceprog" "$DEST/bin/iceprog"
sudo install -o root -g "$GROUP" -m 755 "$STAGE/bin/ftread" "$DEST/bin/ftread"
[ "${#FTLIBS[@]}" -eq 0 ] || sudo install -d -o root -g "$GROUP" -m 755 "$DEST/lib/ftread"
for name in ${FTLIBS[@]+"${FTLIBS[@]}"}; do
  sudo install -o root -g "$GROUP" -m 755 "$STAGE/lib/ftread/$name" "$DEST/lib/ftread/$name"
done
for name in ${libs[@]+"${libs[@]}"}; do
  sudo install -o root -g "$GROUP" -m 755 "$STAGE/lib/$name" "$DEST/lib/$name"
done
for f in "$DEST/bin/iceprog" "$DEST/bin/ftread" ${libs[@]+"${libs[@]/#/$DEST/lib/}"}; do check_root_binary "$f" >/dev/null; done
n=$(( ${#libs[@]} + ${#FTLIBS[@]} ))
echo "installed iceprog, ftread and $n bundled librar$( [ "$n" = 1 ] && echo y || echo ies), root-owned, in $DEST"

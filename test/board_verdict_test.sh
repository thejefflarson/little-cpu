#!/bin/bash
# Grades verdict parsing, the display filter and the root-binary check; $1/$2/$3 override the library, suite script and
# installer (probe_gates.sh's mutants).
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
LIB=${1:-$HERE/../soc/board_verdict.sh}
[ -f "$LIB" ] || { echo "error: no verdict library at $LIB" >&2; exit 1; }
SUITE=${2:-$HERE/../soc/run_suite_board.sh}
INSTALL=${3:-$HERE/../soc/install_board_tools.sh}
. "$LIB"

WORK=$(mktemp -d "${TMPDIR:-/tmp}/board_verdict.XXXXXX")
trap 'rm -rf "$WORK"' EXIT
cd "$WORK"
fail=0

expect() {
  local v=$1 want=$2 got
  got=$(grade_verdict "$v" 2>/dev/null) || got="shell error $?"
  if [ "$got" != "$want" ]; then
    echo "FAIL: verdict '$v' graded '$got', expected '$want'" >&2
    fail=1
  fi
}

expect 1 PASS
expect 7 "FAIL 3"
expect 08 "FAIL 4"
expect 001 PASS
expect '' PARSE
expect abc PARSE
expect -3 PARSE
expect 1234567890 PARSE

# 'v[$(:>p)]' fits the length cap and names a set local, so only the character-class arm stops it.
hostile=("x[\$(touch $WORK/pwned)]" "x[\`touch $WORK/pwned\`]" "a[\$(touch $WORK/pwned)]+1" 'v[$(:>p)]')
for v in "${hostile[@]}"; do
  expect "$v" PARSE
done

if [ -e "$WORK/pwned" ] || [ -e "$WORK/p" ]; then
  echo "FAIL: a hostile verdict executed a command" >&2
  fail=1
fi

hostile_text=$'ok 1\n\033]0;owned\007\033[31mred\033[0m\r\x80\xff tail\n'
shown=$(printf '%s' "$hostile_text" | display_safe | od -An -c | tr -d ' \n')
case $shown in
  *033*|*\\a*|*\\r*|*200*|*377*) echo "FAIL: control or high bytes reached the displayed text: $shown" >&2; fail=1;;
esac
[ "$(printf 'a\nb\n' | display_safe)" = $'a\nb' ] || { echo "FAIL: display_safe dropped printable text or newlines" >&2; fail=1; }

# Every displayed use of the raw capture or parsed block must go through the filter.
display_pat='(echo|sed .s/\^/).*[$](raw|block)|printf .%s. "[$](raw|block)" [|] sed'
unfiltered=$(grep -E "$display_pat" "$SUITE" | grep -v display_safe || true)
if [ -n "$unfiltered" ]; then
  echo "FAIL: UART text reaches the terminal unfiltered in $SUITE: $unfiltered" >&2
  fail=1
fi

mkdir "$WORK/stubs"
cat > "$WORK/stubs/otool" <<'STUB'
#!/bin/bash
echo "$2:"
[ -f "$STUB_DEPS/$(basename "$2").deps" ] && sed 's/^/	/; s/$/ (compatibility version 1.0.0)/' "$STUB_DEPS/$(basename "$2").deps"
exit 0
STUB
chmod 755 "$WORK/stubs/otool"
cat > "$WORK/stubs/readelf" <<'STUB'
#!/bin/bash
[ -f "$STUB_DEPS/$(basename "$2").rpath" ] || exit 0
echo ' 0x000000000000001d (RUNPATH)            Library runpath: ['"$(cat "$STUB_DEPS/$(basename "$2").rpath")"']'
STUB
cat > "$WORK/stubs/ls" <<'STUB'
#!/bin/bash
if [ "$1" = -lde ] && grep -qxF -- "$2" "$STUB_ACL"; then echo "drwxr-xr-x+ 2 u g 64 Jan 1 00:00 $2"; echo ' 0: user:evil allow add_file'; exit 0; fi
exec /bin/ls "$@"
STUB
cat > "$WORK/stubs/getfacl" <<'STUB'
#!/bin/bash
for a; do f=$a; done
printf 'user::rwx\ngroup::r-x\nother::r-x\n'
grep -qxF -- "$f" "$STUB_ACL" && echo 'user:evil:rwx'
exit 0
STUB
chmod 755 "$WORK/stubs/readelf" "$WORK/stubs/ls" "$WORK/stubs/getfacl"
export STUB_DEPS=$WORK/deps; mkdir "$STUB_DEPS"
export STUB_ACL=$WORK/acl; : > "$STUB_ACL"
PATH=$WORK/stubs:$PATH

bin=$WORK/tool; mkdir "$WORK/d"; printf '\177ELF' > "$bin"
me=$(id -u)
chmod 755 "$bin"; chmod 755 "$WORK"
expect_bin() {
  local want=$1 desc=$2 owner=${3-$me}
  if check_root_binary "$bin" "$owner" >/dev/null 2>&1; then got=accept; else got=refuse; fi
  if [ "$got" != "$want" ]; then echo "FAIL: $desc: got $got, expected $want" >&2; fail=1; fi
}
expect_bin accept "owner-only-writable file in an owner-only-writable directory"
expect_bin refuse "file owned by someone else" 99999
chmod 775 "$bin"; expect_bin refuse "group-writable file"
chmod 757 "$bin"; expect_bin refuse "world-writable file"
chmod 755 "$bin"; chmod 775 "$WORK"; expect_bin refuse "group-writable parent directory"
chmod 777 "$WORK"; expect_bin refuse "world-writable parent directory"
chmod 755 "$WORK"
ln -s "$bin" "$WORK/link"
if check_root_binary "$WORK/link" "$me" >/dev/null 2>&1; then echo "FAIL: a symlink was accepted" >&2; fail=1; fi
if check_root_binary "$WORK/missing" "$me" >/dev/null 2>&1; then echo "FAIL: a missing path was accepted" >&2; fail=1; fi
printf '#!/usr/bin/env bash\nexec true\n' > "$WORK/script"; chmod 755 "$WORK/script"
if check_root_binary "$WORK/script" "$me" >/dev/null 2>&1; then echo "FAIL: a #! script was accepted" >&2; fail=1; fi
: > "$WORK/empty"; chmod 755 "$WORK/empty"
if check_root_binary "$WORK/empty" "$me" >/dev/null 2>&1; then echo "FAIL: an empty file was accepted" >&2; fail=1; fi

chmod 755 "$WORK" "$bin"
mkdir -p "$WORK/a/b" "$WORK/lib"; chmod 755 "$WORK/a" "$WORK/a/b" "$WORK/lib"
cp "$bin" "$WORK/a/b/tool"; chmod 755 "$WORK/a/b/tool"
expect_at() {
  local want=$1 desc=$2 path=$3
  if check_root_binary "$path" "$me" >/dev/null 2>&1; then got=accept; else got=refuse; fi
  if [ "$got" != "$want" ]; then echo "FAIL: $desc: got $got, expected $want" >&2; fail=1; fi
}
expect_at accept "nested binary in owner-only-writable directories" "$WORK/a/b/tool"
chmod 777 "$WORK/a"; expect_at refuse "world-writable ancestor above the parent" "$WORK/a/b/tool"
chmod 775 "$WORK/a"; expect_at refuse "group-writable ancestor above the parent" "$WORK/a/b/tool"
chmod 755 "$WORK/a"
ln -s "$WORK/a" "$WORK/alias"
expect_at accept "a symlinked ancestor resolves before the walk" "$WORK/alias/b/tool"
chmod 777 "$WORK/a"; expect_at refuse "a symlinked ancestor that is writable once resolved" "$WORK/alias/b/tool"
chmod 755 "$WORK/a"
printed=$(check_root_binary "$WORK/alias/b/tool" "$me" 2>/dev/null) || printed=""
[ "$printed" = "$(cd -P "$WORK/a/b" && pwd -P)/tool" ] \
  || { echo "FAIL: the checked path printed is '$printed', not the resolved one a caller must run" >&2; fail=1; }
resolve_path "$WORK/a/b" | grep -qxF "$(cd -P "$WORK/a/b" && pwd -P)" || { echo "FAIL: resolve_path" >&2; fail=1; }
grep -qF 'ICEPROG_BIN=$("$ROOT/soc/check_root_binary.sh" "$ICEPROG_BIN")' "$SUITE" \
  && grep -qF 'FTREAD=$("$ROOT/soc/check_root_binary.sh" "$FTREAD")' "$SUITE" \
  || { echo "FAIL: $SUITE runs a path other than the one check_root_binary.sh resolved" >&2; fail=1; }
mkdir -p "$WORK/a b/*"; chmod 755 "$WORK/a b" "$WORK/a b/*"
cp "$bin" "$WORK/a b/*/tool"; chmod 755 "$WORK/a b/*/tool"
expect_at accept "a path with a space and a glob character in it" "$WORK/a b/*/tool"
chmod 777 "$WORK/a b"; expect_at refuse "a writable directory whose name has a space" "$WORK/a b/*/tool"
chmod 755 "$WORK/a b"
printf '%s\n' "$(cd -P "$WORK/a" && pwd -P)" > "$STUB_ACL"
expect_at refuse "an ancestor whose ACL grants what its mode does not" "$WORK/a/b/tool"
: > "$STUB_ACL"

printf '\177ELF' > "$WORK/lib/libx.dylib"; printf '\177ELF' > "$WORK/lib/liby.dylib"
chmod 755 "$WORK/lib/libx.dylib" "$WORK/lib/liby.dylib"
printf '/usr/lib/libSystem.B.dylib\n/System/Library/Frameworks/IOKit.framework/IOKit\n%s\n' "$WORK/lib/libx.dylib" > "$STUB_DEPS/tool.deps"
expect_bin accept "system libraries and a root-owned library"
chmod 777 "$WORK/lib/libx.dylib"; expect_bin refuse "world-writable library dependency"
chmod 755 "$WORK/lib/libx.dylib"
chmod 775 "$WORK/lib"; expect_bin refuse "library in a group-writable directory"
chmod 755 "$WORK/lib"
printf '%s\n' "$WORK/lib/liby.dylib" > "$STUB_DEPS/libx.dylib.deps"
expect_bin accept "transitive library, all clean"
chmod 666 "$WORK/lib/liby.dylib"; expect_bin refuse "world-writable library two hops down"
chmod 755 "$WORK/lib/liby.dylib"; rm "$STUB_DEPS/libx.dylib.deps"
printf '%s/lib/gone.dylib\n' "$WORK" > "$STUB_DEPS/tool.deps"; expect_bin refuse "library that does not exist"
printf '@rpath/libx.dylib\n' > "$STUB_DEPS/tool.deps"; expect_bin refuse "@rpath dependency"
printf '/usr/lib/../../%s/lib/libx.dylib\n' "${WORK#/}" > "$STUB_DEPS/tool.deps"; expect_bin refuse "system prefix climbed out of with .."
printf '/System/Volumes/Data%s/lib/libx.dylib\n' "$WORK" > "$STUB_DEPS/tool.deps"; expect_bin refuse "a library on the /System/Volumes data volume"
printf '@executable_path/lib/libx.dylib\n' > "$STUB_DEPS/tool.deps"; expect_bin accept "@executable_path library beside the binary"
chmod 777 "$WORK/lib"; expect_bin refuse "@executable_path library in a world-writable directory"
chmod 755 "$WORK/lib"
rm -f "$STUB_DEPS/tool.deps"
printf '%s/lib:$ORIGIN/lib' "$WORK" > "$STUB_DEPS/tool.rpath"; expect_bin accept "a RUNPATH of clean directories"
chmod 777 "$WORK/lib"; expect_bin refuse "a world-writable RUNPATH directory"
chmod 755 "$WORK/lib"
printf 'lib' > "$STUB_DEPS/tool.rpath"; expect_bin refuse "a relative RUNPATH"
rm -f "$STUB_DEPS/tool.rpath"
if grep -nE '/System/\*|/System/Volumes/[^)]*\) *continue' "$INSTALL"; then
  echo "FAIL: $INSTALL trusts all of /System, the writable data volume included" >&2; fail=1
fi

[ "$fail" -eq 0 ] || exit 1
echo "board verdict parse OK: hostile verdicts rejected, nothing executed, display filtered, root binaries checked"

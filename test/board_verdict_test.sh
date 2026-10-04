#!/bin/bash
# Grades verdict parsing, the display filter and the root-binary check; $1/$2 override the library and suite script (probe_gates.sh's mutants).
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
LIB=${1:-$HERE/../soc/board_verdict.sh}
[ -f "$LIB" ] || { echo "error: no verdict library at $LIB" >&2; exit 1; }
SUITE=${2:-$HERE/../soc/run_suite_board.sh}
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

bin=$WORK/tool; mkdir "$WORK/d"; printf '\177ELF' > "$bin"
me=$(id -u)
chmod 755 "$bin"; chmod 755 "$WORK"
expect_bin() {
  local want=$1 desc=$2 owner=${3-$me}
  if check_root_binary "$bin" "$owner" 2>/dev/null; then got=accept; else got=refuse; fi
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
if check_root_binary "$WORK/link" "$me" 2>/dev/null; then echo "FAIL: a symlink was accepted" >&2; fail=1; fi
if check_root_binary "$WORK/missing" "$me" 2>/dev/null; then echo "FAIL: a missing path was accepted" >&2; fail=1; fi
printf '#!/usr/bin/env bash\nexec true\n' > "$WORK/script"; chmod 755 "$WORK/script"
if check_root_binary "$WORK/script" "$me" 2>/dev/null; then echo "FAIL: a #! script was accepted" >&2; fail=1; fi
: > "$WORK/empty"; chmod 755 "$WORK/empty"
if check_root_binary "$WORK/empty" "$me" 2>/dev/null; then echo "FAIL: an empty file was accepted" >&2; fail=1; fi

[ "$fail" -eq 0 ] || exit 1
echo "board verdict parse OK: hostile verdicts rejected, nothing executed, display filtered, root binaries checked"

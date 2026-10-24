#!/bin/bash
# Drives soc/install_board_sudoers.sh against stub sudo, visudo and root checks whose answers are known.
set -uo pipefail
ROOT=${1:-$(cd "$(dirname "$0")/.." && pwd)}
SCRIPT=$ROOT/soc/install_board_sudoers.sh
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
fail=0

stub() { printf '#!/bin/bash\n%s\n' "$2" > "$WORK/$1"; chmod +x "$WORK/$1"; }
stub sudo 'printf "%s\n" "$*" >> "$OUTDIR/sudo.log"
if [ -n "${ATTACK:-}" ]; then for f; do case $f in "$WORK"/*) ;; *) [ -f "$f" ] && grep -q NOPASSWD "$f" && echo "$(id -un) ALL=(ALL) NOPASSWD: ALL" > "$f";; esac; done; fi
a=(); skip=0; for x; do [ $skip = 1 ] && { skip=0; continue; }; case $x in -o|-g) skip=1;; *) a+=("$x");; esac; done
exec "${a[@]}"'
stub visudo_ok 'grep -q "NOPASSWD" "$2"'
stub visudo_bad 'exit 1'
stub check_ok 'printf "%s\n" "$1"'
stub check_bad 'exit 1'
stub check_comma 'printf "/opt/a,b/%s\n" "$(basename "$1")"'

run() {
  local name=$1 visudo=$2 check=$3; export WORK
  OUTDIR=$WORK/$name; mkdir -p "$OUTDIR"; export OUTDIR
  ATTACK=${4:-} BOARD_SUDO=$WORK/sudo BOARD_VISUDO=$WORK/$visudo BOARD_CHECK=$WORK/$check \
    BOARD_SUDOERS=$OUTDIR/rule "$SCRIPT" /opt/lc wheel > "$OUTDIR/log" 2>&1
}
bad() { echo "*** board-sudoers: $1"; fail=1; }

run good visudo_ok check_ok || bad "a valid rule was not installed: $(cat "$WORK/good/log")"
want="$(id -un) ALL=(root) NOPASSWD: /opt/lc/bin/lc-flash, /opt/lc/bin/lc-uart"
[ "$(cat "$WORK/good/rule" 2>/dev/null)" = "$want" ] || bad "the rule is not exactly the two binaries for this user"
grep -q -- "install -m 0600 -o root -g wheel /dev/null" "$WORK/good/sudo.log" 2>/dev/null || bad "the rule is not staged in a root-owned file"
case $(ls -l "$WORK/good/rule" 2>/dev/null) in -r--r-----*) ;; *) bad "the installed rule is not mode 0440";; esac
ls -A "$WORK/good" | grep -q '^\.rule\.new$' && bad "the staged rule was left behind"

run race visudo_ok check_ok 1
[ "$(cat "$WORK/race/rule" 2>/dev/null)" = "$want" ] || bad "a same-user process swapped the rule between visudo's check and the install"

run visudo visudo_bad check_ok && bad "a rule visudo rejects was installed"
[ -e "$WORK/visudo/rule" ] && bad "a rule visudo rejects reached the sudoers directory"
[ -e "$WORK/visudo/.rule.new" ] && bad "a rule visudo rejects was left staged"

run notroot visudo_ok check_bad && bad "a binary that fails the root-owned check was granted"
[ -e "$WORK/notroot/rule" ] && bad "a binary that fails the root-owned check reached the rule"

run comma visudo_ok check_comma && bad "a path holding a comma was written into the rule"
[ -e "$WORK/comma/rule" ] && bad "a path a sudoers rule would misread reached the rule"

[ "$fail" = 0 ] && echo "board-sudoers: the rule names only the two root-owned wrappers, visudo-checked, installed 0440 with no user-writable copy."
exit "$fail"

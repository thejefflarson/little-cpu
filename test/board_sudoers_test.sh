#!/bin/bash
# Drives soc/install_board_sudoers.sh against stub sudo, visudo and root checks whose answers are known.
set -uo pipefail
ROOT=${1:-$(cd "$(dirname "$0")/.." && pwd)}
SCRIPT=$ROOT/soc/install_board_sudoers.sh
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
fail=0

stub() { printf '#!/bin/bash\n%s\n' "$2" > "$WORK/$1"; chmod +x "$WORK/$1"; }
stub sudo 'printf "%s\n" "$*" > "$OUTDIR/sudo.args"; shift 7; cp "$1" "$2"'
stub visudo_ok 'grep -q "NOPASSWD" "$2"'
stub visudo_bad 'exit 1'
stub check_ok 'printf "%s\n" "$1"'
stub check_bad 'exit 1'
stub check_comma 'printf "/opt/a,b/%s\n" "$(basename "$1")"'

run() {
  local name=$1 visudo=$2 check=$3
  OUTDIR=$WORK/$name; mkdir -p "$OUTDIR"; export OUTDIR
  BOARD_SUDO=$WORK/sudo BOARD_VISUDO=$WORK/$visudo BOARD_CHECK=$WORK/$check \
    BOARD_SUDOERS=$OUTDIR/rule "$SCRIPT" /opt/lc wheel > "$OUTDIR/log" 2>&1
}
bad() { echo "*** board-sudoers: $1"; fail=1; }

run good visudo_ok check_ok || bad "a valid rule was not installed: $(cat "$WORK/good/log")"
want="$(id -un) ALL=(root) NOPASSWD: /opt/lc/bin/iceprog, /opt/lc/bin/ftread"
[ "$(cat "$WORK/good/rule" 2>/dev/null)" = "$want" ] || bad "the rule is not exactly the two binaries for this user"
grep -q -- "-m 0440 -o root -g wheel" "$WORK/good/sudo.args" 2>/dev/null || bad "the rule is not installed root-owned and 0440"

run visudo visudo_bad check_ok && bad "a rule visudo rejects was installed"
[ -e "$WORK/visudo/rule" ] && bad "a rule visudo rejects reached the sudoers directory"

run notroot visudo_ok check_bad && bad "a binary that fails the root-owned check was granted"
[ -e "$WORK/notroot/rule" ] && bad "a binary that fails the root-owned check reached the rule"

run comma visudo_ok check_comma && bad "a path holding a comma was written into the rule"
[ -e "$WORK/comma/rule" ] && bad "a path a sudoers rule would misread reached the rule"

[ "$fail" = 0 ] && echo "board-sudoers: the rule names only the two root-owned binaries, visudo-checked, installed 0440."
exit "$fail"

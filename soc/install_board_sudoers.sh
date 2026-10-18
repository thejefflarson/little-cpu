#!/bin/bash
# Grants passwordless sudo for DEST/bin/iceprog and ftread only, each after the board targets' root-owned check.
set -euo pipefail
DEST=${1:?usage: $0 DEST GROUP}; GROUP=${2:?usage: $0 DEST GROUP}
SUDO=${BOARD_SUDO:-sudo}
VISUDO=${BOARD_VISUDO:-/usr/sbin/visudo}
CHECK=${BOARD_CHECK:-$(dirname "$0")/check_root_binary.sh}
OUT=${BOARD_SUDOERS:-/etc/sudoers.d/little-cpu}
NAME=$(id -un)

case $NAME in ''|*[!A-Za-z0-9._-]*) echo "error: user name '$NAME' cannot be written into a sudoers rule" >&2; exit 1;; esac

paths=()
for b in iceprog ftread; do
  p=$("$CHECK" "$DEST/bin/$b") || {
    echo "error: $DEST/bin/$b is not a root-owned binary; run \`make install-board-tools\` first" >&2; exit 1; }
  case $p in /*) ;; *) echo "error: $b resolved to '$p', not an absolute path" >&2; exit 1;; esac
  case $p in *[!A-Za-z0-9/._-]*) echo "error: $p holds a character a sudoers rule would misread" >&2; exit 1;; esac
  paths+=("$p")
done

# The rule reaches root over a pipe into a root-owned file, so no user-writable copy sits between visudo's check and the install.
RULE=$(printf '%s ALL=(root) NOPASSWD: %s, %s' "$NAME" "${paths[0]}" "${paths[1]}")
NEW=$(dirname "$OUT")/.$(basename "$OUT").new
trap '"$SUDO" rm -f "$NEW"' EXIT
"$SUDO" install -m 0600 -o root -g "$GROUP" /dev/null "$NEW"
printf '%s\n' "$RULE" | "$SUDO" tee "$NEW" >/dev/null
"$SUDO" "$VISUDO" -cf "$NEW" >/dev/null || { echo "error: visudo rejected the rule; nothing installed" >&2; exit 1; }
"$SUDO" chmod 0440 "$NEW"
"$SUDO" mv -f "$NEW" "$OUT"
trap - EXIT
echo "installed $OUT: $RULE"

#!/bin/bash
# usage: flash_board.sh IPROG FILE [DEVICE [OFFSET]]. Flashes FILE (at byte OFFSET, past the bitstream, when given) through flash_bitstream (a bounded -t drain, a bounded write, VERIFY OK required).
set -uo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
. "$ROOT/soc/board_verdict.sh"
BOUNDED="python3 $ROOT/soc/bounded.py"
ICEPROG_SUDO=${ICEPROG_SUDO-}
LOG=${FLASH_LOG:-$ROOT/build/prog.flash.log}
mkdir -p "$(dirname "$LOG")"
flash_bitstream "${1:?usage: $0 IPROG FILE [DEVICE [OFFSET]]}" "${2:?usage: $0 IPROG FILE [DEVICE [OFFSET]]}" "$LOG" "${3-}" "${4-}"; rc=$?
display_safe < "$LOG" | sed 's/^/   | /'
if [ "$rc" != 0 ]; then echo "FLASH FAILED: no VERIFY OK in time (log: $LOG)" >&2; exit 1; fi
echo "flash: VERIFY OK"

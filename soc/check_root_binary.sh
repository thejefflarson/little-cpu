#!/bin/bash
set -uo pipefail
. "$(dirname "$0")/board_verdict.sh"
for b in "$@"; do check_root_binary "$b" || exit 1; done

#!/bin/bash
# usage: board_stamp.sh write BIN ROM OSC ROMHEX...   |   board_stamp.sh check BIN REQUESTED_ROM REQUESTED_OSC
# BIN.stamp records the BOARD_ROM, BOARD_OSC, ROM image digest and bitstream digest BIN was packed from; check refuses a request that names another build.
set -uo pipefail
digest() { cat "$@" | shasum -a 256 | cut -d' ' -f1; }
field() { sed -n "s/^$2=//p" "$1.stamp"; }
mode=${1:?usage: $0 write|check ...}
bin=${2:?usage: $0 write|check BIN ...}
case $mode in
  write)
    rom=${3:?rom} osc=${4:?osc}; shift 4
    printf 'rom=%s\nosc=%s\nrom_digest=%s\nbin_digest=%s\n' "$rom" "$osc" "$(digest "$@")" "$(digest "$bin")" > "$bin.stamp"
    ;;
  check)
    want_rom=${3-} want_osc=${4-}
    [ -s "$bin.stamp" ] || { echo "*** $bin has no stamp, so what it holds is unknown. Run make bitstream." >&2; exit 1; }
    have_rom=$(field "$bin" rom) have_osc=$(field "$bin" osc)
    if [ "$(field "$bin" bin_digest)" != "$(digest "$bin")" ]; then
      echo "*** $bin changed after it was stamped (stamp: BOARD_ROM=$have_rom BOARD_OSC=$have_osc). Run make bitstream." >&2; exit 1
    fi
    if { [ -n "$want_rom" ] && [ "$want_rom" != "$have_rom" ]; } || { [ -n "$want_osc" ] && [ "$want_osc" != "$have_osc" ]; }; then
      echo "*** refusing to flash: $bin was built from BOARD_ROM=$have_rom BOARD_OSC=$have_osc," >&2
      echo "*** but this request names BOARD_ROM=${want_rom:-(unset)} BOARD_OSC=${want_osc:-(unset)}." >&2
      echo "*** Run make bitstream with the request's variables, or drop them to flash what is built." >&2
      exit 1
    fi
    echo "flashing $bin: BOARD_ROM=$have_rom BOARD_OSC=$have_osc"
    ;;
  *) echo "usage: $0 write|check ..." >&2; exit 2;;
esac

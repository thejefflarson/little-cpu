#!/bin/bash
MOUNT=${ICELINK_MOUNT:-/Volumes/iCELink}
LIMIT_S=${ICELINK_EJECT_S:-20}
mounts=$(mount 2>/dev/null || true)
if grep -qF " on $MOUNT (" <<< "$mounts"; then
  python3 "$(dirname "$0")/bounded.py" "$LIMIT_S" diskutil eject "$MOUNT" >/dev/null 2>&1
  status=$?
  if [ "$status" = 124 ]; then
    echo "icelink_eject: warning: ejecting $MOUNT stalled past ${LIMIT_S}s; Finder may hang. Unplug the board or eject the volume in Disk Utility." >&2
  elif [ "$status" != 0 ]; then
    echo "icelink_eject: warning: ejecting $MOUNT failed" >&2
  fi
fi
exit 0


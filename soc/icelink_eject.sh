#!/bin/bash
# Ejects the iCELink debugger's DAPLink volume when it is mounted; Finder can hang on it.
# A no-op where the volume is absent, and an eject that fails warns without failing the caller.
MOUNT=${ICELINK_MOUNT:-/Volumes/iCELink}
if mount 2>/dev/null | grep -qF " on $MOUNT ("; then
  diskutil eject "$MOUNT" >/dev/null 2>&1 || echo "icelink_eject: warning: ejecting $MOUNT failed" >&2
fi
exit 0

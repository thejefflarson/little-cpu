#!/bin/sh
# Fetches sky130_fd_sc_hd's behavioral Verilog, verified like nano-liberty-setup, then
# flattens cells/**/*.v and models/**/*.v (unique basenames) into one -I directory,
# rewriting their `include paths to match.
set -eu

URL=$1
DIGEST=$2
DEST=$3

if command -v shasum >/dev/null 2>&1; then sha='shasum -a 256';
elif command -v sha256sum >/dev/null 2>&1; then sha='sha256sum';
else
  echo "neither shasum nor sha256sum is on PATH; refusing to fetch a cell library" >&2
  echo "this machine cannot verify." >&2
  exit 1
fi

TARBALL="$DEST.tar.gz"
STAMP="$DEST/.verified-sha256"

if [ -f "$TARBALL" ] && [ "$($sha "$TARBALL" | cut -d ' ' -f 1)" = "$DIGEST" ]; then
  echo "$TARBALL already verified."
else
  mkdir -p "$(dirname "$TARBALL")"
  tmp=$(mktemp "$(dirname "$TARBALL")"/.download.XXXXXX)
  echo "fetching $URL"
  curl -fsSL -o "$tmp" "$URL"
  got=$($sha "$tmp" | cut -d ' ' -f 1)
  if [ "$got" != "$DIGEST" ]; then
    echo "SHA-256 MISMATCH for $TARBALL -- refusing to keep it:" >&2
    echo "  expected : $DIGEST" >&2
    echo "  actual   : $got" >&2
    rm -f "$tmp"
    exit 1
  fi
  echo "sha256 ok: $got"
  mv "$tmp" "$TARBALL"
fi

if [ -f "$STAMP" ] && [ "$(cat "$STAMP")" = "$DIGEST" ] && [ -n "$(ls -A "$DEST" 2>/dev/null)" ]; then
  echo "$DEST already extracted for $DIGEST."
  exit 0
fi

rm -rf "$DEST"
mkdir -p "$DEST"
workdir=$(mktemp -d "$(dirname "$DEST")"/.extract.XXXXXX)
trap 'rm -rf "$workdir"' EXIT
tar -xzf "$TARBALL" -C "$workdir"

root=$(find "$workdir" -mindepth 1 -maxdepth 1 -type d | head -1)
if [ -z "$root" ]; then
  echo "error: $TARBALL did not extract to a single top-level directory." >&2
  exit 1
fi

find "$root/cells" "$root/models" -name '*.v' -exec cp -n {} "$DEST"/ \;

python3 - "$DEST" <<'PYEOF'
import glob
import os
import re
import sys

dest = sys.argv[1]
pattern = re.compile(r'`include\s+"([^"]*/)?([^"/]+)"')
for path in glob.glob(os.path.join(dest, "*.v")):
    with open(path) as f:
        text = f.read()
    rewritten = pattern.sub(lambda m: '`include "%s"' % m.group(2), text)
    if rewritten != text:
        with open(path, "w") as f:
            f.write(rewritten)
PYEOF

printf '%s' "$DIGEST" > "$STAMP"
echo "extracted $(find "$DEST" -name '*.v' | wc -l | tr -d ' ') Verilog models into $DEST"

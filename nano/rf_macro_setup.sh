#!/bin/sh
# Fetches the register-file macro's LEF, liberty and GDS, each verified against its SHA-256 before it is kept,
# into the tool cache; with an install directory, copies the verified files there for the Tiny Tapeout flow.
set -eu

if [ "$#" -lt 5 ] || [ "$#" -gt 6 ]; then
  echo "usage: rf_macro_setup.sh <base-url> <cache-dir> <lef-sha256> <lib-sha256> <gds-sha256> [install-dir]" >&2
  exit 2
fi
BASE=$1
CACHE=$2
LEF_SUM=$3
LIB_SUM=$4
GDS_SUM=$5
INSTALL=${6:-}

if command -v shasum >/dev/null 2>&1; then sha='shasum -a 256';
elif command -v sha256sum >/dev/null 2>&1; then sha='sha256sum';
else
  echo "neither shasum nor sha256sum is on PATH; refusing to fetch a macro this machine cannot verify." >&2
  exit 1
fi

fetch() {  # $1 = file name, $2 = expected SHA-256
  dest=$CACHE/$1
  if [ -f "$dest" ] && [ "$($sha "$dest" | cut -d ' ' -f 1)" = "$2" ]; then
    echo "$dest already verified."
    return 0
  fi
  mkdir -p "$CACHE"
  tmp=$(mktemp "$CACHE"/.download.XXXXXX)
  echo "fetching $BASE/$1"
  curl -fsSL -o "$tmp" "$BASE/$1"
  got=$($sha "$tmp" | cut -d ' ' -f 1)
  if [ "$got" != "$2" ]; then
    echo "SHA-256 MISMATCH for $dest -- refusing to keep it:" >&2
    echo "  expected : $2" >&2
    echo "  actual   : $got" >&2
    rm -f "$tmp"
    return 1
  fi
  echo "sha256 ok: $got"
  mv "$tmp" "$dest"
}

rc=0
fetch rf_top.lef "$LEF_SUM" || rc=1
fetch rf_top.lib "$LIB_SUM" || rc=1
fetch rf_top.gds.gz "$GDS_SUM" || rc=1
[ "$rc" -eq 0 ] || exit 1

if [ -n "$INSTALL" ]; then
  mkdir -p "$INSTALL"
  cp "$CACHE/rf_top.lef" "$CACHE/rf_top.lib" "$CACHE/rf_top.gds.gz" "$INSTALL"/
  echo "installed the verified macro files into $INSTALL"
fi

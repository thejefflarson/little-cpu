#!/bin/sh
# CoreMark's trademark terms permit the name only for an unmodified copy, so every
# route that compiles the vendored sources checks them against PINNED.sha256 first.
# ONE implementation for every route that reaches hardware: `make coremark-rom-ecp5`
# and `make coremark-rom-up5k` both run this, so the route that flashes a board
# cannot be the one whose check was forgotten.
#
# Membership before hashes: `shasum -c` cannot see a file the manifest never named,
# so an unlisted `core_portme.h` beside the vendored ones would shadow the port's own
# header and pass a hash check that never looked at it.
#
# Usage: coremark_pin_check.sh <vendor-dir>
set -e

VENDOR_DIR=${1:?usage: coremark_pin_check.sh <vendor-dir>}
if [ ! -d "$VENDOR_DIR" ]; then
  echo "error: '$VENDOR_DIR' is not a directory, so there is nothing to check." >&2
  exit 1
fi
if [ ! -f "$VENDOR_DIR/PINNED.sha256" ]; then
  echo "error: $VENDOR_DIR/PINNED.sha256 is missing, so the vendored CoreMark" >&2
  echo "sources cannot be checked against anything." >&2
  exit 1
fi

tmp=$(mktemp -d "${TMPDIR:-/tmp}/coremark-pin.XXXXXX")
trap 'rm -rf "$tmp"' EXIT

awk '!/^#/ && NF { print $NF }' "$VENDOR_DIR/PINNED.sha256" | sort > "$tmp/manifest"
(cd "$VENDOR_DIR" && for f in *; do
  [ -f "$f" ] && [ "$f" != "PINNED.sha256" ] && echo "$f"
done | sort) > "$tmp/tree"

missing=$(comm -23 "$tmp/manifest" "$tmp/tree")
unlisted=$(comm -13 "$tmp/manifest" "$tmp/tree")
if [ -n "$missing" ] || [ -n "$unlisted" ]; then
  echo "error: $VENDOR_DIR does not have exactly the files PINNED.sha256 lists --" >&2
  echo "shasum -c cannot see a file the manifest never named." >&2
  if [ -n "$missing" ]; then
    echo "named in the manifest but missing from the directory:" >&2
    printf '%s\n' "$missing" | while IFS= read -r f; do echo "  $f" >&2; done
  fi
  if [ -n "$unlisted" ]; then
    echo "in the directory but not named in the manifest:" >&2
    printf '%s\n' "$unlisted" | while IFS= read -r f; do echo "  $f" >&2; done
  fi
  exit 1
fi

if command -v shasum >/dev/null 2>&1; then
  set -- shasum -a 256 -c --strict
elif command -v sha256sum >/dev/null 2>&1; then
  set -- sha256sum -c --strict
else
  echo "error: neither shasum nor sha256sum is on PATH, so the vendored" >&2
  echo "CoreMark sources cannot be checked against $VENDOR_DIR/PINNED.sha256." >&2
  exit 1
fi

if ! (cd "$VENDOR_DIR" && "$@" PINNED.sha256) > "$tmp/out" 2>&1; then
  cat "$tmp/out" >&2
  echo >&2
  echo "*** $VENDOR_DIR no longer matches PINNED.sha256. CoreMark's own" >&2
  echo "*** trademark terms permit quoting the name only for an unmodified" >&2
  echo "*** copy of the benchmark -- re-vendor from the pinned commit" >&2
  echo "*** rather than editing a file in that directory." >&2
  exit 1
fi
cat "$tmp/out" >&2

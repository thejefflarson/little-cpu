#!/bin/bash
# Confirms the OSS CAD Suite tools on PATH are the pinned release: each tool sits in a suite
# whose VERSION file names the pinned release (the tag's date without dashes), and yosys prints the pinned version.
set -euo pipefail

if [ "$#" -lt 3 ]; then
  echo "usage: oss_cad_pin_test.sh <pinned-tag> <pinned-yosys-version> <tool>..." >&2
  exit 1
fi

TAG=$1
YOSYS_PREFIX=$2
shift 2

rc=0
for tool in "$@"; do
  resolved=$(command -v "$tool") || {
    echo "error: $tool is not on PATH; run \`make cad-setup\`." >&2
    rc=1
    continue
  }
  version_file=$(dirname "$resolved")/../VERSION
  have=$(sed -n '1p' "$version_file" 2> /dev/null || true)
  if [ "$have" != "${TAG//-/}" ]; then
    echo "error: $tool resolves to $resolved, a suite that is not the pinned release:" >&2
    echo "  its VERSION : ${have:-none}" >&2
    echo "  pinned      : ${TAG//-/} ($TAG)" >&2
    rc=1
    continue
  fi
  if [ "$tool" = yosys ]; then
    said=$("$resolved" -V 2>&1 | sed -n '1p')
    case $said in
      "$YOSYS_PREFIX"*) ;;
      *)
        echo "error: $resolved prints a different yosys than the pin:" >&2
        echo "  prints : $said" >&2
        echo "  pinned : $YOSYS_PREFIX..." >&2
        rc=1
        continue
        ;;
    esac
  fi
  echo "$tool resolves to the pinned OSS CAD Suite $TAG: $resolved"
done
if [ "$rc" -ne 0 ]; then
  echo "A build made with this PATH would place, prove and time with an unpinned" >&2
  echo "toolchain. \`make cad-setup\` installs the pinned suite; mk/toolchain.mk puts" >&2
  echo "it first on PATH." >&2
fi
exit "$rc"

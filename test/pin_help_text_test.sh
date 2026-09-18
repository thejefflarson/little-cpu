#!/bin/bash
# Checks the soc-timing recipe's help text against check-sources' own exit contract.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=${1:-$(cd "$HERE/.." && pwd)}

MAKEFILE="$REPO/Makefile"
SOC_PIN="$REPO/soc/soc_pin.py"

for f in "$MAKEFILE" "$SOC_PIN"; do
  if [ ! -f "$f" ] || [ ! -r "$f" ]; then
    echo "error: '$f' does not exist or is not readable." >&2
    exit 1
  fi
done

if ! contract=$(grep '(check-sources):' "$SOC_PIN"); then
  echo "error: soc_pin.py's usage block no longer states check-sources' exit contract." >&2
  exit 1
fi
if ! grep -qi 'warns' <<< "$contract" || ! grep -qi 'does not fail' <<< "$contract"; then
  echo "error: soc_pin.py's own exit contract no longer says a mismatch warns and does" >&2
  echo "not fail, so there is nothing left to check the Makefile's help text against:" >&2
  echo "  $contract" >&2
  exit 1
fi

# A real digest, computed for one file and checked against a second, so it is provably stale.
tmp=$(mktemp -d "${TMPDIR:-/tmp}/pin-help-text.XXXXXX")
trap 'rm -rf "$tmp"' EXIT
printf 'module a; endmodule\n' > "$tmp/a.v"
digest=$(python3 "$SOC_PIN" digest "$tmp/a.v")
cat > "$tmp/pin.json" <<JSON
{
  "sources_digest": "$digest",
  "seed": 1,
  "measured_mhz": 12.5,
  "min_mhz": 12.0,
  "toolchain": "probe"
}
JSON
printf 'module a; /* edited, so the digest above is now stale */ endmodule\n' > "$tmp/a.v"

set +e
out=$(python3 "$SOC_PIN" check-sources "$tmp/pin.json" "$tmp/a.v" 2>&1)
rc=$?
set -e
if [ "$rc" -ne 0 ]; then
  echo "error: soc_pin.py check-sources exited $rc on a digest mismatch, but its own" >&2
  echo "usage block promises 0:" >&2
  echo "  $contract" >&2
  printf '%s\n' "$out" | sed -e 's|^|  |' >&2
  exit 1
fi

# Every consecutive `@echo` line starting at the paragraph's anchor, not a fixed count.
help=$(awk '
  /With no SOC_SEED override this places at the PINNED seed/ { found = 1 }
  found {
    if ($0 !~ /^\t@echo/) exit
    print
  }
' "$MAKEFILE")
if [ -z "$help" ]; then
  echo "error: the soc-timing recipe's pin-staleness help text is gone from '$MAKEFILE'." >&2
  exit 1
fi

if ! grep -qi 'warn' <<< "$help" || ! grep -qi 'not fail' <<< "$help"; then
  echo "error: the Makefile's help text no longer says a digest mismatch warns and does" >&2
  echo "not fail -- check-sources' own usage block says it does:" >&2
  echo "  $contract" >&2
  printf '%s\n' "$help" | sed -e 's|^|  |' >&2
  exit 1
fi

echo "the Makefile's pin-staleness help text agrees with check-sources' warn-not-fail contract."

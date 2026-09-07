#!/bin/bash
# Asserts that `SB_LUT4` appears only at a fixed, reviewed list of sites.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=${1:-$(cd "$HERE/.." && pwd)}

if [ ! -d "$REPO" ]; then
  echo "error: '$REPO' is not a directory, so there is nothing to scan." >&2
  exit 1
fi

# One path per line -- a file, or a directory ending in `/` -- with the reason it carries
# the string written above it.
allow_paths() {
  cat <<'PATHS'
# The measurements-and-ratchets section's own warning against this unit, and
# the specific measured cases it warns about.
CLAUDE.md

# The one sanctioned functional use: soc/compare/placed_vs_synth.py reads
# `SB_LUT4` out of a core's standalone synthesis log and compares it, as a
# ratio and not a difference, against that SAME core's own placed ICESTORM_LC
# -- a liveness check that the datapath survived placement, not a cross-tree
# area budget. soc/compare/dhry_fit.py and soc/compare/coremark_fit.py's
# comments each point at that same log shape.
soc/compare/placed_vs_synth.py
soc/compare/dhry_fit.py
soc/compare/coremark_fit.py

# A comment about how a generated cell gets NAMED after flattening, not a count.
soc/depth/path_stages.py

# The probes that force placed_vs_synth.py and the netlist-digest structural
# diff red plant SB_LUT4 fixture lines and JSON cell types. A probe that
# cannot name what it is planting is not a probe.
test/probe_gates.sh

# This file: the paragraph above and this list. The check has to be able to
# say what it is looking for.
test/lut4_site_test.sh

# Dated decision records and dated proposals. Several measured SB_LUT4 against
# ICESTORM_LC on the tree of the day; rewriting them would make the history
# agree with a later measurement, which is vandalism rather than a sweep.
docs/adr/
docs/ideas/
PATHS
}

tmp=$(mktemp -d "${TMPDIR:-/tmp}/littlecpu-lut4site.XXXXXX") || {
  echo "error: could not create a temporary directory under ${TMPDIR:-/tmp}." >&2
  exit 1
}
trap 'rm -rf "$tmp"' EXIT

allow_paths | sed -e 's/#.*//' -e 's/[[:space:]]*$//' -e '/^$/d' > "$tmp/allow"
if [ ! -s "$tmp/allow" ]; then
  echo "error: the allow-list is empty, so this check would scan for a string" >&2
  echo "nothing is permitted to carry and report green over every hit anyway." >&2
  exit 1
fi

# Tracked files only: what this guards against is the string arriving in a commit, and a
# checkout also carries build artifacts and whole worktrees under `.claude/`.
if ! git -C "$REPO" ls-files -z > "$tmp/files" 2>/dev/null || [ ! -s "$tmp/files" ]; then
  echo "error: cannot enumerate any tracked files under $REPO. This check reads" >&2
  echo "git's index; a tree git cannot list is a scan of nothing reporting green." >&2
  exit 1
fi

match_entry() {  # $1 = path
  local path=$1 entry
  while IFS= read -r entry; do
    case "$entry" in
      */) case "$path" in "$entry"*) printf '%s' "$entry"; return 0 ;; esac ;;
      *)  if [ "$path" = "$entry" ]; then printf '%s' "$entry"; return 0; fi ;;
    esac
  done < "$tmp/allow"
  return 0
}

hits=$( (cd "$REPO" && xargs -0 grep -nI -e 'SB_LUT4' -- /dev/null < "$tmp/files") || true)

: > "$tmp/unexpected"
: > "$tmp/covered"

while IFS= read -r hit; do
  [ -n "$hit" ] || continue
  path=${hit%%:*}
  entry=$(match_entry "$path")
  if [ -n "$entry" ]; then
    printf '%s\n' "$entry" >> "$tmp/covered"
  else
    printf '%s\n' "$hit" >> "$tmp/unexpected"
  fi
done <<< "$hits"

rc=0

if [ -s "$tmp/unexpected" ]; then
  rc=1
  echo "error: 'SB_LUT4' appears where nothing on the allow-list covers it:" >&2
  sed -e 's|^|  |' "$tmp/unexpected" >&2
  echo >&2
  echo "SB_LUT4 is yosys's PRE-PLACE LUT count, not nextpnr's packed" >&2
  echo "ICESTORM_LC -- counting it instead of the packed figure has given two" >&2
  echo "planning estimates that were wrong in opposite directions here before." >&2
  echo "If this site derives a ratchet, a budget, or any other cross-tree area" >&2
  echo "decision from it, read ICESTORM_LC instead. If it is the one sanctioned" >&2
  echo "use -- a same-design placed/pre-place sanity ratio, the shape" >&2
  echo "soc/compare/placed_vs_synth.py already has -- or a comment that only" >&2
  echo "warns against the unit, add its path to test/lut4_site_test.sh's" >&2
  echo "allow-list with the reason written above it." >&2
fi

while IFS= read -r entry; do
  if ! grep -qxF -- "$entry" "$tmp/covered"; then
    rc=1
    echo >&2
    echo "error: the allow-list exempts $entry, and 'SB_LUT4' does not appear" >&2
    echo "there any more. Delete the entry -- or if that use moved, move the" >&2
    echo "entry with it. An exemption kept past its reason is how the next one" >&2
    echo "gets waved through, and it is why this comparison runs both ways." >&2
  fi
done < "$tmp/allow"

if [ "$rc" -ne 0 ]; then
  exit 1
fi

entries=$(wc -l < "$tmp/allow" | tr -d ' ')
echo "'SB_LUT4' confined to its $entries allowed sites"

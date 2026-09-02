#!/bin/bash
# Asserts that `SB_LUT4` appears only at a fixed, reviewed list of sites.
#
# Usage: lut4_site_test.sh [repo-root]     # defaults to this script's parent
#
# WHY THIS EXISTS. `SB_LUT4` is yosys's pre-place LUT count, and it is the
# wrong unit for an area budget or a ratchet on this fabric: a flip-flop that
# cannot share a cell with the LUT feeding it takes a whole packed cell by
# itself, and counting `SB_LUT4` instead of nextpnr's packed `ICESTORM_LC` gave
# two planning estimates that were wrong in opposite directions. Every place in
# this tree that reads `SB_LUT4` today is either a comment explaining that
# pitfall or `soc/compare/placed_vs_synth.py`'s one sanctioned use -- a sanity
# ratio between a design's own placed and pre-place counts, not a cross-tree
# area decision. Nothing here can tell a THIRD kind of use -- a new script that
# quietly treats `SB_LUT4` as a budget -- apart from those two by reading the
# surrounding text; that is a fact about what a number is used FOR, not a
# pattern grep can see. What this script mechanises instead is the
# reintroduction path: `SB_LUT4` arriving at a site nobody on this list
# reviewed. A new site is not wrong by construction, but it needs a reviewed
# line here saying which of the two uses it is, the same way this repo already
# insists a `.gitignore` rule or a retired term earn its exemption rather than
# accumulate silently.
#
# THE ALLOW-LIST IS PATHS, THE SAME SHAPE test/retired_term_test.sh USES for
# ITS table, because the question is the same one: does this FILE already carry
# a reviewed reason for the string, not how many times.
#
# Hermetic: git and grep only. No toolchain, no simulator, no yosys, so this
# runs inside `make test` anywhere.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=${1:-$(cd "$HERE/.." && pwd)}

if [ ! -d "$REPO" ]; then
  echo "error: '$REPO' is not a directory, so there is nothing to scan." >&2
  exit 1
fi

# One path per line -- a file, or a directory ending in `/` -- with the reason
# it carries the string written above it.
allow_paths() {
  cat <<'PATHS'
# The measurements-and-ratchets section's own warning against this unit, and
# the specific measured cases it warns about.
CLAUDE.md

# The tripwire this script exists to grade -- naming the unit it warns against
# is the whole content of the warning.
Makefile

# Comments about ABC's LUT-packing behaviour on a specific mux, not a count
# read back and compared anywhere.
rtl/littlesoc.v
rtl/uart.v

# soc/baseline_summary.py's CELL_UNIT table names `SB_LUT4` only to say a
# report's own `lc` column is never it; soc/baseline_sweep.sh's comment says
# the same about the count that sweep records.
soc/baseline_summary.py
soc/baseline_sweep.sh

# The one sanctioned functional use: soc/compare/placed_vs_synth.py reads
# `SB_LUT4` out of a core's standalone synthesis log and compares it, as a
# ratio and not a difference, against that SAME core's own placed ICESTORM_LC
# -- a liveness check that the datapath survived placement, not a cross-tree
# area budget. soc/compare/dhry_fit.py's comment points at that same log shape.
soc/compare/placed_vs_synth.py
soc/compare/dhry_fit.py

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

# Tracked files only, git's own enumeration -- what this guards against is the
# string arriving in a commit, and a checkout carries build artifacts and, in
# the primary checkout here, whole worktrees of the repo under `.claude/`.
if ! git -C "$REPO" ls-files -z > "$tmp/files" 2>/dev/null || [ ! -s "$tmp/files" ]; then
  echo "error: cannot enumerate any tracked files under $REPO. This check reads" >&2
  echo "git's index; a tree git cannot list is a scan of nothing reporting green." >&2
  exit 1
fi

# Which allow-list entry covers a path, or nothing. A directory entry matches
# on the trailing `/` it ends with, so `docs/adr/` cannot quietly cover a
# `docs/adrenaline.md` that nobody meant to exempt.
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

# `/dev/null` as a first argument so grep always prefixes the filename, even
# when xargs hands it a single file. No match at all leaves grep with status 1
# and xargs with 123, which is not an error here.
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

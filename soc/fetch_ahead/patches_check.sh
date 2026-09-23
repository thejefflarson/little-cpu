#!/usr/bin/env bash
# Every tracked spike patch under soc/fetch_ahead/ must still apply to the tree it is kept
# against, and skid.patch's own fetcher bench must pass on the staged result. Only the files
# a patch names are staged, so the check costs kilobytes rather than the tree.
# Usage: patches_check.sh [patch-dir]
set -euo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
patchdir=${1:-$root/soc/fetch_ahead}

tmp=$(mktemp -d "${TMPDIR:-/tmp}/patches-check.XXXXXX")
trap 'rm -rf "$tmp"' EXIT

stage() {  # <patch> <dir>: the tree's copy of every file the patch names, plus rtl/
  mkdir -p "$2"
  cp -R "$root/rtl" "$2/rtl"
  sed -n 's,^+++ b/,,p' "$1" | while read -r f; do
    [ -e "$root/$f" ] || continue
    mkdir -p "$2/$(dirname "$f")"
    cp "$root/$f" "$2/$f"
  done
}

for patchfile in "$patchdir"/*.patch; do
  name=$(basename "$patchfile")
  stage "$patchfile" "$tmp/$name"
  patch --fuzz=0 -p1 -d "$tmp/$name" < "$patchfile"
  echo "$name applies to this tree"
done

skid=$tmp/skid.patch
if [ -d "$skid" ]; then
  srcs=$(sed -n 's/^UNIT_BENCH_SRC_fetcher_tb := //p' "$skid/Makefile")
  (cd "$skid" && iverilog -I./rtl/ -g2012 -o fetcher.vvp $srcs test/fetcher_tb.v)
  vvp "$skid/fetcher.vvp" 2>&1 | tee "$skid/bench.log"
  grep -q '^PASSED: fetcher.v' "$skid/bench.log"
  echo "skid.patch's fetcher bench passes on the staged tree"
fi

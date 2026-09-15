#!/usr/bin/env bash
# Stages a spike patch (prototype.patch unless named) onto a copy of the tree, for placing,
# timing or simulating it without touching rtl/ in the checkout. Usage: apply.sh <dir> [patch]
set -euo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
out=${1:?usage: soc/fetch_ahead/apply.sh <output-dir> [patch]}
patchfile=${2:-$root/soc/fetch_ahead/prototype.patch}
marker=.fetch-ahead-tree

if [ -e "$out" ] && [ ! -f "$out/$marker" ]; then
  echo "refusing to replace $out: it is not a tree this script made" >&2
  exit 2
fi
rm -rf "$out"
mkdir -p "$out"
touch "$out/$marker"
for d in rtl formal test soc mk nano; do
  cp -R "$root/$d" "$out/$d"
done
cp "$root/Makefile" "$out/Makefile"
rm -rf "$out/formal/riscv-formal"
ln -sfn "$root/formal/riscv-formal" "$out/formal/riscv-formal"

patch --fuzz=0 -p1 -d "$out" < "$patchfile"
echo "$out"

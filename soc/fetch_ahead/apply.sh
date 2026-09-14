#!/usr/bin/env bash
# Stages prototype.patch onto a copy of the tree, for placing/timing/simulating the
# fetch-ahead decoder without touching rtl/ in the checkout. Usage: apply.sh <output-dir>
set -euo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
out=${1:?usage: soc/fetch_ahead/apply.sh <output-dir>}

rm -rf "$out"
mkdir -p "$out"
for d in rtl formal test soc mk nano; do
  cp -R "$root/$d" "$out/$d"
done
cp "$root/Makefile" "$out/Makefile"
rm -rf "$out/formal/riscv-formal"
ln -sfn "$root/formal/riscv-formal" "$out/formal/riscv-formal"

patch -p1 -d "$out" < "$root/soc/fetch_ahead/prototype.patch"
echo "$out"

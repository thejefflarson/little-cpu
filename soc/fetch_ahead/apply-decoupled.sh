#!/usr/bin/env bash
# Stages prototype-decoupled.patch: a fetch address reading only registers (rtl/fetchqueue.v).
set -euo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
out=${1:?usage: soc/fetch_ahead/apply-decoupled.sh <output-dir>}

rm -rf "$out"
mkdir -p "$out"
for d in rtl formal test soc mk nano; do
  cp -R "$root/$d" "$out/$d"
done
cp "$root/Makefile" "$out/Makefile"
rm -rf "$out/formal/riscv-formal"
ln -sfn "$root/formal/riscv-formal" "$out/formal/riscv-formal"

patch -p1 -d "$out" < "$root/soc/fetch_ahead/prototype-decoupled.patch"
echo "$out"

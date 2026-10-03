#!/bin/bash
# Runs nano_rf_model_tb.v against nano.v's register-file model, which must pass, and against three mutants of it, each of which must fail the check written for it.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WORKDIR="$HERE/nano-rf-model-test"
TB="$HERE/nano_rf_model_tb.v"

if ! command -v iverilog >/dev/null 2>&1; then
  echo "error: iverilog is not on PATH." >&2
  exit 2
fi

rm -rf "$WORKDIR"
mkdir -p "$WORKDIR"

run() {  # $1 = nano.v to read, $2 = stem
  iverilog -g2012 -o "$WORKDIR/$2.vvp" "$1" "$TB"
  vvp "$WORKDIR/$2.vvp"
}

echo "control: the shipping model"
out=$(run "$REPO/nano/nano.v" shipping)
echo "$out"
if ! grep -q '^PASS$' <<< "$out"; then
  echo "*** the shipping model fails its own bench, so a mutant failing the same way proves nothing." >&2
  exit 1
fi

expect_red() {  # $1 = stem, $2 = what the mutant does, $3 = sed program, $4 = the check that must name the failure
  sed "$3" "$REPO/nano/nano.v" > "$WORKDIR/$1.v"
  if cmp -s "$REPO/nano/nano.v" "$WORKDIR/$1.v"; then
    echo "error: nano.v no longer spells the line the $2 mutant rewrites. Re-anchor it --" \
         "left alone this runs the shipping model twice and proves nothing." >&2
    exit 2
  fi
  echo
  echo "mutant: $2"
  out=$(run "$WORKDIR/$1.v" "$1")
  echo "$out"
  if ! grep -q "^FAIL $4" <<< "$out"; then
    echo "*** the $2 mutant did not fail \"$4\"." >&2
    exit 1
  fi
}

same_edge="a read of the word written on the same edge returns the old word inverted"
expect_red old_word "same-edge read returns the old word" \
  's/storage\[ra_addr\] ^ {32{w_ena && ra_addr == w_addr}}/storage[ra_addr]/' "$same_edge"
expect_red write_through "same-edge read returns the written word" \
  's/ra_data <= storage\[ra_addr\] ^ {32{w_ena && ra_addr == w_addr}}/ra_data <= (w_ena \&\& ra_addr == w_addr) ? w_data : storage[ra_addr]/' \
  "$same_edge"
expect_red port_b "port b reads port a's address" \
  's/rb_data <= storage\[rb_addr\]/rb_data <= storage[ra_addr]/' \
  "a read returns its word one edge after its address, port b"

echo
echo "The register-file model passes its bench and each mutant fails the check written for it."

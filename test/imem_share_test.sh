#!/bin/bash
# Asserts that rtl/imemory.v's fetch windows read ONE storage, on the mapped
# netlist of both parts, and forces that assertion red.
#
# Usage: imem_share_test.sh      # every case; exit 0 only if all of them hold
#
# WHY THIS EXISTS. The module asks each bank for one read port per window.
# Neither part has a primitive with two read ports, so yosys answers by
# replicating the bank and driving every copy from the same write -- and that is
# the whole basis for calling the text shared rather than mirrored. THE CLAIM IS
# ABOUT THE MAPPED NETLIST AND NOTHING IN THE SOURCE: at RTL there is one array
# and every window reads it by construction, so no simulation of rtl/imemory.v
# can confirm the replication or fail on its absence. A comment saying "yosys
# replicates this" is not a check, and the mapping is a property of a toolchain
# nothing here pins.
#
# BOTH PARTS, because the answer is the toolchain's rather than the design's and
# the two primitives differ: ice40's block RAM is one read port and one write
# port, ECP5's is true dual-port and yosys writes through one side of it. The
# counts are what the ECP5 budget for the dual SoC is built on, and they are the
# reason the dual configuration can never be built on the up5k -- 32 block RAMs
# against that part's 30.
#
# THE RED DIRECTIONS ARE HERE, not argued: a mutant whose second window reads a
# pair of banks the write does not reach must be reported, and both of
# test/rom_replication.py's other refusals are forced against a netlist that
# passes. Needs yosys and python3; `make window-test` already requires yosys, so
# this does not narrow where `make test` runs.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/.." && pwd)

for tool in yosys python3; do
  if ! command -v "$tool" >/dev/null 2>&1; then
    echo "error: $tool is not on PATH, so the ROM's mapping cannot be measured." >&2
    echo "Skipping would report a green run for a claim nothing checked, which" >&2
    echo "is what this file exists to prevent." >&2
    exit 1
  fi
done

tmp=$(mktemp -d "${TMPDIR:-/tmp}/littlecpu-imem-share.XXXXXX")
test -n "$tmp" -a -d "$tmp"
trap 'rm -rf "$tmp"' EXIT

cases=0
failed=0

# Pinned as a literal, for the reason test/PROBES_EXPECTED gives: a case
# deleted, or one stopped being reached by an early return, would otherwise cut
# this file's coverage while it went on printing a green summary.
CASES_EXPECTED=9

ok()   { cases=$((cases + 1)); printf 'ok   %s\n' "$1"; }
bad()  { cases=$((cases + 1)); failed=$((failed + 1)); printf 'FAIL %s\n     -> %s\n' "$1" "$2" >&2; }

# map <source> <part> <NHARTS> <tag> -> $tmp/<tag>.json and $tmp/<tag>.log
map() {
  local src=$1 part=$2 harts=$3 tag=$4 script
  case $part in
    ice40) script="synth_ice40 -device u -top imemory" ;;
    ecp5)  script="synth_ecp5 -top imemory" ;;
  esac
  if ! yosys -p "read_verilog -sv $src; chparam -set NHARTS $harts imemory; $script; write_json $tmp/$tag.json" \
       > "$tmp/$tag.log" 2>&1; then
    tail -20 "$tmp/$tag.log" >&2
    echo "error: yosys could not map $src for $part at NHARTS=$harts." >&2
    exit 1
  fi
}

# census <tag> <cell> <expected> <label>
census() {
  local out
  if out=$(python3 "$REPO/soc/cell_census.py" "$tmp/$1.log" "$2" "$3" \
             "rtl/imemory.v's banks have stopped mapping the way the dual-hart budget was built on" 2>&1); then
    ok "$4: $out"
  else
    bad "$4" "$out"
  fi
}

# shares <tag> <cell> <copies> <groups> <label>
shares() {
  local out
  if out=$(python3 "$REPO/test/rom_replication.py" "$tmp/$1.json" \
             --cell "$2" --copies "$3" --groups "$4" 2>&1); then
    ok "$5: $out"
  else
    bad "$5" "$out"
  fi
}

# refuses <tag> <cell> <copies> <groups> <fragment> <label>
refuses() {
  local out rc
  set +e
  out=$(python3 "$REPO/test/rom_replication.py" "$tmp/$1.json" \
          --cell "$2" --copies "$3" --groups "$4" 2>&1)
  rc=$?
  set -e
  if [ "$rc" -eq 0 ]; then
    bad "$6" "it reported the netlist as shared; the refusal never fired"
  elif ! grep -qF -- "$5" <<< "$out"; then
    bad "$6" "it failed without \"$5\": $out"
  else
    ok "$6 refused"
  fi
}

echo "== the shipping window count maps as it always did"
map "$REPO/rtl/imemory.v" ice40 1 one_ice40
map "$REPO/rtl/imemory.v" ecp5  1 one_ecp5
census one_ice40 SB_RAM40_4K 16 "ice40, one window"
census one_ecp5  DP16KD       4 "ECP5, one window"

echo
echo "== two windows are two copies of one storage"
map "$REPO/rtl/imemory.v" ice40 2 two_ice40
map "$REPO/rtl/imemory.v" ecp5  2 two_ecp5
census two_ice40 SB_RAM40_4K 32 "ice40, two windows"
census two_ecp5  DP16KD       8 "ECP5, two windows"
shares two_ice40 SB_RAM40_4K 2 16 "ice40, the copies share one write"
shares two_ecp5  DP16KD      2  4 "ECP5, the copies share one write"

# The mutation the brief names: a write that lands in only one hart's copy. The
# second window reads a private pair of banks, written on a strobe of its own, so
# the two copies are two storages that can hold different words at one address.
echo
echo "== a copy with a write of its own is reported"
mutant=$tmp/imemory_private.v
sed -e 's/rom_even\[w_even_index\]/rom_even_private[w_even_index]/' \
    -e 's/rom_odd\[w_odd_index\]/rom_odd_private[w_odd_index]/' \
    -e 's|^  logic \[31:0\] rom_odd \[0:BANK_WORDS-1\];|&\n  logic [31:0] rom_even_private[0:BANK_WORDS-1];\n  logic [31:0] rom_odd_private [0:BANK_WORDS-1];\n  always_ff @(posedge clk) begin\n    if (text_write_even \&\& !mem_wdata[0]) rom_even_private[data_index] <= mem_wdata;\n    if (text_write_odd  \&\& !mem_wdata[0]) rom_odd_private[data_index]  <= mem_wdata;\n  end|' \
    "$REPO/rtl/imemory.v" > "$mutant"
# EACH EDIT IS CHECKED, not just the name it introduces: the read substitutions
# and the declaration insertion are three independent patterns, and one of them
# silently missing leaves a file that either does not elaborate or is the
# shipping design under another name. Either way the red direction below would be
# reporting on something nobody built.
for token in 'rom_even_private\[0:BANK_WORDS-1\]' 'rom_odd_private \[0:BANK_WORDS-1\]' \
             'rom_even_private\[w_even_index\]' 'rom_odd_private\[w_odd_index\]'; do
  if ! grep -q "$token" "$mutant"; then
    echo "error: the private-copy mutation did not apply -- no \`$token\` in it." >&2
    echo "rtl/imemory.v was respelled; teach this script the new spelling rather" >&2
    echo "than letting the red direction synthesise the shipping design." >&2
    exit 1
  fi
done
map "$mutant" ice40 2 mutant_ice40
refuses mutant_ice40 SB_RAM40_4K 2 16 "write of its own" \
  "ice40, the second window's private banks"

# The checker's own two other exits, forced against a netlist that passes, so
# neither can rot into a verdict it can no longer reach.
echo
echo "== the checker's other refusals"
refuses two_ice40 SB_RAM40_4K 2 8 "against the 16 declared" \
  "a count that does not match the declaration"
refuses two_ice40 SB_RAM40_4K 1 32 "shares a write with nothing" \
  "a single-copy run, which would pass without looking"

echo
if [ "$failed" -ne 0 ]; then
  echo "$failed of $cases mapping checks did not behave as required." >&2
  exit 1
fi
if [ "$cases" -ne "$CASES_EXPECTED" ]; then
  echo "error: ran $cases mapping checks, expected $CASES_EXPECTED." >&2
  echo "A case was added or removed. Update CASES_EXPECTED in the same commit;" >&2
  echo "the literal is what stops this file quietly covering less than it did." >&2
  exit 1
fi
echo "$cases mapping checks: the windows read one storage on both parts."

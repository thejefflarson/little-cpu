#!/bin/sh
# The control the netlist digest is void without: place, then ask the placer.
#
# The digest claims that a change whose canonical netlist is unchanged has an
# unmoved placement distribution. Two things have to be true for that, neither of
# them derivable, and both are re-taken here on the tree being asked about rather
# than quoted from the day the gate was written.
#
#   A. Placement is a function of the netlist. One netlist, placed twice at one
#      seed, must give a byte-identical bitstream. If it does not, no digest of
#      any form predicts anything.
#   B. What the canonicalisation forgives, the placer does not see. A copy of
#      this tree is mutated with the three edit classes the digest is built to
#      forgive -- a comment, a blank line, and a dead tie-off yosys optimises
#      away -- and its SHIPPING netlist must place to the same bitstream as this
#      tree's while its canonical digest stays equal.
#
# B IS THE DIRECT QUESTION, AND IT IS NOT THE ONE THAT WAS FIRST ASKED. The
# obvious control is to place the purged and unpurged forms of one tree and
# require the same bitstream. That was run, and it FAILS: the purged netlist
# places to a different bitstream at the same seed, same design, same tools.
# `opt_clean -purge` moves the placement. What follows from that is only that the
# canonical JSON is a classifier and must never be handed to nextpnr -- not that
# the gate narrows, because nobody places the canonical form. What gets placed is
# the shipping netlist of this tree and of the mutant, and those are what B
# compares.
#
# B ALSO CHECKS THAT THE MUTATION LEFT A TRACE. A mutant whose shipping netlist
# is byte-identical to the base's proves nothing: it would pass this control
# while demonstrating that nothing was injected. That is the vacuity check, and
# it is also what catches the injection site going stale under an RTL change.
#
# Three placements, about three minutes. Nothing is measured here -- no
# frequency, no cell count -- so a seed is picked once and never varied; what is
# compared is bytes.
#
# Driven by the Makefile, which owns the part table:
#   NETLIST_SYNTH    the shipping synth script, minus its `-json <file>`
#   NETLIST_PNR      the place-and-route command, minus its --json and its output
#   NETLIST_PNR_OUT  the flag that names the placed output (`--asc` on ice40)
#   NETLIST_OUT      where the netlists, bitstreams and logs are kept to read
set -eu

cd "$(dirname "$0")/.."

# Spelled out rather than left to `${VAR:?}`, whose exit status is the shell's
# to choose: this one is probed, and a status that moves with /bin/sh is a probe
# that passes on one machine and not the next.
for required in "NETLIST_SYNTH=${NETLIST_SYNTH:-}" "NETLIST_PNR=${NETLIST_PNR:-}"; do
  case $required in
    *=) echo "*** make netlist-determinism: ${required%=} is not set, so there" >&2
        echo "*** is nothing to synthesise or place. The Makefile's part table" >&2
        echo "*** sets it." >&2
        exit 2 ;;
  esac
done
pnr_out=${NETLIST_PNR_OUT:---asc}
out=${NETLIST_OUT:-netlist.out}
case $out in
  ""|/|*..*) echo "*** soc/netlist_determinism.sh: NETLIST_OUT='$out' is not a" >&2
             echo "*** directory this may delete and rebuild." >&2; exit 2 ;;
esac

# The file the mutation goes into and the signal the dead wire reads. Both are
# core RTL rather than a part's top, so this control does not need a per-part
# entry -- and the vacuity check above is what reports either of them going
# stale, in place of a comment nobody re-reads.
MUTANT_FILE=rtl/decoder.v
MUTANT_SIGNAL=reg_rs1

fail() {
  echo "*** make netlist-determinism: $1" >&2
  shift
  for line in "$@"; do echo "*** $line" >&2; done
  echo "*** THE DIGEST GATE IS VOID until this passes. Spend the seeds." >&2
  exit 1
}

need() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "*** make netlist-determinism: no $1 on PATH, so nothing was placed" >&2
    echo "*** and nothing was controlled. That is not a pass." >&2
    exit 2
  }
}
need yosys
# The placer's name comes out of the part table with the rest of its command
# line, so this reaches nextpnr-ecp5 as readily as nextpnr-ice40.
need "${NETLIST_PNR%% *}"

rm -rf "$out"
mkdir -p "$out"

synth() {  # <tree> <shipping json> <canonical json> <log>
  ( cd "$1" && yosys -p "$NETLIST_SYNTH -json $2" ) > "$4" 2>&1 || {
    tail -40 "$4" >&2
    fail "yosys could not synthesise $1 (log: $4)."
  }
  ( cd "$1" && yosys -p "$NETLIST_SYNTH; opt_clean -purge; write_json $3" ) >> "$4" 2>&1 || {
    tail -40 "$4" >&2
    fail "yosys could not write the canonical form of $1 (log: $4)."
  }
}

# nextpnr's exit status is not the signal here for the reason the Makefile's
# `soc.asc` records -- it grades its own default clock with its own estimator --
# and this control grades neither. What it needs is the bitstream, so an absent
# one is the failure.
place() {  # <json> <placed output> <log>
  $NETLIST_PNR --json "$1" "$pnr_out" "$2" > "$3" 2>&1 || true
  test -s "$2" || {
    tail -30 "$3" >&2
    fail "nextpnr wrote no bitstream for $1, so nothing was placed (log: $3)."
  }
}

echo "== A. placement is a function of the netlist"
synth . "$PWD/$out/this.json" "$PWD/$out/this.canon.json" "$out/this.synth.log"
place "$out/this.json" "$out/this.1.asc" "$out/this.1.pnr.log"
place "$out/this.json" "$out/this.2.asc" "$out/this.2.pnr.log"
if ! cmp -s "$out/this.1.asc" "$out/this.2.asc"; then
  fail "one netlist placed twice at one seed gave two different bitstreams." \
       "Placement is not a function of the netlist on this toolchain, so no" \
       "digest of any form predicts it. Read $out/this.1.pnr.log and" \
       "$out/this.2.pnr.log."
fi
echo "   two placements of one netlist agree, byte for byte over $(wc -c < "$out/this.1.asc" | tr -d ' ') bytes"

echo "== B. what the canonicalisation forgives, the placer does not see"
mutant=$out/mutant
mkdir -p "$mutant/soc"
cp -R rtl "$mutant/rtl"
for f in soc/*.hex soc/*.pcf; do
  if [ -e "$f" ]; then cp "$f" "$mutant/soc/"; fi
done

# The three classes in one mutant, because one placement answers for all three
# and a red one is re-read here rather than bisected in the report.
target=$mutant/$MUTANT_FILE
test -f "$target" || fail "$MUTANT_FILE is not in this tree, so the mutant could not be built."
python3 - "$target" "$MUTANT_SIGNAL" <<'PY' || fail "the mutant could not be built; read soc/netlist_determinism.sh's injection."
import sys
path, signal = sys.argv[1], sys.argv[2]
text = open(path).read()
lines = text.splitlines(True)
if len(lines) < 2:
    sys.exit(f"{path}: too short to insert into")
lines.insert(1, "// A comment that says nothing, so every line under it moves.\n")
lines.insert(2, "\n")
text = "".join(lines)
marker = "\nendmodule"
if marker not in text or signal not in text:
    sys.exit(f"{path}: no trailing `endmodule` or no `{signal}` to read")
cut = text.rindex(marker)
dead = (f"  logic [31:0] netlist_control_dead;\n"
        f"  assign netlist_control_dead = {{{signal}[15:0], {signal}[31:16]}};\n")
open(path, "w").write(text[:cut + 1] + dead + text[cut + 1:])
PY

synth "$mutant" "$PWD/$out/mutant.json" "$PWD/$out/mutant.canon.json" "$out/mutant.synth.log"

if cmp -s "$out/this.json" "$out/mutant.json"; then
  fail "the mutant's shipping netlist is byte-identical to this tree's, so" \
       "nothing was injected and this control demonstrates nothing." \
       "The injection site in soc/netlist_determinism.sh has gone stale:" \
       "$MUTANT_FILE no longer carries what it reaches for."
fi
echo "   the mutant's shipping netlist differs from this tree's, as it must"

if ! python3 soc/netlist_digest.py compare "$out/this.canon.json" "$out/mutant.canon.json" \
     --base-label "this tree" --new-label "this tree, mutated" > "$out/mutant.digest.log" 2>&1; then
  cat "$out/mutant.digest.log" >&2
  fail "the canonical form no longer forgives a comment, a blank line and a" \
       "dead tie-off. The digest would now report 'different' for the change" \
       "class it exists for, which makes it a grader with one verdict."
fi
echo "   the mutant's canonical digest equals this tree's, as it must"

place "$out/mutant.json" "$out/mutant.asc" "$out/mutant.pnr.log"
if ! cmp -s "$out/this.1.asc" "$out/mutant.asc"; then
  fail "two trees the digest calls equal placed to different bitstreams." \
       "The canonicalisation forgives something the placer sees, so" \
       "digest-equal no longer implies an unmoved placement." \
       "Read $out/this.1.pnr.log and $out/mutant.pnr.log."
fi

echo "   the mutant places to this tree's bitstream, byte for byte"
echo
echo "netlist-determinism: PASS. Placement is a function of the netlist here, and"
echo "the three edit classes the digest forgives do not move it. What is under"
echo "$out/ is a classifier and not a design: the canonical netlist there is not"
echo "what ships, and nothing may hand it to nextpnr."

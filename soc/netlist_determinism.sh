#!/bin/sh
# The control the netlist digest is void without: place, then ask the placer.
set -eu

cd "$(dirname "$0")/.."

# Spelled out rather than left to `${VAR:?}`, whose exit status is the shell's to choose:
# this one is probed, and a status that moves with /bin/sh is a probe that passes on one
# machine and not the next.
for required in "NETLIST_SYNTH=${NETLIST_SYNTH:-}" "NETLIST_PNR=${NETLIST_PNR:-}" \
                "NETLIST_MUTANT=${NETLIST_MUTANT:-}"; do
  case $required in
    *=) echo "*** make netlist-determinism: ${required%=} is not set, so there" >&2
        echo "*** is nothing to synthesise or place. The Makefile's part table" >&2
        echo "*** sets it." >&2
        exit 2 ;;
  esac
done
pnr_out=${NETLIST_PNR_OUT:---asc}
pnr_done=${NETLIST_PNR_DONE:-Info: Program finished normally.}
out=${NETLIST_OUT:-netlist.out}
case $out in
  ""|/|*..*) echo "*** soc/netlist_determinism.sh: NETLIST_OUT='$out' is not a" >&2
             echo "*** directory this may delete and rebuild." >&2; exit 2 ;;
esac

# The file the mutation goes into and the signal the dead wire reads.
MUTANT_FILE=${NETLIST_MUTANT%% *}
MUTANT_SIGNAL=${NETLIST_MUTANT##* }
DEAD_NET=netlist_control_dead

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
# The placer's name comes out of the part table with the rest of its command line, so
# this reaches nextpnr-ecp5 as readily as nextpnr-ice40.
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

place() {  # <json> <placed output> <log>
  status=0
  $NETLIST_PNR --json "$1" "$pnr_out" "$2" > "$3" 2>&1 || status=$?
  test -s "$2" || {
    tail -30 "$3" >&2
    fail "nextpnr wrote no bitstream for $1 (exit $status), so nothing was" \
         "placed (log: $3)."
  }
  grep -qF "$pnr_done" "$3" || {
    tail -30 "$3" >&2
    fail "nextpnr never finished for $1 (exit $status): its log never says" \
         "'$pnr_done', so $2 holds whatever had been written when it stopped." \
         "A partial bitstream is not a placement, and a deterministic placer" \
         "truncated twice compares equal to itself (log: $3)."
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

target=$mutant/$MUTANT_FILE
test -f "$target" || fail "$MUTANT_FILE is not in this tree, so the mutant could not be built."
python3 - "$target" "$MUTANT_SIGNAL" "$DEAD_NET" <<'PY' || fail "the mutant could not be built; read soc/netlist_determinism.sh's injection."
import re
import sys

path, signal, dead_net = sys.argv[1], sys.argv[2], sys.argv[3]
text = open(path).read()
lines = text.splitlines(True)
if len(lines) < 2:
    sys.exit(f"{path}: too short to insert into")
lines.insert(1, "// A comment that says nothing, so every line under it moves.\n")
lines.insert(2, "\n")
text = "".join(lines)

marker = "\nendmodule"
if marker not in text:
    sys.exit(f"{path}: no trailing `endmodule` to splice the tie-off before")
cut = text.rindex(marker)

# The signal has to be in scope IN THE MODULE the assign lands in, not merely
# somewhere in the file: a same-named signal in an earlier module leaves the
# injected one implicitly declared one bit wide, the part-selects fold to a
# constant, the assign is optimised away, and the control goes on passing
# having stopped injecting the class it is here to exercise.
declarations = [m.start() for m in re.finditer(r"^module\b", text, re.M)]
if not declarations or declarations[-1] > cut:
    sys.exit(f"{path}: no module declaration before the last `endmodule`")
if signal not in text[declarations[-1]:cut]:
    sys.exit(f"{path}: `{signal}` is not in the module the tie-off splices into")

dead = (f"  logic [31:0] {dead_net};\n"
        f"  assign {dead_net} = {{{signal}[15:0], {signal}[31:16]}};\n")
open(path, "w").write(text[:cut + 1] + dead + text[cut + 1:])
PY

synth "$mutant" "$PWD/$out/mutant.json" "$PWD/$out/mutant.canon.json" "$out/mutant.synth.log"

if cmp -s "$out/this.json" "$out/mutant.json"; then
  fail "the mutant's shipping netlist is byte-identical to this tree's, so" \
       "nothing was injected and this control demonstrates nothing." \
       "The injection site in the Makefile's part table has gone stale:" \
       "$MUTANT_FILE no longer carries what it reaches for."
fi
echo "   the mutant's shipping netlist differs from this tree's, as it must"

if ! grep -q "$DEAD_NET" "$out/mutant.json"; then
  fail "the injected dead tie-off left no trace in the mapped netlist, so the" \
       "dead-net class is not exercised and only the comment class is." \
       "$MUTANT_FILE has to be the TOP module: injected into a submodule this" \
       "wire is optimised away before the netlist is written, and the two" \
       "netlists then differ in source attributes alone."
fi
if grep -q "$DEAD_NET" "$out/mutant.canon.json"; then
  fail "the dead tie-off survives \`opt_clean -purge\` into the canonical form," \
       "so the canonicalisation is no longer what forgives a dead net and the" \
       "digest below is equal for some other reason."
fi
echo "   the dead net reaches the shipping netlist and is purged from the canonical one"

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

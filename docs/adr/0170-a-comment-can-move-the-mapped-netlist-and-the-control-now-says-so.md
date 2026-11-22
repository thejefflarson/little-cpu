# ADR-0170: A comment can move the mapped netlist, and the control now says so

**Status:** Accepted · 2026-09-07 · *Amends ADR-0122. Narrows the digest's claim in the direction
already measured, and moves the digest's comment-class control onto a file where it can fail for
the reason it exists. No commitment touched, no gate on `make test` or CI.*

## Context

PR #291 (`45f0b9b`) cut every file in `rtl/` to at most 5% comment lines and rewrote what survived
-- a diff that is entirely comments and blank lines. Measured fresh (Yosys 0.68+48 `ff5817c34`,
nextpnr-ice40 0.11-1-`g62e659ed`, no `make`, every input SHA-256'd): that commit moved the SoC's
mapped netlist **6,266 -> 6,289 cells (+23 `SB_LUT4`, 0.37%)**. Synthesis is deterministic --
repeated fresh runs of both trees digest identically -- and the move bisects to `rtl/csrs.v` alone;
each of the other fifteen `rtl/` files carries the comment diff with the netlist unmoved.

ADR-0122 built exactly the control this should have failed: `soc/netlist_determinism.sh` mutates a
tree with a comment, a blank line and a dead tie-off and asserts the canonical digest and the placed
bitstream are both unmoved. It never caught this, because its comment-class case lands two lines at
the top of `rtl/littlesoc.v` -- 153 lines, and never once measured to move. **The control was correct
in shape and pointed at a file it had already proven safe.** A grader that inserts a comment where a
comment cannot move anything is not exercising the class it is named for.

## The mechanism, now demonstrated rather than inferred

A separate investigation, run in parallel with this ticket on the same toolchain: the two versions
of `rtl/csrs.v` parse to an **identical AST** (975 node lines, locations masked) and identical RTLIL
once `src` is masked. Every one of the 42 changed lines matches `^\s*//`; every non-comment line is
byte-identical in sequence. The two files' *meaning* is unarguably the same.

The carrier is yosys's own auto-generated cell-name string, which embeds a `file:line` pair (for
example `$logic_and$rtl/csrs.v:92$1`). `abc9_ops.cc` builds a topological sort over cell names
(`TopoSort<IdString, RTLIL::sort_by_id_str>`) before `write_xaiger` emits them, and
`sort_by_id_str` is `IdString::lt_by_name` -- a `memcmp` over the name bytes, comparing the embedded
line number's digits as characters rather than as a number. Everything before that pass is a pure
renaming: per-stage RTLIL dumps (`flatten`, `coarse`, `map_ram`, `map_ffram`, `map_gates`,
`map_ffs`, `map_luts`), with `src` dropped and `` `line `` directives masked, are byte-identical at
every stage, and the `xaiger` header matches exactly (`aig 18593 5126 0 9238 13467`); the named
PI/PO sets are identical, only their *order* differing (1,237 of 7,671 positions). ABC9's own
order-sensitive passes (`&dch`/`&if`) see the same gate set in a different order and pick a
different one of several functionally equivalent LUT mappings: `and=18454 ch=2653` against
`and=18612 ch=2854`, `LUT=3801` against `LUT=3831`. Decisive test: patching **only** `` `line ``
directives inside yosys's pre-`abc9_ops` intermediate dump -- no source file in play at all --
reproduces both digests on demand: #291's text told to report base's line numbers gives base's
digest (6,266); base's text told to report #291's numbers gives #291's (6,289). Text held constant,
numbers varied, the netlist follows the numbers, not the text.

**This is a property of the flow, not of `rtl/csrs.v` alone.** Putting `rtl/csrs.v` first on the
`read_verilog` command line -- which renumbers yosys's global `$autoidx` in every generated name --
produced two more, different digests (6,231 and 6,258) from the same two source trees. Renaming a
file does not reproduce it on its own: `flatten` prefixes the instance name onto every cell, so a
renamed file changes sort order only through where it happens to push `$autoidx`, not by the filename
appearing in a compared string.

**+23 cells is 0.37% of 6,266 -- inside this repo's own ±50-cell churn band**, and the logic behind
it is provably identical. This is a placement-relevant reshuffle of equivalent mappings, not a
functional regression.

## Decision

**`soc/netlist_determinism.sh`'s comment class now lands in a large, representative file
(`NETLIST_COMMENT_FILE`, `rtl/csrs.v` for up5k), separately from the dead-net class, which stays at
the top module (`NETLIST_MUTANT`, `rtl/littlesoc.v`) because a submodule's dead wire is optimised
away before the shipping netlist is written -- measured directly: injecting the same tie-off into
`rtl/csrs.v` leaves it absent from the shipping JSON. The same digest-equality and placed-bitstream
assertions cover both mutations at once, so the control fails loudly, for the observation and
regardless of the cause, if either class ever moves the digest again.

**The claim is narrower than ADR-0122 stated it, in the direction this measurement points.**
Digest-equal remains sound -- nothing here found a counterexample, and every case where the digest
stayed equal also placed identically. What is no longer true: digest-different does not imply a
semantic (RTL-meaning) change. It can mean a comment crossed a lexical-rank boundary in a large
file, with the logic unchanged and only which of several equivalent LUT mappings ABC9 picked
different. Either way the placement can move, so the operational answer is unchanged: digest
different, spend the sweep.

`soc/netlist_digest.py`'s header and CLAUDE.md's `netlist-digest` command entry both carry the
caveat and this measurement now, so a reader does not have to find this ADR to learn the digest's
equal-class is narrower than "comment-only edits are always forgiven."

## What was considered and rejected

**Reverting PR #291's comments to shrink the netlist back to 6,266.** Out of scope: the 12 MHz
margin question this movement raises is a requirements decision for its own ticket, and the logic
is unchanged either way -- there is nothing here to "fix" by editing comments.

**Naming the mechanism in the control's own comments or fail messages.** Declined. The control
should be right regardless of which pass turns out to be sensitive; encoding "`abc9_ops`'s sort
order" into the script would make it read like a claim about *why*, when what it asserts is only
*that* the digest stayed equal. The mechanism belongs here and in the digest's dated header, not in
a comment a future refactor of `abc9_ops` would leave stale.

**Combining the comment and dead-net classes in one file, as before.** Impossible for the dead-net
class without losing its own oracle: the tie-off has to live in the top module to survive synthesis
at all, and the file that demonstrates the comment class is a submodule.

## Consequences

- **`make netlist-determinism` and `make netlist-digest` are unaffected on this tree, today**: the
  two-line insertion this control performs on `rtl/csrs.v` does not move the digest further --
  whatever crossed the boundary already landed with PR #291. The control is now *capable* of failing
  for this reason where it structurally could not before; it is not expected to fail on every tree.
- **A future comment-only edit to a large file may legitimately fail `make netlist-determinism`.**
  That is the control working, not a bug in it: read the fail message, spend the sweep, and do not
  read "canonicalisation is broken" into a result this ADR predicts.
- **The toolchain-dependence already stated for `make fit`'s churn band gets sharper**: a change
  this small (0.37%) is not distinguishable from placement noise by area alone, so a digest
  difference of this shape is a signal to sweep, not a signal to investigate the diff.

## Amendment 2026-11-13: the control is red under the shared ABC9 script, and stays pointed at `rtl/csrs.v`

**What changed.** Since ADR-0255's amendments, every ice40 synthesis maps through
`soc/ice40_abc9.script`, which `NETLIST_SYNTH` names by relative path. The mutant tree this control
builds copied `rtl/` and the ROM and constraint files but not that script, so the mutant's synthesis
could not find it and the control failed on a missing file rather than on anything it measures.
`soc/netlist_determinism.sh` now copies every `.v` and `.script` file `NETLIST_SYNTH` names, and
`soc/netlist_base.sh` does the same for a base ref that names no synth script of its own.

**What the control then measures.** Re-taken twice on 2026-11-13 (Yosys 0.68+48 `ff5817c34`, the
engineer's run and an independent one on the same branch): the two-line comment this control inserts
at the top of `rtl/csrs.v` moves the up5k SoC's canonical netlist from 4,586 to 4,654 `SB_LUT4`
(+68, 1.5%; 6,612 to 6,680 cells in all), DIGEST-DIFFERENT, deterministically. Comment insertions
into `rtl/decoder.v`, `rtl/executor.v` and `rtl/accessor.v` each pass. This is the effect the
Decision above describes, at roughly three times the size it had under one default ABC9 pass: the
longer script runs `&dch` and `&syn2` twice, and each run is another order-sensitive pass for a
shifted name rank to steer. That attribution is an inference from the mechanism, not a separate
measurement. CI's yosys 0.70 was not measured: `make netlist-determinism` runs on neither `make test`
nor CI.

**Decision: keep `NETLIST_COMMENT_FILE` at `rtl/csrs.v`, and accept a red control.** The
Consequences above already state that a comment-only edit to a large file may fail this control and
that the failure is the control working. Moving the comment class to a file that holds would make the
control pass by aiming it where the effect has not been seen, which is the defect this ADR was
written to correct. A green control on this toolchain would have to rest on a different mechanism,
not a different target.

**What this costs, stated plainly.** On yosys 0.68 with the shared script, `make netlist-digest` and
`make netlist-diff` stop at their prerequisite, so the digest no longer excuses any sweep: every
change that could move the netlist owes the sixteen paired seeds, the answer this repo's timing
rules give anyway when the digest is not available. That is the sound direction. The gate becomes
usable again only when the comment class passes on `rtl/csrs.v`, for example if the shipping flow
stops embedding source line numbers in the cell names `abc9_ops` sorts by. That change would move
every shipping netlist and is separate work, priced on its own sweep.

## Amendment 2026-11-21: on yosys 0.70 the dead net never survives mapping, and the comment class is red for a different reason

**Why the dead-net precondition failed.** On yosys 0.70+360 (OSS CAD Suite 2026-11-09, the pinned
release) the injected `netlist_control_dead` wire is in the design when `synth_ice40` reaches its
`map_luts` stage and gone when that stage ends: the `abc9` pass drops a public wire nothing reads.
Measured by running `synth_ice40 -run :map_luts` and then `abc9 -W 250` by hand with a `select`
count of the wire between them (1 before, 0 after). Every shape of dead net tried vanished the same
way (a 32-bit swizzle, a full-width alias, a one-bit XOR, a reduction, an alias of a live signal);
only `(* keep *)` kept one, and a kept wire also survives `opt_clean -purge`, so it would not be
purged from the canonical form and cannot stand in for the class. The shipping netlist's surviving
extra net names (153 of them on this tree) are all wires some cell reads, such as `flash.mosi`, and
`opt_clean -purge` is what removes those. On 0.68 the same wire survived `abc9` as an
`unused_bits` entry, which is why the old check looked for it in the shipping JSON.

**What the precondition tests now.** `soc/netlist_determinism.sh` looks for the wire in the design
the mapper reads (`$NETLIST_SYNTH -run :map_luts`, then `select -list`), which keeps what the check
was written to catch: a tie-off injected into a submodule is flattened to a different name and not
found, and a frontend that dropped the assign is not found either. It no longer requires the wire to
be in the shipping JSON, because the flow itself deletes it there; it still requires the wire to be
absent from the canonical JSON. The forced-red direction is the same probe in `make probe-gates`
(`a dead tie-off that never reached the netlist exercises no class`), now with a stub whose
pre-mapping `select` finds nothing.

**What the comment class reads on 0.70.** Run on the pinned release with the fixed control: the
two-line comment at the top of `rtl/csrs.v` leaves the canonical netlist at 6,625 cells and 4,599
`SB_LUT4` on both sides, so the +68 `SB_LUT4` of the 0.68 reading does not reproduce here. The digest
is still DIGEST-DIFFERENT, and everything that moved is the numbering of `$auto$rtlil_bufnorm.cc:460:bufNormalize$N`
buffer cells and the net ids beside them inside the two `SB_MAC16` wrapper modules, which
`soc/netlist_digest.py` keeps because it drops only `src` and `module_src`. Placing the mutant
with `nextpnr-ice40 --up5k --package sg48 --pcf soc/littlesoc.pcf` (the default seed) writes a
bitstream byte-identical to the base tree's, both at 14.33 MHz. So the control stays red by the
digest's own rule, the placer is unmoved, and the red is a naming effect and not a mapping one.
The earlier stages pass: two placements of the base netlist agree byte for byte, the mutant's
shipping netlist differs from the base's, and the dead net reaches the mapper.

**Decision: unchanged.** `NETLIST_COMMENT_FILE` stays `rtl/csrs.v` and the control stays red, as the
2026-11-13 amendment decided; the digest gate remains void on this release and every change that
could move the netlist still owes the sixteen paired seeds. This amendment records what the red now
means: a classifier that does not forgive buffer-cell numbering, not an unforgiven mapping change.
Forgiving it is a change to the canonical form, which that amendment's argument applies to as it
applied to moving the file, and is left as separate work.

## Amendment 2026-11-21: bufNormalize cells are named by content, and the gate is live on 0.70

The 2026-11-21 amendment above left the 0.70 red as a naming effect and put forgiving it down as
separate work. This is that work. `soc/netlist_digest.py` renames each cell matching
`$auto$rtlil_bufnorm.cc:N:bufNormalize$M` to `$auto$bufNormalize$<sha256 of the cell's whole
content>#<rank>` before it hashes, where the rank counts cells of identical content. Nothing else is
renamed: every other cell name and every net name is kept as before.

**Why it is sound.** The hash covers type, parameters, attributes and connections, and the cell keeps
all of them in the digested value, so a buffer wired to another net, widened, or given another
attribute differs in its value and in its name. The rank makes the renamed cells the same multiset
as the originals, so a buffer that disappeared, or one more identical to an existing buffer, still
changes the dictionary. What is forgiven is only the counter's value, which `M` is: the pass assigns
it from `$autoidx`, so a comment upstream shifts it with no change to the mapped logic. Connections
hold net ids and no cell names, so nothing else refers to a name that was dropped. What the form
cannot tell apart is which buffer held which counter value, which no input to the placer records.

**What the control reads on 0.70.** On yosys 0.70+360 the `rtl/csrs.v` comment class is now
DIGEST-EQUAL, and `make netlist-determinism` passes end to end: two placements of the base agree,
the mutant's canonical digest equals the base's, and the mutant places to the base's bitstream
byte for byte. A real mapping change still reads different: `make probe-gates`'s `a one-bit constant is a different digest` stays red.

**Decision: the gate is live on the pinned release.** An equal digest again skips the sixteen-seed
sweep on 0.70, on the same one-directional claim as before: equal is sound, different implies
nothing. It stays void on any release where `make netlist-determinism` is red, as on 0.68. The
forced-red probes in `make probe-gates` (`bufNormalize cells renumbered...` and the four that follow
it) hold both sides: a renumbering is forgiven, and a rewired, deleted, duplicated or non-bufNormalize
auto-named cell is not. `test/netlist_digest_test.py` grades the canonicalizer directly.

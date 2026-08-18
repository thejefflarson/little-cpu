# ADR-0120: A netlist that cannot have moved buys its sweep back

**Status:** Accepted · 2026-08-17 · *Adds an instrument, no ratchet and no gate on the shipping
design. Amends no commitment. Qualifies ADR-0113's sixteen-seed rule by saying when the sixteen are
owed at all.*

## Context

Sixteen placements are about twelve minutes a side, ADR-0113 made sixteen the floor rather than
eight, and ADR-0111 priced margin as the currency every declined win is queued against. Meanwhile
the multi-hart work coming to ECP5 lands its first changes as **tie-offs on a design whose worst
placement of sixteen clears 12 MHz by a fraction of a nanosecond**, with area measured twice not to
buy any of that back (ADR-0088, ADR-0090). A change that cannot have moved the placer's input at all
should not cost a sweep, and today every change costs one because nothing in the tree can tell the
two apart.

The obvious instrument — hash the mapped netlist — does not work, and that is measured rather than
assumed. On `553a241` with the cached OSS CAD Suite (Yosys 0.68+48 `ff5817c34`, nextpnr-0.11
`62e659ed`):

- **Synthesis is byte-deterministic run to run.** Two builds of `littlecpu`, identical SHA-256.
- **A comment-only edit moves the mapped JSON in 66 places, all of them `src` / `module_src` line
  numbers.** Nothing else in the file moves.
- **A dead wire yosys optimises away moves it too.** `assign dead_tieoff = {word[15:0],
  word[31:16]};`, never read, leaves a `netnames` entry carrying `unused_bits` behind and renumbers
  every net bit id after it: 104 differing lines injected into `rtl/regsel.v`, 1447 into
  `rtl/decoder.v`.

So strict equality answers "different" for exactly the class the gate would exist for, which
degenerates it into "always sweep" — a grader with one verdict.

## The canonical form

The shipping synth script verbatim, plus `opt_clean -purge`, with `soc/netlist_digest.py` dropping
`src` and `module_src` from every attributes object and dropping nothing else. Both halves are
load-bearing and were measured apart: the purge removes the dead net and restores the numbering, the
attribute drop survives a comment moving a line, and neither alone forgives both classes.

Under it, on the SoC that places:

| tree | shipping netlist | canonical digest | placed bitstream, one seed |
|---|---|---|---|
| base | `cd9fc674…` | `17c60265…` | `8da98943…` |
| a comment inserted | `db5f2a1c…` | `17c60265…` | `8da98943…` |
| a blank line inserted | `447e3e06…` | `17c60265…` | `8da98943…` |
| a dead tie-off injected | `8e9256c1…` | `17c60265…` | `8da98943…` |

Four different netlists, one digest, one bitstream. A one-bit change to the `misa` constant is a
different digest, and the report names it: −7 `SB_LUT4`, +3 `SB_CARRY`, −4 cells, +27 named nets.

## The control, and the experiment that changed its shape

The gate is void without two facts, and neither is derivable, so `make netlist-determinism` re-takes
both on the tree being asked about and is a prerequisite of the digest the way `pcloop_cover` is of
`pcloop`.

**A. Placement is a function of the netlist.** One netlist placed twice at one seed gives a
byte-identical bitstream. It does here, over 7 309 930 bytes.

**B. What the canonicalisation forgives, the placer does not see.** This was specified as placing the
purged and unpurged forms of one tree and requiring one bitstream. **That control was run and it
fails**: the purged netlist places to `e47243d7…` where the shipping one places to `8da98943…`, same
seed, same design, same tools — both reporting 12.80 MHz from nextpnr's own estimator, so the
difference is the placement and not the quality of it. `opt_clean -purge` moves the placement.

What follows from that is narrower than it looks. **Nobody places the canonical form.** The proxy
was sufficient, not necessary, and the necessary question is whether the differences the digest
forgives move the placement of the *shipping* netlist — which is the table above, and which passes.
So the gate lands whole rather than narrowed to strict equality of the unpurged form, and control B
asks the direct question instead: a copy of the tree carrying all three forgiven classes is
synthesised in the shipping form, and its bitstream must equal the base's while its digest stays
equal. The mutant's shipping netlist must also **differ** from the base's, or the control passed
having injected nothing.

## Decision

**`make netlist-digest` and `make netlist-diff BASE=<ref>` are the instrument, and the claim is one
direction wide.** Digest-equal implies the placer's input differs only in dead nets and source
attributes, so the placement distribution is unmoved and no sweep is owed. Digest-different implies
nothing at all, so the seeds get spent.

**The landing procedure, written where the sweeps are** (`soc/baseline_sweep.sh`,
`soc/timing_sweep.sh`): digest unchanged, no sweep owed; digest changed, the paired sixteen-seed
sweep is owed. And at today's margin a difference is a **stop-and-redesign signal rather than merely
a sweep owed** — there is nowhere to put the cells.

**It replaces a sweep, never a gate.** `make fit` and `make soc-timing` are graded against exactly
what they were graded against before this, and the synth script and placement command they use are
now named once in the Makefile so the digest cannot describe a netlist nothing builds. Nothing here
runs on CI and nothing here adds a ratchet.

**The toolchain is inside the digest.** `write_json` records its `creator` string, so a yosys that
moved reads as different — the sound direction, and the one this repo has been bitten in twice.

**The canonical netlist is a classifier, not a design.** It is not what ships and nothing may hand it
to nextpnr. That is what the failed proxy above bought.

## What was considered and rejected

**Digesting the netlist `make fit` synthesises instead.** `littlecpu` alone is the area instrument,
but the placement — which is what the sweep costs and what the margin is about — is `littlesoc`'s.
An implication for `fit` does not follow from the SoC's digest either: a tie-off dead inside the SoC
can be live at a top whose inputs are unconstrained. One part, one design, one claim.

**Comparing both trees through this tree's synth script.** A flag that moved between the two would
then be invisible, which is a change to the placer's input like any other. `soc/netlist_base.sh`
asks the base tree for its own script and uses that, and says so in its report when the base tree
names none.

**Grading the purged-versus-shipping placement as a third control.** It is a fact about a netlist
nobody places, and requiring it to keep differing would go red for a toolchain improvement. It is
recorded here and in that script's header, where it stops the canonical form being placed by
mistake, rather than asserted.

## Consequences

- **A tied-off change states its digest and owes no sweep.** The three tickets queued behind this one
  are all of that shape.
- **The control is re-taken per invocation, not quoted.** Three placements and two syntheses, about
  three minutes against twelve for a sweep a side, and the empirical half of the claim is therefore
  as fresh as the tree it is claimed about.
- **A digest is comparable only within one toolchain**, by construction rather than by convention —
  the same argument `soc/baseline_summary.py` already makes by refusing to subtract two sweeps whose
  stamps disagree.
- **The instrument is part-parameterised and used on up5k**, where the seeds are expensive and the
  requirement binds. A part is one block in the Makefile; an unknown one is refused rather than
  digested, because a table with nothing in it would synthesise nothing and call two empty trees
  equal.

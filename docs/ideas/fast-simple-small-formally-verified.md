# Fast, simple, small, formally verified

**Status:** executed · 2026-08-02 · landed as the `CLAUDE.md` rewrite of 2026-08-03 (commit 5a612c6): design commitments keep their numbers, 7 and 9 are retired; the `parallel_case` rule is ADR-0068.

Four goals. They are not in tension, and the formal harness is what lets us have all of
them — it licenses simplifications that would otherwise be too risky to make.

This brief restructures `CLAUDE.md`'s invariants section, which currently reads as nine
commandments of equal weight, and replaces the sentence that would forbid the work.

## The thesis, with evidence

The decoder proves its instruction flags are one-hot. `rtl/decoder.v:1184` asserts
`$onehot(...)` over about forty of them, and `rtl/executor.v:320` assumes the same set and
cites that proof as its discharge.

That proof is what makes `(* parallel_case *)` legal. Without it the attribute is a lie to
synthesis; with it, a `case (1'b1)` over mutually exclusive flags becomes a flat mux —
about `log N` deep instead of `N`.

Four of the six such statements in `rtl/decoder.v` already carry it. **Two do not**, at
lines 483 and 777, and those synthesize as priority chains preserving an ordering the
design has proven can never be observed.

So the flat mux is:

- **faster** — fewer LUT levels on the path that sets Fmax
- **simpler to read** — a priority chain implies an ordering that does not exist, so the
  chain is the misleading spelling
- **smaller** — a chain of N muxes against a tree
- **licensed by the proof** — the thing that makes it safe is the thing that makes it fast

That is not a trade. It is one lever with four names, and it is the pattern this project
should reach for whenever the four goals appear to conflict: **prove the property, then
spend it.**

## What is wrong with the invariants section today

Nine numbered entries share one heading and one threat level: *violating one is a bug even
if tests pass.* Three are architectural bets with measurable cost. The rest are contracts,
consequences, and one line of file hygiene. Entry 7 is "do not hand-edit a generated file",
sitting beside the design's central commitment in the same voice.

Entry 8 is twenty lines of design description — an inventory of stall reasons that a change
landing this week already had to edit, in two places, to keep in step with the ADRs.

## What it becomes

**Two groups. Numbers become permanent identifiers and are never reused.**

That last part is the migration: about 130 references to `invariant N` exist, and 56 of
them are inside ADRs, which are records and cannot be rewritten. Renumbering would create
a mapping table that never goes away. Keeping the numbers and tombstoning the retired ones
costs **zero code changes**.

### Bets — three of them

Each states what it buys, what it costs, and how it gets reopened. Header sentence:
*changing one is a new ADR that pays the listed price.*

- **1 — no wrong-path state.** Reworded, below.
- **2 — all traps detected and committed in decode.** Unchanged in strength. Gains its
  falsifier: a bus that can refuse raises faults after decode, five declined ladder checks
  are declined against exactly this, and ADR-0044 holds three options unpicked. ADR-0059
  chose a non-faulting bus for the text region, so the bet is currently unpressured.
- **4 — no forwarding network.** Already correctly worded; it names its escape hatch. Gains
  one line of measured price: the rejected second bypass level was 44/52 (ADR-0042).

### Contracts — the things that are silently breakable

One to three sentences each, plus a pointer to what enforces them, and an explicit
`prose-only` flag where nothing does. Header sentence keeps *violating one is a bug even if
tests pass*, because it is true of them — an architectural write past `reg_ch0`'s bound is
invisible to every gate (ADR-0032).

- **3** — retire is `valid` reaching writeback. This is the definition of retire and the
  RVFI interface contract, not a convention.
- **5** — CSR instructions and `mret` serialize. This is the mechanism that makes bet 2's
  precision and ADR-0027's `minstret` exactness true.
- **6** (with **9** folded in) — decode observes the architectural value of rs1/rs2
  including a same-cycle writeback, and the regfile's answer belongs to the address pair
  presented last cycle. The first sentence predates the synchronous read and is what
  ADR-0004's scoreboard stands on. Enforced by `test/regfile_tb.v`, `reg_ch0`, and
  `pcloop`'s transcribed `operand_stall` register.
- **8a–c** — the three non-local stall rules, kept verbatim in substance. Rule (c) has
  already caught a real bug shape. The *inventory* of stall reasons leaves; it lives in the
  ADRs and in the code.

### Evicted

- **7** moves to "Engineering rules in force". It is hygiene.
- **8's inventory** leaves entirely. Two documents no longer have to be kept in step.

### Tombstones

One line each, pointing at the new home. `7 → Engineering rules in force.`
`8's inventory → ADR-0026, ADR-0042 and successors; rules a–c above.` `9 → folded into 6.`

### The admission rule

In the header, so the section cannot re-grow: **a new entry is a bet with a measured price
or a contract with an enforcement pointer. Descriptions go in ADRs.**

## The no-flush rewording

Replace *"No flush logic may be introduced, ever"* with:

> **There is no wrong-path state: no state may exist that a later cycle must un-commit.**

A bubble (`valid = 0`) is not revocation. A kill signal, a re-steered fetch register, or an
issued-but-revocable result is.

This is better than the old wording in two ways. It is checkable by reading a diff. And it
does not rest on a claim the measurements no longer support.

**The claim that is struck:** that no-flush is what keeps the formal ladder converging.
Measured, the ladder's cost scales with depth floors, not state size at this scale —
ADR-0057 moved three `[depth]` lines and went 7m14s → 13m42s, about 2× wall for +4 depth.
That is a slope, not a cliff. The cliffs this repo has actually hit were never
state-growth: the multiply miter returned no verdict at *depth 1* (an operator cliff,
ADR-0051), `equiv.sh` diverged on name matching plus induction (ADR-0047), and
`clk2fflogic` was a semantics mismatch (ADR-0040). riscv-formal's RVFI interface is
designed so that speculation is invisible at retire. **A flush the ladder can model is
fine.**

**What the bet actually buys**, and this survives: F = 6 and G = 6 stay small and derivable,
and every `[depth]` line is a function of them (ADR-0046, ADR-0057). Bet 2's precision needs
no reorder buffer. Retire needs no filter. And `pcloop`'s k-induction never has to invent an
invariant over speculative state.

**What it costs:** the fetch loop is the critical path, and everything on it is paid once per
cycle.

~~the design sits at 10.2–10.4 MHz against a 12 MHz crystal whose next divider step is 6.~~
**Struck.** The design reaches **12.7–13.5 MHz** across four placements, and 12 MHz is a
requirement now rather than an intent. It got there by finding one comparator on the wrong side of
the instruction — the write-through bypass selected on a live combinational `rs1` when the contract
already said the answer belongs to the previous cycle's pair. Six LUT levels for four lines.

**That changes the bar for reopening the bet, and it is the more useful correction.** This section
was written when the bet looked like it was costing the board clock. It was not. A flush can no
longer justify itself by reaching 12, because 12 is met without wrong-path state — the next thing
worth wanting is 24, which needs roughly 11 of the remaining 23 levels. Whether that is reachable
at all is measured separately, and it may not be.

## Reopening bet 1

Written now, so the decision arrives as an ADR with priced conditions rather than as
heresy. Reopening requires demonstrating:

1. **The target is unreachable without wrong-path state.** The cheap work comes first: two
   missing `parallel_case` attributes, the decode head, and the `minstret` carry path, which
   caps the design at 11.49 MHz independently of the fetch loop (ADR-0058).
2. **The full chain measures ≥ 12.0 MHz at every placement**, clearing both noise bands
   (4–9% placement, 3.6% edit — CLAUDE.md carries both) at twelve to sixteen seeds. A flush
   that lands at 11.6 buys nothing and costs the bet.
3. **F and G re-measured under the kill**, and every `[depth]` line re-derived in the same
   change (ADR-0046's rule).
4. **The machinery reads well.** One killed valid bit on a registered fetch might pass that
   bar. A skid buffer does not.

The section enshrines the procedure, never the number 12.

## Method: iterate through simplification

The point of having the harness is that simplification stops being risky. The loop:

1. Find a place where the design pays for a property it already proves — a priority chain
   over proven-disjoint flags, a comparator that cannot differ, a mux with an unreachable
   arm.
2. Simplify it.
3. The ladder, the component proofs and the `.S` suite say whether the property still holds.
4. `make fit` and `make soc-timing` say what it bought.

Every step of that is instrumented today. Nothing new is needed to start.

## What this is not

**Not an RTL change.** No `rtl/`, `formal/` or `test/` file moves. CI is untouched by
construction.

**Not a change to what is permitted.** This restructures justifications and shape. Every
rule that binds today still binds tomorrow.

**Not a licence to add rules.** The section goes from about 58 lines to about 35, and the
admission rule is there to keep it that way.

## Sequence

1. Land the sixth-stall-reason change first — it edits entry 8, which this brief deletes.
2. One PR against `CLAUDE.md`.
3. One ADR recording the rewording of bet 1 and its reopening procedure. It supersedes the
   *wording* of invariant 1, not its substance.
4. Separately, and not part of this: fix the two missing `parallel_case` attributes and
   measure. That is the first application of the method, and it is a real change with a
   real number.

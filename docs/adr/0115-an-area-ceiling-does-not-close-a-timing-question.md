# ADR-0115: An area ceiling does not close a timing question

**Status:** Accepted · 2026-08-16 · *Narrows how ADR-0090, ADR-0094, ADR-0096, ADR-0097 and
ADR-0112 are read. Supersedes none of them — every measurement in them stands. Amends four clauses
in `CLAUDE.md` that restated them more broadly than they were taken.*

## Context

This repo has taken a lot of deleted-whole ceilings, and they have paid: ADR-0090 found −404 cells
in `rtl/executor.v` after that block was considered done, ADR-0094 killed three plausible decode
edits before anyone built them, ADR-0112 this week closed most of a planned harvest for a day of
synthesis runs.

Every one of them asks the same question: **how many cells does deleting this block save?**

An agent was then asked to read `rtl/` for optimisations with `docs/adr/` explicitly out of scope.
It found six, and **three of them are in blocks these ceilings had closed** — not because a ceiling
was wrong, but because none of them was ever a claim about the period.

## Decision — a ceiling bounds cells, and only cells

**An area ceiling says nothing about what a construct costs on the critical path.** A block whose
deletion saves no cells can still hold a term that adds a LUT level to a loop, and this is not a
hypothetical:

- **`rtl/decoder.v`'s `instr_error`** read the *muxed* `rs1` and `rd` rather than the raw
  instruction fields. The term requires `uncompressed`, where `regsel`'s default arm gives
  `rs1 = rs1_field` and the `rd` case's default gives `rd = rd_field` — so the two spellings are
  identical in value. The muxed one puts the entire compressed-decode cone into `trap_pending →
  trap_taken → next_pc`, which the placement report shows at 56–62 ns of an 82.49 ns path.
  **The correction costs nothing in cells**, which is exactly why every ceiling taken over that
  block missed it.
- **`rtl/writeback.v`** masked `waddr` and `wdata` with `wen`, which all four consumers in
  `rtl/regfile.v` already test. Redundant, and it does survive DCE, because the bypass mux it feeds
  had already spent four LUT inputs so ABC had nowhere to fold the AND. **This ADR then treated that
  as a reason to delete it, and the measurement says otherwise**: removing the masks is +2.83% of
  median period and **11.89 MHz at the worst of sixteen seeds**, under the board clock, for 32 cells
  (ADR-0117). They stay. The mechanism above is confirmed and the inference from it was wrong —
  finding a redundant term on a path says it is a candidate, not that removing it is a win.
- **`rtl/decoder.v` presented a register pair decoded from a data word** on a stolen-fetch cycle,
  which `operand_stall` then correctly refused, costing a second bubble on every load or store into
  the text range.

## The four clauses this amends, and why each misled

Each is a fair summary of the ADR behind it and a poor rule read alone.

**"yosys and ABC already do everything derivable from the expression — dead bits, common
subexpressions, duplicate adders."** True of the mapper, and it predicts the `wen` mask is already
gone. It is not. **Dead logic is free only where ABC has a LUT input to fold it into** — a fact
about the consumer, not about the expression. The clause now says so.

**"that block is closed on two measurements; read the ceilings before reopening it."** Closed for
*area*. `instr_error` is in that block and cost a level for nothing. The clause now names the unit.

**"There is no single lever."** A bound on what one *deletion* buys. It says nothing about a
construct already on the path that can be spelled differently for free, and it was read as closing
the fetch loop to inspection entirely.

**"Touching `operand_stall` is an amendment, not a tuning change."** Correct, and it is a rule about
**the guard**, not about the neighbourhood. What is *presented* on a stalled cycle is a separate
question — `operand_stall` stays the exact compare whatever is presented, so a change there is
checked by that signal rather than weakening it. The clause's blast radius exceeded its scope and a
whole region went unexamined.

## What is not being claimed

**The ADRs are not stale and none is superseded.** −404 cells is still −404 cells; the compressed
decode is still 253 LUTs deleted whole; ADR-0112's table still closed four blocks correctly and
saved the week it was taken to save. **Deleting them would remove the evidence for the practice that
paid for itself twice.** What changes is how far their conclusions travel.

**Nor is this a claim that ceilings are the wrong instrument.** They are the cheapest way to find
out whether a block is worth days, and this ADR does not license re-opening a closed block for
cells without a reason its ceiling does not already answer — ADR-0111's taxonomy is untouched.

## Consequences

- **A timing question needs a timing measurement.** No deleted-whole *period* ceiling has ever been
  taken here — what does the period do if this block vanishes — and it is a different, cheaper
  experiment than the area one. Nobody has run it.
- **Read the placement report against the RTL, not only the cell counts.** Both timing findings above
  came from `soc.timing.rpt` and the netlist. Neither is visible in any census or ceiling table.
- **A tripwire at the site outperformed a tripwire in a decision record.** The same agent correctly
  skipped the duplicate `imm + rs1` sum because `opt_merge` already shares one carry chain, and
  skipped the fetch-window register, the forwarding network and the flattened `next_pc` chain —
  every one of those declines is recorded in a comment beside the code it is about. The declines
  recorded only in `docs/adr/` deterred inspection instead of informing it.
- **This file is a rulebook, and a rule that over-reaches is a cost.** Four clauses summarising
  correct measurements closed three questions nobody had measured. That is worth watching for in the
  next summary as much as in the next measurement.

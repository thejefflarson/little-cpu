# ADR-0011: Load/store misalignment detection stays in the accessor until M3

**Status:** Accepted · 2026-07-27 · *Scoping decision under ADR-0005*

## Context

ADR-0005 states that all traps — including load/store misalignment — are detected and committed in
**decode**, and that "nothing can fault after decode." That is the target design, and it is what
makes precise trap handling possible without a reorder buffer.

Today, misalignment is detected in `rtl/accessor.v:25-27` and ORed into `trap` at
`rtl/littlecpu.v:51`. That is post-decode, and it contradicts the invariant.

The ADR-0004 interlock work touches exactly this wiring. Without an explicit scoping decision, the
engineer implementing the interlock will encounter the contradiction and may "helpfully" restructure
trap detection mid-ticket — quietly growing an ADR-0005 implementation inside a ticket scoped to
hazard control, in the same files, in the same PR.

## Decision

**The move to decode-stage misalignment detection is M3 work, not M1.**

The M1 interlock ticket's only obligation regarding misalignment is to **gate the existing
accessor-side signal with `valid`**, so that bubbles and squashed instructions do not raise spurious
traps. The detection logic stays where it is.

When M3 implements the CSR file and machine-mode traps, misalignment detection moves to decode
along with illegal-instruction, `ecall`, and `ebreak` — as one coherent change, in one ticket,
against ADR-0005.

## Rationale

Trap restructuring and hazard control are separable concerns that happen to touch adjacent lines.
Bundling them makes the M1 ticket unreviewable and gives it no clean acceptance criterion — "traps
are precise" is not checkable until there is a `mepc` to check it against.

Deferring costs nothing: with `valid` gating, the accessor-side signal is functionally correct for
M1's purposes (the `.S` suite has no misalignment tests, and there is no trap handler to enter).
The invariant is temporarily unmet, but it is unmet *today* — this decision does not make anything
worse, it just declines to fix it in the wrong ticket.

## Consequences

- **ADR-0005's "nothing faults after decode" invariant is not fully satisfied until M3.**
  `CLAUDE.md` states it as a design invariant, which is correct as an intent; this ADR records that
  the implementation lands in M3.
- The M1 interlock ticket carries an explicit constraint note forbidding the restructure.
- The M3 CSR ticket carries the move as an explicit deliverable, so it does not get lost.
- M2's formal parity checkpoint does not depend on this — the serialized core never had precise
  traps either, so parity is unaffected.

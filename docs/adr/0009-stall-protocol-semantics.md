# ADR-0009: Stall protocol — upstream freezes, downstream drains

**Status:** Accepted · 2026-07-27

## Context

ADR-0004 says the decode scoreboard "freezes the PC and emits a bubble" and that the divider
"freezes decode globally and drops `valid` until it completes." That text is underspecified in a
way that matters: it does not say whether stall is a single broadcast signal or a chain of
per-stage ready signals, and — the part that actually bites — whether the accessor and writeback
**keep draining** while the executor is dividing, or freeze along with it.

Draining is what ADR-0004 implies and what avoids the memory-replay defect (today the accessor
re-issues the previous load/store once per divide iteration). Freezing is the more obvious
implementation. Two competent engineers will build these two different ways from the current text,
and only one of them is correct.

## Decision

**A single global `stall` broadcast from `rtl/littlecpu.v`.**

- **Upstream of the stalling stage freezes.** The PC holds; the fetch window re-presents the same
  bytes (which is also exactly the rand-stable behaviour ADR-0003 asks the formal wrapper to
  model); decode holds its output.
- **Downstream of the stalling stage drains.** Instructions already past the stall point continue
  to retire normally.
- **`valid = 0` is injected at the stalling stage's output**, so the drain is fed bubbles rather
  than repeats. This is the direct fix for the divide-replay defect.
- **No per-stage ready/valid handshake.** A skidbuffer/handshake pipeline existed here once and was
  deliberately removed in `49b317a`. Do not reintroduce it.

**Reset semantics:** reset clears `valid` on every struct and zeroes the payload. Zeroing costs
nothing in FF count that matters and makes `formal/complete.sv`'s reset handling trivial in M2.

## Rationale

A single broadcast is the simplest thing that works and matches the design's existing shape — there
is no backpressure source in this core other than the divider and the decode scoreboard, so a
handshake network would be machinery with one customer.

Freeze-upstream/drain-downstream is the only variant that fixes the memory replay. Freezing
everything would leave the accessor holding a live load/store request across the whole divide,
which is the bug.

## Consequences

- Add to the `CLAUDE.md` invariant list, since it is the kind of thing a later change can silently
  violate.
- The regression test is direct and checkable: **for every store, `|mem_wstrb` is high for exactly
  one cycle.** That assertion belongs in the testbench permanently.
- Formal assertions this enables: a stalled cycle never advances `pc`; `valid == 0` implies
  `out.rd == 0`; `rs1 == 0` / `rs2 == 0` never cause a stall.
- Worst case is a 2-cycle RAW bubble and a ~32-cycle divide. Accepted per ADR-0004.

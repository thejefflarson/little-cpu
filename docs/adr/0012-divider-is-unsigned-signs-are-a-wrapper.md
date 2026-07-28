# ADR-0012: The divider is unsigned; sign handling is a wrapper around it

**Status:** Accepted · 2026-07-27 · *Supplements ADR-0010*

## Context

JEF-605 was scoped to seven itemized datapath/decode defects. Building the ADR-0010 differential
bench surfaced an eighth, in the same three lines of `rtl/executor.v` the ticket already had open:

`executor.v`'s restoring divider is **unsigned**. It loads `mul_div_x <= {32'b0, rs1}` and
`mul_div_y <= {1'b0, rs2, 31'b0}` and iterates compare/subtract/shift. Sign restoration was already
present at capture (`in.rs1[31] != in.rs2[31] ? -store : store` for `div`, `in.rs1[31] ? -x : x` for
`rem`) — but the operands fed *in* were raw two's complement. Every signed division with a negative
operand was therefore wrong, and no check in the repo would have caught it, because ALTOPS means
riscv-formal never runs the real divider and there was no `.S` M-extension coverage at all.

The engineer raised this as a `DECISION NEEDED`: fix out-of-list, or ship a bench that fails?

## Decision

**Fix it in JEF-605**, and record the structure it implies.

**The restoring divider operates on magnitudes only.** `div_x`/`div_y` are the absolute values of
`rs1`/`rs2` for `div`/`rem`, and pass through unchanged for `divu`/`remu`. Signs are applied on the
way in (negate) and restored on the way out (negate the quotient when the operand signs differ; the
remainder takes the dividend's sign). The two RISC-V special cases — divide-by-zero and
`INT_MIN / -1` — are handled combinationally in `init` and never enter the loop.

Fixing it here was correct rather than deferring: it is in `rtl/executor.v`, which JEF-605 already
owned exclusively (JEF-604 held `Makefile`/`formal/`, so there was no collision risk), and ADR-0010
makes the differential bench a mandatory deliverable of this ticket. A ticket that requires a bench
and forbids the fix the bench demands is not a shippable ticket.

## Rationale

Radix-2 restoring division has no signed form. Every implementation either converts to magnitude or
uses non-restoring division with sign correction. Magnitude conversion is two negations and reads
straightforwardly; the alternative buys nothing on a project whose stated goal is readability
(`CLAUDE.md`), and the deferred-decisions list already rules out radix-4 for the same reason.

## Consequences

- **The executor reads `in.is_div`/`is_divu`/`is_rem`/`is_remu` fresh at capture time**, 33 cycles
  after issue — they are not latched with the operands. This is correct only because ADR-0009's
  stall protocol holds the decode output steady for the whole multi-cycle operation. The executor's
  component proof states this as an explicit environmental assumption
  (`rtl/executor.v`, the `state == divide || $past(state) == divide` assume block). **If ADR-0009's
  stall wiring ever lets `in` move mid-divide, the divider silently returns the wrong sign** —
  latching the op select at issue is the fix, and it is cheap.
- The component BMC proves the divider by loop invariant over `div_x`/`div_y`, not over
  `in.rs1`/`in.rs2`, so it covers the magnitude conversion rather than assuming it away. It is
  restricted to 4-bit operands per ADR-0010's bounded-effort clause; the restriction is documented
  inline in `rtl/executor.v` rather than in the sby task, because JEF-604 owned
  `formal/components.sby` in the same sprint. **Move that comment to the sby task** when convenient —
  ADR-0010 asked for it there.
- Verified non-vacuous by mutation: reintroducing the `mul_sign_x` bug makes
  `sby -f formal/components.sby executor` fail at `executor.v:232`, and reverting the magnitude
  conversion makes `test/exec_tb.v` mismatch within the first few random vectors.
- ALTOPS builds are untouched; they never enter this path.

# ADR-0010: A randomized differential bench is the primary mul/div guarantee

**Status:** Accepted · 2026-07-27 · *Supplements ADR-0006*

## Context

riscv-formal runs with `RISCV_FORMAL_ALTOPS`, which replaces multiply and divide with cheap
placeholder functions. **The formal ladder therefore never checks the real arithmetic.**
ADR-0006 designates a depth-~70 executor BMC against the SystemVerilog `*`, `/`, and `%` operators
as the plug for that hole.

Two problems surfaced when the plan was checked against the actual code:

1. **The BMC may not converge.** A 33×33 bitvector multiply plus a 32-iteration restoring divider
   unrolled to depth 70 is a genuinely hard SMT instance. The multiplier half is easy; the divider
   half is of uncertain tractability. Two independent reviewers flagged this.
2. **There is no `.S` coverage of the M extension at all** — `test/asm/` contains no `mul.S`, no
   `div.S`, nothing. So today the arithmetic has *zero* checks of any kind.

This matters more than it looks, because the multiplier turns out to be badly broken:
`rtl/executor.v:28-30` uses **declaration initializers**, not continuous assignments, so the
product is evaluated once before time 0 and never again — all four MUL variants return a constant.
The sign enables are also swapped (MULHSU is signed × unsigned, the code has it backwards), and
the sign bits are taken from bit 0 rather than bit 31. Three stacked defects in three lines, none
of which any current check would catch.

## Decision

**The randomized differential unit bench is the primary guarantee. The BMC is secondary and
bounded-effort.**

1. **Primary — `test/exec_tb.v`**, run under iverilog on every PR: drives `executor` standalone with
   ≥10,000 randomized operand pairs per operation across `mul`, `mulh`, `mulhu`, `mulhsu`, `div`,
   `divu`, `rem`, `remu`, comparing against SystemVerilog `*`, `/`, `%` with RISC-V divide-by-zero
   and `INT_MIN / -1` semantics. Exits non-zero on any mismatch.
   **Required directed vectors:** `MULH(-1,-1) = 0`, `MULHSU(-1,1) = -1`,
   `MULHU(-1,-1) = 0xFFFFFFFE`. These are exactly the cases the swapped sign enables get wrong.
   The bench also asserts the divider produces its result **33 cycles** after issue (32 iterations
   plus capture), which is the regression test for the 65→32 iteration-count fix.
2. **Secondary — the executor BMC** (`sby -f formal/components.sby executor`): nightly,
   bounded-effort. If it does not converge within 30 minutes, restrict it (bounded operand range or
   reduced depth) and **record the restriction as a comment in the sby task**. A restricted proof
   that runs is worth more than an unrestricted one that never returns.
3. **Never vacuous.** `components_executor` currently reports `PASS 0 0` — it passes because it
   contains no assertions at all. A task with zero assertions is a lie and must not be allowed to
   stand as a gate.
4. **The eight `rv32um` tests** (`mul`, `mulh`, `mulhsu`, `mulhu`, `div`, `divu`, `rem`, `remu`) are
   added to the `.S` suite as end-to-end coverage.
5. **The BMC does not gate PRs until its runtime has been measured.**

## Rationale

The purpose of the ALTOPS hole-plug is to catch arithmetic defects. A check that may not terminate
cannot be the primary mechanism for that. A randomized differential bench over 10k vectors per
operation will find every defect currently in this code within seconds, and it runs in CI without
risk.

This is not a retreat from formal methods — it is putting the exhaustive tool where it converges
and the statistical tool where it doesn't.

## Consequences

- Amends ADR-0006's component-proof #2 from "the guarantee" to "one of two, the slower one."
- The PR gate gains a fast, reliable arithmetic check instead of a possibly-hanging one.
- If the BMC proves tractable at full depth, promote it later — that is a strict improvement and
  needs no new ADR.
- `rtl/memory.v` has the same shape of problem: it is in **no** current test path (the cxxrtl top is
  `testbench`, which has its own inline memory, and the synthesis path is broken). Its
  read-override bug needs a dedicated bench for the same reason.

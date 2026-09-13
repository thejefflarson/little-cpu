# ADR-0181: nano's divider is fixed, and checked by its own differential oracle

**Status:** Accepted · 2026-09-12

## Context

ADR-0180 recorded nano's real divider as broken -- `div 20, 6` returned 0, not 3 -- and
baselined `divide.S MONITOR-ERROR 105` rather than fix it, because riscv-formal's
generated checks run under `RISCV_FORMAL_ALTOPS` and nano had no `test/exec_tb.v`
equivalent to catch the defect any other way.

## The defect was three bugs, not one

`nano.v`'s divide state loaded `mul_div_x` and `mul_div_y` from `regs[rs1]`/`regs[rs2]`
verbatim, compared them with `mul_div_x <= mul_div_y`, and ran for 65 iterations:

1. **The comparison was backwards.** The divisor sits pre-shifted into bits [62:31], far
   larger than the dividend at the first iteration, so `x <= y` is true immediately, sets
   the top quotient bit, and subtracts a huge value from a small one -- corrupting the
   remainder on iteration one, for every dividend and divisor tried.
2. **The iteration count was wrong.** A divisor pre-shifted by 31 needs exactly 32 steps
   of `y >>= 1` to walk back to bit 0; 65 both wastes 33 cycles and, combined with bug 1,
   never lands on a correct state.
3. **Signed operands were never converted to magnitude.** `DIV`/`REM` loaded the raw
   two's-complement bits of a negative operand as if they were its unsigned value, so
   even a corrected loop divided the wrong numbers; only the final sign restore assumed a
   magnitude computation underneath it.

Fixing the comparison and the count alone reproduces `20 / 6 = 3` (verified against
non-RTL magnitude arithmetic before either fix was applied to `nano.v`), but a negative
operand still fails without the magnitude conversion, and a zero divisor overflows into
`-mul_div_store` for a negative dividend -- a zero divisor drives every loop comparison
true regardless of the dividend, so `mul_div_store` always ends up all-ones, and the
naive sign-flip is correct for a real quotient but not for that constant. `DIV`'s sign
restore gates on `rs2 != 0` rather than adding a second arm that restates the all-ones
constant: when `rs2 == 0` the gate is false and the unflipped value passes through, which
is already the right answer. `REM`'s magnitude arithmetic produces the right by-zero and
`INT_MIN / -1` results with no gating at all, matching what the oracle below checks
directly.

`nano/asm/divide.S` itself carried a fourth bug, unrelated to the RTL: its `rem -20, 6`
vector expected `2`, but RISC-V's remainder takes the sign of the dividend, so the
correct value is `-2` (-20 = 6*(-3) + (-2)). Fixed alongside the RTL, since the whole
point of the acceptance criterion is that this exact program passes.

## The oracle

`nano/tb/nano_exec_cxxrtl.cc`, in the shape of `test/exec_tb.v`: hand-computed literal
reference values self-tested before any RTL vector runs, then 2,000 random vectors per
operation plus nineteen directed vectors (`INT_MIN / -1`, divide/remainder by zero, and
by zero with a negative dividend, negative/positive and positive/negative operands, rd
aliasing rs1, rs1 aliasing rs2, and five multiply corners) driven into the real `riscv` state machine. nano has no separate executor module the way
littlecpu does, and `nano.v` is not iverilog-elaborable (ADR-0180), so `nano/tb/nano_exec_tb.v`
wraps the real core with its bus tied off and the oracle pokes `regs`/`instr`/`cpu_state`
directly through cxxrtl's `debug_items`, walking one instruction from `decode_instr`
through retirement and reading the result back out of `regs` -- bypassing fetch/decode
the way `test/exec_tb.v` bypasses littlecpu's decoder, just through a different
mechanism. `make nano-exec-test` joins `make test`.

`nano/tb/nano_exec_probe.sh` is the forced-red direction: it reintroduces each of the
three RTL bugs above, and a `MULHSU` that sign-extends rs2, into a scratch copy of
`nano.v`, one at a time, rebuilds the oracle against it, and requires a reported
mismatch. All four are caught. It runs as a
prerequisite of `nano-exec-test`, the same relationship `formal/executor-zkt-probe.py`
has to `components_executor`.

## What does not change

nano's formal checks run entirely under `RISCV_FORMAL_ALTOPS`, which substitutes a
cheap function of the operands for the real divider's result and never enters the real
divide state at all, so shortening it from 65 to 32 iterations changes nothing there.
`make -C nano/formal remeasure-fg` reproduces `F = 12, G = 10` unchanged, and
`make -C nano/formal check`/`complete`/`dmemcheck`/`imemcheck`/`ill_e` all pass exactly
as before -- `complete`'s own depth (20) is short of the real divide loop's latency
either way, so it never observed either the broken or the fixed sequencer completing.

## Consequence

`nano/asm/EXPECTED_FAIL` is empty; `nano/asm/OBSERVED_FLOOR`'s `divide.S` line moves from
5 (the retire count at the point the old defect trapped the monitor) to 35 (a full,
passing run). `NANO_MAX_UM2` moves 60759 -> 61412, rounding the measured 61411.4 µm² up
to the next whole µm² the way ADR-0178 did -- the magnitude conversion is new logic on
`regs[rs1]`/`regs[rs2]`, read on every `DIV`/`REM`, not just inside the (now shorter) loop.

The fix moves ADR-0179's projection with it. That projection started from 60,758 µm²
local, a core whose divider did not divide; at 61,411.4 µm², with the same +11,500 for the
two unbuilt layers and the same 0.915 calibration, the finished core projects to about
66,714 µm² in the layout flow, 708 over the 66,006 line rather than 110. A correct divider
is not traded against area, so this reopens nothing here: ADR-0179's pre-committed cut
order is what answers the line if the hardened flow agrees with the projection.

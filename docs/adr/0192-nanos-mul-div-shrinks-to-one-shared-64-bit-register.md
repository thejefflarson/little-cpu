# ADR-0192: nano's mul/div shrinks to one shared 64-bit register

**Status:** Accepted · 2026-09-18

## Context

ADR-0184's work order for fitting nanocpu onto a 2×2 Tiny Tapeout tile is: (1) a latch-array
register file, (2) mul/div rebuilt around one shared 64-bit register and a 32-bit adder, keeping
M, (3) a one-read-port register file, with M itself a second permitted cut only if those three are
not enough. ADR-0189 shipped step 1: `NANO_LATCH_RF`, off by default, real and substantial (4×2
area 81,879.78 → 70,070.95 µm², routing demand 101.05% → 70.55%) but not enough alone — 4×2 still
fails `disallow_congestion=true` on localized overflow, and 2×2 does not even place. This is step
2, the shared-register mul/div ADR-0184's own ablation measured as this ticket's headroom: replacing
the unit with a non-functional stand-in read 64,285.40 µm² and 56.76% demand on 4×2, against the
81,879.78 µm² / 101.05% baseline.

Today's unit (`nano/nano.v`, before this change) keeps three 64-bit registers — `mul_div_store`,
`mul_div_x`, `mul_div_y` — plus a 7-bit counter, all fed through a 64-bit adder, subtractor and
comparator: multiply zero-extends or sign-extends both operands to 64 bits and shift-adds for 32
or 64 iterations depending on the op; divide pre-shifts the divisor into `mul_div_y`'s top half
and walks it back down over 32 iterations, comparing and subtracting the full 64-bit width each
step. About 199 flops (3×64 + 7).

## Decision

Rebuild the unit around exactly the resources ADR-0184 named: one 64-bit register holding the
product (multiply) or `{remainder, quotient}` (divide), one 32-bit register holding the
multiplicand or divisor, and one 32-bit adder/subtractor, shared between the two loops rather than
duplicated.

**Multiply** reduces every op to an unsigned 32×32→64 shift-add. MULHU never converts an operand:
it is already unsigned. MUL only reads the low 32 bits of the product, and a two's-complement
product's low bits equal an unsigned product's low bits on the same bit pattern regardless of
sign, so MUL also skips the conversion and runs on the raw operands. Only MULH and MULHSU take a
magnitude (`mul_mag_rs1`/`mul_mag_rs2`, mirroring the file's existing `div_abs_rs1`/`div_abs_rs2`)
and reapply the sign afterward (`want_neg_mul`). This means **every multiply now runs 32
iterations**, not 32 for MUL and 64 for MULH/MULHU/MULHSU — the old design's 64-bit sign-extension
made the top half meaningful for a full 64-bit shift-add, where the magnitude trick makes the top
half derivable from a 32×32 unsigned product instead. The loop shifts a 64-bit register right one
bit per cycle: the low half is the still-unconsumed multiplier, the high half accumulates a 33-bit
sum (the adder's carry-out, kept rather than dropped, is what lets the running product exceed 32
bits without a second adder).

**Divide** is textbook restoring division over the combined register, shifting `{remainder,
quotient}` left one bit per cycle: subtract the fixed 32-bit divisor from the shifted remainder,
keep the subtraction only where it did not borrow, and shift a quotient bit in behind it. The
divisor sits in the 32-bit operand register and never itself shifts, unlike the old design's
`mul_div_y`. Sign handling is unchanged from before this ticket — `div_abs_rs1`/`div_abs_rs2`
convert to magnitude, and the result is resigned afterward — and still gets `INT_MIN / -1` right
for free, since `-(-2^31)` overflows back to `2^31`'s own magnitude in two's complement, matching
the spec's wraparound answer with no special case. **Divide by zero is not free here.** The old
design's `mul_div_y` pre-shifted the divisor and left the dividend register, `mul_div_x`,
untouched by a zero-valued subtraction, so "quotient all ones, remainder unchanged" fell out of
the loop's own arithmetic. This design's remainder and quotient share one shifting register, so a
zero divisor is answered directly from the operands in the same final mux that already resigns the
result, rather than from the loop.

**The shared adder/subtractor** computes `A + (sub ? ~B + 1 : B)` at 33 bits, where `A` is either
the product's upper half (multiply) or the shifted remainder (divide) and `B` is the 32-bit
operand register; `sub` is `cpu_state == divide`. Subtracting through the adder's own carry-in
turns its carry-out into "no borrow" directly, and that carry-out — ORed with the bit the shift
pushes out of the remainder's own top, which forces the quotient bit high without a compare, since
a remainder that large already exceeds any 32-bit divisor — is the whole of `div_qbit`. `mul_div_counter`
narrows from 7 bits (needed to count to 64) to 6 (needed to count to 32).

**Register count**: 977 flip-flops → 880 in `make nano-area`'s local `synth; dfflibmap; abc`
report (−97, exactly 64+64+7 − 32−6, i.e. `mul_div_x`, `mul_div_y` and the counter's extra bit
gone, `mul_div_operand` added).

## Correctness

`nano/tb/nano_exec_cxxrtl.cc` (the differential oracle ADR-0181 built) passes unchanged: 19
directed vectors plus 2,000 random vectors per op, all eight M instructions, zero mismatches.
`nano/tb/nano_exec_probe.sh`'s forced-red mutations are rewritten against the new source (the old
ones named `mul_div_x`/`mul_div_y`, which no longer exist) and all four are caught: an inverted
divide quotient-bit condition, one fewer iteration on both loops, a divisor read raw instead of as
a magnitude, and MULHSU never negating `rs1`.

`make nano-test`, `nano-latch-test`, `nano-startup-test` and `nano-latch-startup-test` all pass,
retiring the same instruction counts as before this change on all six suite programs and the
startup check, on both simulator legs. `make -C nano/formal check` (82 checks, 80 pass, the same 2
known-fail baseline) and `complete`/`complete_cover`/`dmemcheck`/`imemcheck`/
`dmemcheck_cover`/`imemcheck_cover`/`ill_e`/`ill_e_cover`/`check-rvfi-insn-check` all pass —
unsurprising, since every one of these runs under `RISCV_FORMAL_ALTOPS`, which retires a multiply
or divide in one cycle regardless of the real loop's structure (ADR-0181 already recorded this
boundary). `make -C nano/formal remeasure-fg` reproduces F = 12, G = 10 unchanged, for the same
reason: nothing about ALTOPS's retire timing depends on the loop this ticket rebuilt.

## Measurement

**Local instrument** (`make nano-area`, `synth; dfflibmap; abc -liberty`, never merged with the
Tiny Tapeout numbers below): 61,411.40 → 57,704.09 µm² (−6.04%), against the pinned sky130hd
liberty. `NANO_MAX_UM2` was already 61412 and needs no change; this is a null on that ratchet, not
a new ceiling.

**Benchmarks, both sides under the pinned compiler in one session** (xPack `riscv-none-elf-gcc`
15.2.0-1, ADR-0190), `make nano-dhrystone`/`make nano-coremark`, core-only zero-wait-state model:
Dhrystone 503,300 cycles (2,516.5 cycles/dhrystone, 200 runs, 0.226 DMIPS/MHz) and CoreMark
9,301,580 cycles (1,860,316.0 cycles/iteration, 5 iterations, 0.538 CoreMark/MHz), **cycle-for-cycle
identical before and after this change**. This is the honest reading of a benchmark that does not
exercise what moved: MUL and DIV/REM already ran 32 iterations in both the old and new designs, and
neither benchmark's hot path uses MULH, MULHSU or MULHU, the only ops whose iteration count changed
(64 → 32). A workload that does exercise the MULH family would retire it in half the cycles this
unit used to spend; the shipped suite has no such workload to quote.

**Tiny Tapeout flow** (`nano-tt-area-selfhosted.yml`, LibreLane 3.0.14, `AREA 0`,
`disallow_congestion=true`), against ADR-0184's flip-flop baseline and ADR-0189's latch-only step 1,
both regfile builds now carrying this ticket's mul/div on both tiles:

| Tiles | Regfile | Synth area (µm²) | Placement util. | GRT total demand | Wirelength (µm) | Result | Run |
|---|---|---|---|---|---|---|---|
| 4×2 | flops (baseline, ADR-0184) | 81,879.78 | 63.638% | 101.05% | 874,050 | GRT-0116 congestion | [34848744663](https://github.com/thejefflarson/little-cpu/actions/runs/34848744663) |
| 4×2 | latches only (ADR-0189) | 70,070.95 | 54.980% | 70.55% | 653,002 | GRT-0116 congestion, closer not closed | [34925911447](https://github.com/thejefflarson/little-cpu/actions/runs/34925911447) |
| 4×2 | flops + this mul/div | 76,632.25 | 59.391% | 94.15% | 827,020 | GRT-0116 congestion (met2 103.28%) | [35043505103](https://github.com/thejefflarson/little-cpu/actions/runs/35043505103) |
| 4×2 | latches + this mul/div | *does not map under `AREA 0` — see below* | | | | ABC did not converge, killed at the 150-min job limit | [35363072917](https://github.com/thejefflarson/little-cpu/actions/runs/35363072917) |
| 4×2 | latches + this mul/div | *does not map under `AREA 0` — see below* | | | | ABC did not converge, killed at the 150-min job limit (reproduced) | [35378685507](https://github.com/thejefflarson/little-cpu/actions/runs/35378685507) |
| 2×2 | flops + this mul/div | *not yet dispatched* | | | | | |
| 4×2 | latches + this mul/div, `SYNTH_STRATEGY = AREA 2` | *not yet dispatched* | | | | | |

**Finding: `AREA 0` does not converge in ABC on the latch register file and this mul/div combined,
deterministically.** Two dispatches of the identical configuration (4×2, latches, this mul/div,
`AREA 0`, `disallow_congestion=true`), on an idle runner pool both times, both stalled at "155.
Executing ABC pass (technology mapping using ABC)" and were killed at the job's 150-minute limit
with `yosys-abc` still among the orphaned processes: run 35363072917 ran 9,042s (about 2h 27m from
ABC's start to cancellation) and run 35378685507 ran 9,056s, 14s apart — this is not a pod
contention artifact, it is the same hang twice. Three comparison points bound it to this specific
combination: ABC maps the same tile in 14s with latches and the *old* mul/div (run 34925911447), in
8s with the *new* mul/div and the flip-flop regfile (run 35043505103), and in about 3s of ABC time
locally through `make nano-area`'s own recipe (`synth; dfflibmap; abc`, `-D NANO_LATCH_RF`, same
liberty) — the same source that hangs in LibreLane's `AREA 0` strategy maps cleanly everywhere else
tried. **This is a finding about the flow's synthesis strategy on this combination, not a measurement
of its area** — see below for why the number a cancelled run prints cannot be read as one either
way.

**A run that dies before ABC still prints a `Chip area` line, and it is not a post-mapping figure.**
Its last `stat` print, before the ABC call that never returns, reports "Chip area for module ...
18,076.0864 µm²" — that total sums every cell already carrying a liberty area at that point,
sequential and combinational alike (10,869.1744 µm², 60.13%, is only the sequential share; the
rest, 7,206.912 µm², is other already-mapped cells), but the design's combinational logic is still
10,714 generic cells (mostly `$_MUX_`/`$_OR_`/`$_ANDNOT_`) that ABC has not yet turned into standard
cells, each logged "Area for cell type ... is unknown!" and contributing nothing to the total —
which is the actual reason the printed figure is so small. Against a real post-mapping figure in
the 70,000s µm² for the comparable configuration, 18,076 µm² is about a quarter of it — low by
nearly a factor of four. **18,076.0864 µm² is not this configuration's area and is not quoted as
one anywhere in this ADR.** Only a `Chip area` line from a run whose log reaches ABC is a real
figure; the give-away in a cancelled log is the "Area for cell type $_MUX_ is unknown!" block
sitting immediately above the `Chip area` line, which shows in one glance that nothing has been
mapped yet.

The 4×2, flops-only-changed row is real evidence that this ticket's cut, alone, is smaller than
ADR-0184's ablation implied: a non-functional stand-in read 64,285.40 µm² / 56.76% demand, while a
real, working unit reads 76,632.25 µm² / 94.15% — the stand-in removed the whole block rather than
replacing it with a smaller one, so it was always an upper bound on headroom, not a prediction of
this ticket's own result. Total routing demand still falls sharply against the untouched baseline
(101.05% → 94.15%, a 6.9-point drop) and wirelength by 5.4%, but 4×2 flops-only still fails
`disallow_congestion=true` on met2's 103.28%. [Remaining rows filled in as their dispatches land;
see the PR for the final table.]

## Consequences

- `nano/nano.v`'s mul/div is a straight replacement, not a define-selected variant: there is no
  path back to the three-64-bit-register design short of `git revert`.
- The local `NANO_MAX_UM2` ratchet is unaffected (61412, still not tripped); no flow-unit line
  gates nanocpu until a finished, routed core exists (ADR-0184).
- ADR-0184's step 3, a one-read-port register file, is next; M as a second permitted cut is
  reached only if steps 1–3 together are still not enough to route a 2×2 under
  `disallow_congestion=true`.
- `nano/tb/nano_exec_probe.sh`'s four mutations are now the graders for this design's shape, not
  the old one's; a future rewrite of this unit owes the same rewrite this ADR gave it.

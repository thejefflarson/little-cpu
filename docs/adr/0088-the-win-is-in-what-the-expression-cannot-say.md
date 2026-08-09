# ADR-0088: yosys already does everything derivable from the expression; the win is in what the expression cannot say

**Status:** Accepted · 2026-08-09 · *Eleven behaviour-preserving edits to `rtl/`. This is an AREA
result. The period moved less than the churn band and is a null in both directions.*

## Context

This ticket was filed on a congestion hypothesis: the shipping SoC places at 90% of the up5k, and
`soc/compare/` measured this core at 96% of an hx8k against VexRiscv's 31%
([ADR-0086](0086-both-cores-in-one-harness-and-the-gap-is-the-fetch-loop.md)), which noted that
"congestion is pressure on the longer path and not on the shorter one" and that "controlling for it
would need a bigger ice40 and there is not one". Free the cells, the argument went, and the router
gets room to shorten the fetch loop.

**A ballast experiment disproved that before any of this landed.** Padding the design with logic
that does nothing, from 77% to 95% occupancy, moves the period not at all. Occupancy on this part is
not what sets the period; the fetch loop is, exactly where ADR-0076, ADR-0078 and ADR-0087 already
left it. The cells came out anyway, for a reason unrelated to the hypothesis, and that reason is
what this ADR is for.

## Decision

**Where synthesis is weak is not arithmetic — it is facts that live outside the expression.**

yosys and ABC already do everything derivable from the text of an expression: dead bits, common
subexpressions, duplicate adders, constant folding, the lot. Rewriting an expression that says the
same thing in the same terms buys nothing, and this ADR records two measurements that show it
buying nothing.

What they cannot do is use a fact the expression does not state:

- a parameter is a power of two, so a magnitude compare is a bit reduction;
- a window is aligned, so membership is an equality on the high bits and the low bits survive the
  subtraction untouched;
- a reversal is wiring, so one right shifter is three shifters;
- an address bit is provably zero, because a trap in decode guarantees it;
- a comparison and the subtraction beside it are the same carry chain read twice.

**Every one of the eleven changes that paid is in that second category, and none of them is a
cleverer spelling of arithmetic.**

## The eleven

| # | File | Fact synthesis could not derive |
|---|---|---|
| 1 | `imemory.v` | `in_range`/`in_range2`: `< ROM_WORDS` on a power of two is a reduction on the bits above the ROM |
| 2 | `imemory.v` | `text_range`: the same, on the data port |
| 3 | `memory.v` | an aligned power-of-two window is an equality on the high bits, not a subtraction and two magnitude compares |
| 4 | `timer.v` | the same, and an aligned `BASE` leaves the low two bits of the address untouched, so the register select needs no subtraction |
| 5 | `decoder.v` | misalignment needs two bits of a sum, and a sum's low two bits depend only on the operands' low two bits |
| 6 | `decoder.v` | `jal` and a taken branch are one arm: at 32 bits the signed and unsigned sums have identical bits |
| 7 | `decoder.v` | six branch comparators are one subtraction — borrow is unsigned-lt, signed differs only when the sign bits disagree, equal is difference-zero |
| 8 | `executor.v` | the divider's `x >= y` then `x - y` is one subtraction whose borrow answers both |
| 9 | `executor.v` | `mul_div_counter > 0` is a reduction |
| 10 | `executor.v` | three barrel shifters are one right shifter: a left shift is a right shift with both ends reversed, and a reversal is wiring |
| 11 | `accessor.v` | twelve 32-bit load selects over four lanes are one shift down to the addressed byte and one extension |

**Measured individually, every one of the eleven is inside the ±50 cell churn band. Only the group
clears it.** That is the whole reason they land together: nothing here would survive being proposed
on its own, and a reviewer measuring any single edit would correctly call it a null.

## The measurement

`make fit` is the core alone; `make soc-timing` is the SoC that places; the period is eight seeds a
side through `soc/timing_sweep.sh`. Base is `a86f5d6`, measured in a clean export of that commit
with the same toolchain in the same sitting.

| instrument | base | this | delta |
|---|---|---|---|
| `make fit` (up5k, core alone, memories external) | 3958 | **3874** | **−84 (−2.1%)** |
| `make soc-timing` placed `ICESTORM_LC` (up5k SoC) | 4769 | **4600** | **−169 (−3.5%)** |
| `make compare-timing` placed `ICESTORM_LC` (hx8k) | 7400 | **7211** | **−189 (−2.6%)** |
| `make compare-timing` core alone, `SB_LUT4` (hx8k) | 6611 | **6535** | −76 (−1.1%) |

The SoC count is seed-stable across placements of one netlist, which is what a packing number
should be, and is the check that this is a netlist result and not a placement sample.

**`make fit` cannot see changes 1 to 4 at all.** Its top is `littlecpu` with the memories external,
so `imemory.v`, `memory.v` and `timer.v` are not in that netlist. The −84 there is changes 5 to 11
only; the −169 on the SoC is all eleven. An instrument that cannot observe the thing being measured
is worth saying out loud, because the two numbers are otherwise read as one — and here the gap
between them, 85 cells, is the three memories.

### The period does not move

up5k, eight seeds each, `SOC_SEEDS='default 1 2 3 4 5 6 7'`:

| | sorted, ns | worst | median | best |
|---|---|---|---|---|
| base | 76.73 76.97 77.14 77.77 78.40 79.58 81.86 82.00 | 82.00 | 78.09 | 76.73 |
| this | 75.67 77.17 77.53 78.34 78.38 78.59 79.85 81.61 | **81.61** | **78.36** | 75.67 |

**−0.5% on the worst, +0.3% on the median, −1.4% on the best** — against a ~3.6% edit-churn band and
a 1–2% placement spread. That is a null in both directions and must be read as one: the cells came
out and the period did not follow, in either direction. Every one of the eight placements clears the
12 MHz requirement, the worst at 12.25 MHz against `SOC_MIN_MHZ`'s 12.0. `SOC_MIN_MHZ` is untouched.

`make cycles` is unchanged to the cycle: 32 624 cycles, 15 654 retires, CPI 2.08, and the same
split across the six stall reasons. Nothing here touches a stall reason, a stage length or the
scoreboard, so F and G are not re-measured and the BMC depth table does not move.

hx8k, one placement a side, is not a distribution and is quoted only for the cell counts above.

## The counter-evidence, which is what makes the rule falsifiable

Two edits that a reader would expect to pay, measured the same way and against this branch rather
than against base:

**Narrowing `mul_div_store` from 64 bits to 32, where only `[31:0]` is ever read: `make fit` 3874 →
3863, −11 cells.** Inside the churn band; a null. Deleting bits nothing reads buys nothing, because
DCE had already deleted them — the register was never 64 bits in the netlist. This is the same
finding CLAUDE.md already records for `mul_div_counter`, where narrowing *cost* 37 cells, and it
generalises: hand-narrowing a datapath is arguing with a pass that has already run.

**Flattening the `next_pc` priority chain into a parallel mux over pre-computed exclusive selects:
`make fit` 3874 → 3896 (+22, a null), SoC placed −53 cells (also a null), and the period over four
seeds 79.96 · 80.89 · 81.02 · 82.02 against 74.84 · 75.41 · 75.54 · 77.43 for the same netlist
without it.** Seed for seed that is **+3.3%, +7.3%, +8.1% and +8.8% slower — outside the churn band
at three of the four** — for cells inside it. It is the routing-dominated fabric doing what CLAUDE.md
already says it does, wide flat muxes route worse than chains, and it is the clearest case here of an
edit that reads as an optimisation, that ABC had already considered, and that is worse.

Both are the same lesson from the other side: **an edit that restates arithmetic in the same terms
is arguing with a pass; an edit that supplies a fact from outside the expression is not.**

## Four alignment checks, because these facts are now load-bearing

Changes 1 to 4 make "the window is an aligned power of two" a correctness precondition where before
it was a performance question. At a non-power-of-two `ROM_WORDS` the reduction admits the addresses
between the top of the ROM and the next power of two, which index off the end of the banks instead
of reading zero — so a PC that runs off the end aliases real code rather than trapping. At an
unaligned `BASE` the equality names a window the memory does not occupy, and the timer's register
select picks the wrong register. **The old spelling was merely slow at those parameters; the new one
is silently wrong**, and a comment is not a check.

So each of `rtl/imemory.v`, `rtl/memory.v` and `rtl/timer.v` carries an elaboration `$fatal` in a
generate-if beside the test it guards. Both frontends stop on it: iverilog reports it during
elaboration with the message, yosys reports `FATAL` from `read_verilog`.

`test/window_test.sh` is what says those checks can fail. It elaborates each module at a bad
parameter and requires a non-zero status with the right diagnostic, plus a control at the shipping
parameters that must elaborate clean — fourteen elaborations, seven cases across two frontends. It
hangs off `make test`. Deleting any one `$fatal` turns the matching case red, which is how it was
demonstrated rather than assumed.

## Five formal assertions, because the arithmetic no longer states itself

Changes 5, 7, 8 and 10 replace operators with bit tests. The operator was the specification; a bit
test is not. So the identities are asserted in the two `FORMAL` blocks, against the operators they
replaced:

- `rtl/decoder.v`: `cmp_eq`, `cmp_ltu` and `cmp_lt` against `==`, `<` and signed `<`; and
  `mem_addr_low == mem_addr_calc[1:0]`.
- `rtl/executor.v`: `alu_sub`, `alu_ltu` and `alu_lt` the same way; `shift_rev` against `<<`,
  `shift_res` against `>>` and signed `>>>` under their own flags; and the divider's borrow and
  difference against `<` and `-`.

Every reference is a self-determined statement over its own named signed nets, never an arm of a
conditional — the rule that already exists here because IEEE 1800 sign context has produced two
wrong oracles in this repository. These are discharged by `components_decoder` and
`components_executor`, both `mode prove`. Each set has a demonstrated red direction: dropping the
sign-bit correction from `cmp_lt` fails `components_decoder` in the base case, and reversing the
shifter's output the wrong way fails `components_executor`.

That is the trade this change makes explicit. **It removes an operator's guarantee from the RTL and
puts it in an assertion**, which is only an improvement while the assertion runs — so it is stated
here, and the red directions above are the evidence that it does.

## What was verified

| leg | verdict |
|---|---|
| `make test` | 62/62, `test/EXPECTED_FAIL` exact |
| `make test-units` | 9/9, including 14 new load-unpack vectors in `test/accessor_tb.v` |
| `make probe-gates` | 188 probes, clean (via `make test`) |
| `make window-test` | 14 elaborations, each rejected or accepted as required |
| `make cycles` | 32 624 cycles, CPI 2.08, unchanged |
| `make -C formal check` | 85 checks, 85 pass, `EXPECTED_FAIL` and `EXPECTED_CHECKS` exact |
| `components_executor` | successful proof by k-induction |
| `components_decoder` | successful proof by k-induction |
| `components_pcloop` | successful proof by k-induction |
| `components_traps` | successful proof by k-induction |
| `complete_cover` | PASS |
| `nonperturbation` | PASS |
| `make cosim-suite` | 60/62 agreed, byte-identical to the same run on `a86f5d6` |
| `make lint` | svlint clean in both passes |
| `make elaborate-strict` | clean, no warnings |

`make -C formal complete` did not return a verdict here, and not because of this change: it fails
identically on an unmodified export of `a86f5d6` with `ERROR: Command syntax error` on `abc -g AND
-fast`. That is the local Homebrew yosys 0.68+post, whose bundled abc does not take that option; the
`formal` CI job runs the pinned OSS CAD Suite and `complete` is not in that job. Recorded rather than
worked around.

`make cosim-suite` is carried because ADR-0032 keeps it for exactly this: it reads the core's real
`regs_a` and no `rvfi_*` signal, and eleven changes to arithmetic and the load path is the case it
was kept for.

## Consequences

- **A rule that can be applied without measuring first, and a null that can be recognised without
  measuring at all.** Before proposing an edit, ask what fact it gives synthesis that the expression
  did not already contain. If the answer is "none — it is the same arithmetic, spelled better", it
  is a null; ABC has already had it. If the answer names something outside the expression — a
  parameter's shape, an alignment, a proven-zero bit, wiring — it is worth an instrument.
- **The congestion hypothesis this ticket was filed on is retired.** 77% to 95% occupancy moves the
  period not at all on this part. Freeing cells is worth doing for headroom on the up5k's 5280, and
  it is not a route to Fmax. The levers for that remain the three ADR-0087 ranked.
- **This is an area change and must not be quoted as a speed one.** −169 placed cells on the SoC and
  −84 on the core alone are outside their bands; +0.3% of median period is well inside its band and
  is a null in both directions. The 12 MHz requirement holds at every one of eight placements, with
  2.1% of margin at the worst of them, and `SOC_MIN_MHZ` did not move.
- **Three modules now refuse to elaborate at a parameter shape they are not valid for**, and the
  refusal is exercised on every `make test`. `rtl/memory.v`'s `RAM_WORDS` can still double to fill
  the up5k's other two SPRAMs, because 32768 is a power of two and `0x0001_0000` is aligned to it;
  a non-power-of-two size is now a build failure rather than a silent aliasing bug.
- **`make fit` is not a complete instrument for this repository's memories.** It never was, and
  nothing said so. Anything touching `imemory.v`, `memory.v` or `timer.v` has to be read on
  `make soc-timing`, which is the only flow that has them in the netlist.

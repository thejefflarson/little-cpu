# 0151 — The divider's carry chains read their registers uninverted

Status: accepted · a spelling change to `rtl/executor.v` with identical behaviour — **−116 packed
`ICESTORM_LC` on `make fit`, −121 `SB_LUT4`, `SB_CARRY` and the flop census unmoved**, and a period
that is a null. Adds no stall reason, lengthens no stage, widens no scoreboard, so F and G are
untouched at 6. Extends [ADR-0090](0090-the-executor-carries-half-the-registers-it-had.md)'s
question of `rtl/executor.v` with one the expression still could not state.

## Context

An ice40 logic cell's carry unit reads its two addends off the cell's own pins. A LUT sitting in
that cell can feed the sum, but nothing sits between a pin and the carry chain — so an addend that
arrives inverted has to be inverted *somewhere*, and the only place available is a LUT of its own,
one per bit. A constant addend costs nothing at all: `all ones` is wiring.

The divider had three chains reading inverted registers:

- `div_x` and `div_y` negate a magnitude with `-rs1` / `-rs2`. Verilog's unary minus is `0 - x`,
  which yosys builds as `~x + 1` — the register inverted into the chain.
- `rem_sub = rem_shifted - {1'b0, div_divisor}` is the loop's compare-and-subtract, run once per
  iteration. `a - b` is `a + ~b + 1`, so `div_divisor`'s 32 register outputs are inverted into the
  chain there too.

[ADR-0088](0088-the-win-is-in-what-the-expression-cannot-say.md)'s rule says what makes this worth
asking: yosys and ABC already do everything derivable from the expression, and what they cannot use
is a fact from outside it. "The addend the fabric wants is the complement" is such a fact. Nothing
in `a - b` says which of the two operands is cheaper to hold complemented, because at the RTL level
neither is.

## Decision

Three edits, one idea.

**`-x` becomes `~(x - 1)`.** The identity is exact in two's complement for every 32-bit `x`,
`0x80000000` and `0` included: `~(x - 1) = -(x - 1) - 1 = -x`. What changes is where the inversion
sits — `x - 1` is `x + 0xFFFFFFFE + 1`, a chain whose addends are a register straight off the pins
and a constant, and the outer `~` folds into the ternary mux that already selects between the
magnitude and the raw operand.

**The divisor register holds its complement.** `div_divisor_n` is the flip-flop; `div_divisor`
becomes `assign div_divisor = ~div_divisor_n`, so **every existing reader is unchanged** — the
`RISCV_FORMAL_ALTOPS` operand alias, the loop bound in the comments, and all five assertions in the
`ifdef FORMAL` block still name `div_divisor` and still mean the divisor. The load complements once
(`div_divisor_n <= ~div_y`), where the inversion folds into the magnitude select that is already a
mux.

**The subtraction is written out.** `rem_sub = rem_shifted + {1'b1, div_divisor_n} + 33'd1` is the
same value as before by the same `a + ~b + 1` yosys was building anyway, except that the complement
is now what the register holds, so the chain reads it directly.

`div_divisor` reads all-ones out of reset rather than zero. Nothing consumes it outside the divide
state — the assertions that read it are all guarded on `state == divide`, and the one that is not
(`rem_sub[32]` is the borrow) is an identity that holds for any operands.

## The area, on `dba70e9` with a local Homebrew toolchain

yosys 0.68+post (`c12172fb`), nextpnr-ice40 and `icetime` from the same Homebrew install,
`SOC_PROG` at its default `datainit.c`. `make fit` synthesises and places deterministically — both
counts below reproduced on a second run.

| `make fit` (core alone, `littlecpu`) | before | after | Δ |
|---|---|---|---|
| packed `ICESTORM_LC` | 4109 | **3993** | **−116** |
| `SB_LUT4` | 3798 | 3677 | −121 |
| `SB_CARRY` | 579 | 579 | 0 |
| `SB_DFF` / `SB_DFFESR` / `SB_DFFSR` | 76 / 612 / 130 | 76 / 612 / 130 | 0 |
| `ICESTORM_DSP` | 4 | 4 | 0 |

**−116 is 2.3× the ±50 churn band**, and the shape of it is the argument: the carry census does not
move, so no chain was added or deleted — what came out is 121 LUTs that existed only to invert
register outputs on their way onto carry pins. Three chains at 32 bits is 96, and the rest is the
mux restructuring around them.

On the SoC's own top the same change is **4943 → 4910 placed cells, −33**, which is inside the band
and is the top-dependence [ADR-0094](0094-the-compressed-decode-is-already-mapped-and-the-block-is-closed.md)
measured: ABC maps one cone differently for what surrounds it. Read the `fit` number as the size of
the idea and the SoC number as what the SoC got.

`FIT_MAX_LC` stays at 4219 and `FIT_LAST_LC` at 4097. Both are the `fit` **job's** numbers and this
was measured locally, where the two instruments are known to disagree by as much as the band; a
re-derivation is owed to a CI run and not to this one.

## The period, at sixteen placements paired per seed

Both sides swept with `soc/baseline_sweep.sh` at the same sixteen seeds, one tree apart, one
toolchain, `datainit.c`. `make netlist-digest` moved (`d802efa3…` → `81914039…`), which is what
made the sweep owed.

| | before | after |
|---|---|---|
| worst of 16 | 82.89 ns — **12.06 MHz** | 80.15 ns — **12.48 MHz** |
| median | 77.00 ns — 12.99 MHz | 78.72 ns — 12.70 MHz |
| best | 74.79 ns — 13.37 MHz | 75.71 ns — 13.21 MHz |
| best-to-worst spread | 10.8% | 5.9% |
| under 12.00 MHz | **0 of 16** | **0 of 16** |
| placed `ICESTORM_LC` | 4943 | 4910 |
| LUT levels · carry hops | 19–22 · 2–31 | 21–23 · 0–4 |

**Median of the per-seed deltas: +1.53%. Twelve of sixteen seeds slower, two-sided sign test
p = 0.077.** That is inside this part's ~3.6% edit-churn band, so it is a null in both directions —
neither a cost to record nor a win to claim. The requirement holds at sixteen of sixteen on both
sides, and the worst placement moves the other way, 12.06 → 12.48 MHz.

**Do not read the tail as bought.** ADR-0121's ballast experiment is the shape to compare against:
+2.03% of median with 13 of 16 slower and the worst placement 1.1% the other way, from 352 cells of
logic nothing reads. Occupancy does not set the tail on this part, the spread here is a sample of a
distribution the previous sweep took a different draw from, and a worst-of-16 against worst-of-16 is
exactly the comparison this repo's own convention warns throws the pairing away. What the table
supports is that the requirement is met with margin on both sides and nothing measurable moved.

The paired column is where the honest reading is, and it says: whatever this bought in cells, it did
not convert into period. That is [ADR-0121](0121-the-occupancy-prediction-is-registered-and-the-placement-spread-is-corrected.md)'s
result again, from the other direction — freeing cells buys headroom on the 5280 and not margin
against 12 MHz.

## What grades it

Nothing here changes behaviour, so no test fails without the change and none can be written that
does. What the change needs is graders that would notice if the identity were wrong, and they exist
and were run:

- **`make -C formal components_executor`** — the k-induction proof over the divider's arithmetic.
  `assert(div_divisor == div_mag_y)` in the divide state is the round trip through the complemented
  register, stated as a property rather than trusted; `assert(rem_sub[32] == (rem_shifted < {1'b0,
  div_divisor}))` and the loop invariant `div_quot_done * div_divisor + div_rem == div_mag_x_done`
  are the rewritten subtraction. Passes.
- **The red direction was run.** Loading the divisor uncomplemented — `div_divisor_n <= div_y`, one
  character from the shipping text — makes that proof fail at `executor.v:449`, the divisor
  assertion's own line, on the basecase leg. The assertion is not decorative.
- **`test/exec_tb.v`** — directed vectors at `div INT_MIN/-1` (`0x80000000 / 0xffffffff`), both
  divide-by-zero forms and negative operands both ways round, plus randomized vectors. `0x80000000`
  is the one input where `~(x - 1)` and `-x` could plausibly have been made to differ, and it is
  covered by name.
- **`make test`** — 74/74, failure list matching `test/EXPECTED_FAIL`.
- **`make cycles`** — byte-identical to the clean tree, table and both locality counters. The
  divider's latency, its short-circuits and `stalled` are untouched, so no cycle change was possible
  and none happened.

## Consequences

- **A carry chain's addend polarity is a fact from outside the expression, and it is worth 116
  cells here.** ADR-0088's list did not have this entry; it does now. The general form: where a
  design holds a value in a register and every consumer of it feeds a subtractor, hold the
  complement and let the one reader that wants the true value take a `~` it can fold into a mux.
- **`SB_CARRY` unmoved beside `SB_LUT4` down is the signature to look for.** A saving that moves
  both is a chain deleted; a saving that moves only the LUTs, at a multiple of 32, is polarity.
  That is a cheap thing to check on any candidate in this class before building it.
- **`rtl/executor.v` is not closed for area by this.** ADR-0090 took three facts out of this module
  and this is a fourth, found by a different question than the one that found those. Neither
  question has been asked of the multiplier's correction terms or the result muxes for polarity.
- The divisor register's name now says what it holds (`div_divisor_n`) and the derived signal keeps
  the name every reader knew, which is what kept this a three-line change in the always block rather
  than a rewrite of the formal section.

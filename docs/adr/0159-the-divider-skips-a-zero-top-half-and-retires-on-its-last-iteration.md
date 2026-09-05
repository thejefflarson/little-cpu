# 0159 — The divider skips a zero top half and retires on its last iteration

Status: accepted · two changes to `rtl/executor.v`'s divide FSM, both off the fetch loop and both
bit-identical in result — **`make cycles` 39 096 → 38 746 suite cycles, −350 (−0.90%), every one of
them in the DIVIDER column and no other column moved by one**. They cost **+11 packed
`ICESTORM_LC` on `make fit`** — a null inside the ±50 band, and **60 cells LESS than the skip
half costs on its own**, which is the number to read. The period is a null at sixteen paired placements with
0 of 16 under 12.00 MHz on either side. Adds no stall reason, lengthens no stage, widens no
scoreboard, so F and G are untouched at 6. Follows
[ADR-0151](0151-the-dividers-carry-chains-read-their-registers-uninverted.md) on the same block,
from the other direction: that one bought cells at no cycles, this one buys cycles at cells.

## Context

The divider is a 32-iteration restoring loop. `div_quot` is loaded with the dividend's magnitude,
each iteration shifts one dividend bit out of the top into `rem_shifted`, subtracts the divisor, and
shifts the quotient bit in at the bottom. `mul_div_counter` counts the iterations down; the FSM
leaves the divide state when it reads zero.

Two cycles in that are spent on nothing.

**The first sixteen iterations of a small dividend are pure shifts.** With `div_rem` starting at
zero and the dividend's top half zero, `rem_shifted` is zero for sixteen iterations running, the
subtract borrows every time (the divisor is nonzero — `rs2 == 0` short-circuits in `init`), the
quotient bit shifted in is zero, and `div_rem` stays zero. Sixteen cycles to move the dividend
sixteen places left, which a load can do with wires.

**The cycle after the last iteration reads back what the last iteration wrote.** The FSM spends a
33rd cycle in the divide state with the counter at zero, doing nothing but selecting `div_quot` or
`div_rem` into `out.rd_data` and raising `out.valid`. The values it selects are available a cycle
earlier, as the iteration's own next values.

Neither is a fetch-loop question. `rtl/executor.v` publishes `stalled` and the decoder holds the
issued instruction; nothing here is read by `next_pc`.

## Decision

**Load the loop sixteen iterations in when the dividend's top half is zero.** `div_skip` is
`div_x[31:16] == 16'b0` — read off the magnitude the load arm already computes, so a negative
dividend is judged on `-rs1` and not on `rs1`. When it holds, the load writes
`mul_div_counter <= 16` and `div_quot <= {div_x[15:0], 16'b0}` instead of 32 and `div_x`. That is
exactly the state the sixteen shift iterations would have left, `div_rem` included, so every result
is unchanged and only the latency moves.

**Retire on the last iteration's edge.** `div_quot_next` and `div_rem_next` name what the iteration
computes; the divide state assigns them unconditionally and finishes when the counter reads **one**
rather than spending a further cycle at zero. `div_result_mag` selects between the two `_next`
values, so the result is formed from the iteration on the edge that performs it.

**The counter can no longer read zero in the divide state, and that is asserted.** The step
decrements unconditionally, so a state with `state == divide && mul_div_counter == 0` would wrap the
counter to 127 and break the invariant that reads it as an exact count of the iterations left. It is
unreachable in the design — a divide is loaded at 16 or 32 and leaves at 1 — and k-induction has to
be told, so `rtl/executor.v` states `assert(mul_div_counter != 0)` beside the bound that was already
there. `make -C formal components_executor` proves both.

**A third latency class for `DIV`/`REM` is inside the Zkt claim rather than a new hole in it.** The
claim's listed set — RV32I arithmetic, logical and shift, the four multiplies, the arithmetic C
encodings — excludes `DIV`/`REM` precisely because their timing already depends on operand values
(`rs2 == 0` and `INT_MIN / -1` each finish in one cycle). "16 when the dividend's top half is zero"
joins that list. `test/zkt_isolation_test.py` is unaffected: it walks the **decoder's** netlist from
`reg_rs1`, `reg_rs2` and `executor_out.rd_data`, and `divider_stall` is a one-bit decoder input it
neither seeds nor reaches.

## The cycles, on `main` with a local Homebrew toolchain

yosys 0.68+post (`c12172fb`), nextpnr-ice40 and `icetime` from the same install,
`riscv64-elf-gcc`, `SOC_PROG` at its default `datainit.c`.

| `make cycles` (75 programs) | `main` | after | Δ |
|---|---|---|---|
| suite cycles | 39 096 | **38 746** | **−350, −0.90%** |
| DIVIDER | 990 | **640** | **−350** |
| ISSUE · ATOMIC · HAZARD · SERIAL · OPERAND · FETCH · BUS · REGION | 22 074 · 65 · 12 121 · 1 485 · 1 218 · 9 · 0 · 1 134 | identical | 0 |
| CPI | 1.78 | 1.77 | |

The whole delta is in one column and the decomposition is exact. The suite performs **30 real
divides** — 990 / 33 — of which **20 have a dividend magnitude with a zero top half**. After the
change: 20 × 16 + 10 × 32 = 640. The skip saves 16 cycles on 20 divides (320) and the early retire
saves 1 on all 30 (30).

**Those two happen to add, and the reason is that they act on different cycles**, not a general
rule: the skip removes iterations and the early retire removes the capture cycle, and no divide has
its capture cycle removed twice. The cell costs do **not** add, which is the next section.

## The area, and the pair costs less than either part

`make fit` synthesises and places deterministically; every count below is one run of a distinct
netlist, and the two surprising rows were re-run twice and reproduced exactly.

| `make fit` (core alone, `littlecpu`) | packed `ICESTORM_LC` | Δ vs `main` | `SB_LUT4` |
|---|---|---|---|
| `main` | 4059 | — | 3740 |
| skip only | 4130 | **+71** | 3815 |
| early retire only | 4071 | +12 | 3756 |
| **both, as shipped** | **4070** | **+11** | 3757 |

`SB_CARRY` is 579 and `SB_DFF`/`SB_DFFESR`/`SB_DFFSR` are 76/612/131 on all four netlists. Nothing
gained a chain or a flop; the whole difference is 17 LUTs.

**The pair costs 60 cells LESS than the skip alone, and +11 is a null inside the ±50 band.** That
is not a rounding artefact — both rows reproduce exactly on re-run, because `make fit` is
deterministic for a given netlist. It is the strongest form of the reading this ADR already had:
**a cell count is a property of the netlist ABC mapped, not of the idea.** The two edits share the
divide state's own logic — the skip widens the mux feeding `div_quot`'s load, the early retire adds
the `_next` nets and a compare to the same block — so the cone they both touch is mapped once, for
both, and neither the sum nor the maximum of two standalone measurements has any claim on it. This
is [ADR-0097](0097-the-decode-stack-pays-only-in-the-fetch-loop-and-that-is-where-it-cannot.md)'s
spelling-dependence at its sharpest: there, two texts of one idea differed by 44 cells; here, one
text of two ideas is 60 cells cheaper than one of its own halves.

**Read the +11, not the +71.** The shipping netlist is the one in the last row, and the two
intermediate rows exist only to show that decomposing this change's cost is meaningless. A reviewer
who prices the skip on its own and multiplies is 6× out.

**These numbers replace an earlier set and the earlier set is not quoted, because it did not
travel.** Measured against the pre-forwarding tree the same four netlists read 3986 / 4013 / 4005 /
4061 — the pair at **+75**, argued there as "outside the band, so a cost rather than noise", and
larger than either half rather than smaller. Every one of those readings is wrong for the tree this
ships on. What moved underneath was [ADR-0083](0083-the-forwarding-network-is-priced-and-declined-on-the-margin.md)'s
executor-only forwarding landing in between, which changes what surrounds this cone. The cycle
figures survived that move unchanged and the area figures did not, which is the asymmetry worth
carrying away.

On the SoC's own top the pair is **4909 → 4978 placed cells, +69**, identical at every one of
sixteen seeds — larger than the core's own +11, which is ADR-0094's top-dependence.

**The branch is a net cell increase of 11 against `main`**, and `main` already carries the
ADR-0151 respell that took 102 cells out. `FIT_MAX_LC` is 4219 and `make fit`'s ratchet passes with
149 cells of budget.

## The period, at sixteen placements paired per seed

The netlist moved, so the sweep was owed. Both sides run through `soc/baseline_sweep.sh` at the
same sixteen seeds, one toolchain, `datainit.c`, each from its own tree.

| | `main` | after |
|---|---|---|
| worst of 16 | 82.93 ns — **12.06 MHz** | 81.86 ns — **12.22 MHz** |
| median | 80.43 ns — 12.43 MHz | 79.55 ns — 12.57 MHz |
| under 12.00 MHz | **0 of 16** | **0 of 16** |
| placed `ICESTORM_LC` | 4909 | 4978 |

**Median of the per-seed deltas: −1.56%. Five of sixteen seeds slower, two-sided sign test
p = 0.210.** That is a null — well inside this part's ~3.6% edit-churn band, and not distinguishable
from a coin flip on the sign. The requirement holds at sixteen of sixteen on both sides.

**The worst placement moves 12.06 → 12.22 MHz and that is NOT claimed as bought.** ADR-0121 measured
the same shape out of pure ballast — 352 cells of logic nothing reads moved the median 2% with the
worst placement going the other way — so a tail that improves alongside a change that adds cells is
a draw from a distribution, not a mechanism. The honest statement is that nothing measurable moved.

**This is the second time this pair was swept, and the sign of the median flipped between them.**
Against the pre-forwarding tree the same two netlists read **+0.75%, 10 of 16 slower, p = 0.454**;
here they read −1.56%, 5 of 16 slower, p = 0.210. Both are nulls and neither contradicts the other —
which is the point. A median inside the churn band carries no sign worth reporting, and quoting
either number as though it had one would be reading noise. What survived both sweeps is the part
that matters: **0 of 16 placements under 12.00 MHz on either side, twice.**

## The interrupt response was re-measured, and the old figure did not reproduce

The machine timer is taken on a cycle that would otherwise have issued, so its worst-case response
is the longest run of consecutive stalled cycles the core can produce, and the divider sets it.
Removing the capture cycle should remove one cycle from that, and it does — but the number this
tree starts from is not the 33 that has been quoted since
[ADR-0082](0082-the-machine-timer-interrupt-is-taken-at-a-decode-boundary.md).

Measured with ADR-0082's own instrument: `interrupt_pending` is forced high at a swept cycle and the
gap to `trap_entry` is read out of the elaborated design every cycle, over a program of back-to-back
divides with a load, a CSR read and a `fence.i` among them, both trees compiled by iverilog from the
same harness and the same ROM image. The response is the length of the stall run remaining when the
interrupt arrives, which is what ADR-0082 measured between the same two signals.

| workload, arming cycle swept 1–500 | before | after |
|---|---|---|
| a divide with nothing behind it | 34 | **33** |
| divides, a load, a CSR read and a `fence.i` | 35 | **34** |
| the same with a load, a store and a dependent load each placed directly behind a divide | **36** | **35** |

**So the figure moves 36 → 35, 3.00 µs → 2.92 µs at 12 MHz, and the change is worth exactly the one
cycle it was predicted to be worth.** What did not hold is the baseline: ADR-0082's 33 does not
reproduce on this tree with the change absent. Its decomposition — "the 32-cycle divide plus the
operand-fetch cycle behind it" — is one short of what the divide state actually occupied here (33
cycles, which `make cycles` charges as 990 over 30 divides), and the load/store region wait
[ADR-0129](0129-the-region-answer-off-the-fetch-loop-costs-cycles-and-no-clock.md) added afterwards
puts another cycle behind a load that follows a divide. Neither is this change's doing, and the
number was carried forward through both.

**`CLAUDE.md` is deliberately left at 33, and this table was NOT re-taken when the branch was
rebased onto `a866f30`.** Every other figure here was — all four `make fit` netlists, the cycle
decomposition and the sixteen paired placements reproduce exactly — but ADR-0082's sweep is a
one-off harness that was never committed, so there is nothing in the tree to re-run. A rulebook
number is not worth stating on a measurement that cannot be reproduced, so the re-take is owed
before `CLAUDE.md` moves. Read the three rows as a measurement on the tree this ADR was written
against, which is what an ADR is.

This is a **measured maximum over the programs swept**, exactly as ADR-0082's was, and not a bound.
ADR-0082 says why a literal bound cannot be proved in `formal/traps.sv` without four assumptions
this repo would then owe a discharge for, and nothing here changes that.

## What grades it

- **`test/exec_tb.v` grades the latency per divide, and it had to move.** It counted the cycles in
  the divide state and required 33; there is no single number any more. It now predicts the count
  from the operands with a reference of its own — `ref_div_cycles`, which returns 16 or 32 for a
  real divide and **0** for the short-circuits and for everything that is not a divide, so a visit
  to the divide state the bench did not predict is red rather than unmeasured — and the monitor
  snapshots that prediction on the divide's first cycle so a later call cannot overwrite a check
  still pending. The reference is self-tested against eight hand-computed cases before any RTL
  vector runs, on both sides of the boundary (`0x0000ffff` is 16, `0x00010000` is 32) and on both
  sides of the magnitude conversion (`-65535` is 16, `-65536` is 32). A new
  `check_div_lengths` requires both loop lengths to have run: a `$random` dividend has a zero top
  half about once in 65 536, so the 16-iteration arm is reached by the directed vectors at the
  bottom of the run and by nothing else reliably.
- **Both red directions were run.** Removing the skip and leaving the bench alone reports
  `TIMING MISMATCH: divider completed in 32 cycles, expected 16`; putting the retire back a cycle
  reports `completed in 33 cycles, expected 32`. The bench fails for the reason it was written.
- **`make -C formal components_executor`** — successful proof by k-induction, with
  `formal/executor-zkt-probe.py` red first at `executor.v:316`, the MUL constant-latency assertion's
  own line. The loop invariant `div_quot_done * div_divisor + div_rem == div_mag_x_done` is what
  says the skipped load is the right state: at the load `div_done` is 16, `div_quot`'s low sixteen
  bits are zero and `div_mag_x >> 16` is zero, so the identity holds with nothing done.
- **The divide's result assertions changed failure mode, for the better, and the comment says so.**
  The proof cap (`div_mag_x <= 0xff`) puts every proof divide on the 16-iteration path, which
  reaches the completion at step 19 of `mode prove`'s 20 basecase steps. Measured: mutating
  `divu_ref` now reports `FAIL` at that assertion's own line on the basecase leg, where a
  32-iteration divide left it out of basecase reach and reported the induction-only `UNKNOWN
  (rc=4)`.
- **`make test`** — 74/74, failure list matching `test/EXPECTED_FAIL`. **`make test-units`** — all
  thirteen benches. **`make mutation-check`** — 11 mutations, each caught by exactly its paired
  detectors.

## What this does not establish

- **Neither change moves DMIPS, and the measurement is exact rather than argued.** `make dhrystone`
  reports **1 464 021 timed cycles and 0.777 DMIPS/MHz on both trees** — the same digits, not a
  rounding — while the whole run goes 1 756 248 → 1 756 077 cycles and its DIVIDER column goes
  363 → 192. The 171 cycles this saves are **entirely outside the window the program times with
  `mcycle`**: Dhrystone's divides are in the report-formatting tail. So 7.97 DMIPS at 12 MHz
  stands, unchanged. The `.S` suite's 30 divides are where the whole −350 lives, and `make cycles`'s
  own header says to read that suite as a commit-to-commit comparison and nothing else.
- **−0.83% of the suite is not −0.83% of anything anybody runs.** The suite is hand-written assembly
  with an instruction mix nothing else has, and `div.S`, `divu.S`, `rem.S` and `remu.S` are four of
  its 74 programs.
- **The skip's threshold is not tuned.** Sixteen is the half that needs no arithmetic — the load is
  a fixed shift and the test is a reduction on sixteen bits. Eight or twenty-four would need a
  variable shift, which is a barrel shifter the divider does not have. Nothing here measured a
  second threshold and declined it; it was not asked.
- **The +75 cells were not shopped for elsewhere.** ADR-0090's and ADR-0112's ceilings say what
  deleting a given block of `rtl/executor.v` would save, and none was re-taken on this tree to pay
  for these. The SoC is at 4881 of 5280 with the requirement met, so nothing forced the question.

## Consequences

- **The divider's latency is now three numbers, not two.** 16 or 32 iterations plus the load cycle,
  or one cycle for the two short-circuits. Anything that quotes a divide's cost has to say which,
  and `CLAUDE.md`, `formal/checks.cfg`'s non-ALTOPS note and `formal/executor-zkt-probe.py`'s
  header were all updated to.
- **The interrupt figure was stale before this change and is now measured.** A number carried
  through two intervening ADRs that each added a cycle behind it is the shape to watch for: nothing
  in the tree could go red for it, because `test/decoder_tb.v` grades that the interrupt waits out
  every stall and not how long the wait is.
- **A cycle win in this block costs cells at roughly one LUT per bit of state it reshapes**, and the
  two edits' costs do not add. Price a pair by building the pair.

# 0159 — The divider skips a zero top half and retires on its last iteration

Status: accepted · two changes to `rtl/executor.v`'s divide FSM, both off the fetch loop and both
bit-identical in result — **`make cycles` 42 319 → 41 969 suite cycles, −350 (−0.83%), every one of
them in the DIVIDER column and no other column moved by one**. They cost **+75 packed
`ICESTORM_LC` on `make fit`** and +36 placed cells on the SoC — **more than the +46 the two cost
when built apart**, which is the number to read. The period is a null at sixteen paired placements with
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

## The cycles, on `d5f418f` with a local Homebrew toolchain

yosys 0.68+post (`c12172fb`), nextpnr-ice40 and `icetime` from the same install,
`riscv64-elf-gcc`, `SOC_PROG` at its default `datainit.c`.

| `make cycles` (74 programs) | before | after | Δ |
|---|---|---|---|
| suite cycles | 42 319 | **41 969** | **−350, −0.83%** |
| DIVIDER | 990 | **640** | **−350** |
| ISSUE · ATOMIC · HAZARD · SERIAL · OPERAND · FETCH · BUS · REGION | 21 279 · 65 · 15 084 · 1 476 · 1 193 · 8 · 0 · 2 224 | identical | 0 |
| CPI | 2.00 | 1.99 | |

The whole delta is in one column and the decomposition is exact. The suite performs **30 real
divides** — 990 / 33 — of which **20 have a dividend magnitude with a zero top half**. After the
change: 20 × 16 + 10 × 32 = 640. The skip saves 16 cycles on 20 divides (320) and the early retire
saves 1 on all 30 (30).

**Those two happen to add, and the reason is that they act on different cycles**, not a general
rule: the skip removes iterations and the early retire removes the capture cycle, and no divide has
its capture cycle removed twice. The cell costs do **not** add, which is the next section.

## The area, and the pair costs more than the parts

`make fit` synthesises and places deterministically; every count below is one run of a distinct
netlist.

| `make fit` (core alone, `littlecpu`) | packed `ICESTORM_LC` | Δ vs branch base | `SB_LUT4` |
|---|---|---|---|
| branch base (`d5f418f`) | 3986 | — | 3670 |
| skip only | 4013 | **+27** | 3698 |
| early retire only | 4005 | **+19** | 3688 |
| **both, as shipped** | **4061** | **+75** | 3748 |

`SB_CARRY` is 579, `SB_DFF`/`SB_DFFESR`/`SB_DFFSR` are 76/612/130 and `ICESTORM_DSP` is 4 on all
four netlists. Nothing gained a chain, a flop or a DSP; the whole cost is 78 LUTs.

**+75 is not +27 and +19, and it is not the +18 the two measured apart on the pre-respell tree**,
where they read +13 and +5. Three of those four numbers moved, and the honest reading is that a cell count is a property of the netlist ABC mapped and not of the idea: the two
edits share the divide state's own logic — the skip widens the mux feeding `div_quot`'s load, the
early retire adds the `_next` nets and a compare to the same block — so the cone they both touch is
mapped once, for both, and the sum of two standalone measurements has no claim on it. This is
[ADR-0097](0097-the-decode-stack-pays-only-in-the-fetch-loop-and-that-is-where-it-cannot.md)'s
spelling-dependence in the other
direction: there, two texts of one idea differed by 44 cells; here, one text of two ideas differs
from the sum of their texts by 29.

**+75 is outside the ±50 nominal churn band**, so it is a cost rather than noise, and the shape
says the same: three synthesis runs of three distinct netlists, monotone in what was added, with
the carry and flop censuses fixed on all of them.

On the SoC's own top the pair is **4845 → 4881 placed cells, +36** — the sweep's own column, at
sixteen seeds a side, identical at every seed.

**The branch as a whole is still a net cell reduction, and it is now a small one.** With `main`'s
(`dba70e9`) `rtl/executor.v` in this tree, `make fit` reads **4088**; the branch's ADR-0151 respell
takes it to 3986 and these two changes put it at 4061. So the branch is **−27 against `main`**,
where it was −102 — a reduction that no longer clears the churn band. `FIT_MAX_LC` is 4219 and
`make fit`'s ratchet passes with 158 cells of budget; `FIT_LAST_LC` stays at 4097, which is the
`fit` **job's** number and not this local instrument's.

## The period, at sixteen placements paired per seed

`make netlist-digest` moved (`ee02d64e…` → `b4d43206…`) with the SoC's synthesised `SB_LUT4` at
4344 → 4383, so the sweep was owed. Both sides run through `soc/baseline_sweep.sh` at the same
sixteen seeds, one toolchain, `datainit.c`.

| | before | after |
|---|---|---|
| worst of 16 | 79.41 ns — **12.59 MHz** | 79.69 ns — **12.55 MHz** |
| median | 78.11 ns — 12.80 MHz | 78.27 ns — 12.78 MHz |
| best | 76.30 ns — 13.11 MHz | 76.10 ns — 13.14 MHz |
| best-to-worst spread | 4.1% | 4.7% |
| under 12.00 MHz | **0 of 16** | **0 of 16** |
| placed `ICESTORM_LC` | 4845 | 4881 |
| LUT levels · carry hops | 21–22 · 0–7 | 20–22 · 0–8 |

**Median of the per-seed deltas: +0.75%. Ten of sixteen seeds slower, two-sided sign test
p = 0.454.** That is a null in both directions — a fifth of this part's ~3.6% edit-churn band, and a
coin flip on the sign. The requirement holds at sixteen of sixteen on both sides and the worst
placement moves 12.59 → 12.55 MHz, which is 0.4% and is neither a cost to record nor a regression to
act on. The largest single-seed moves go both ways: seed 2 is 2.95% faster, seed 7 is 2.42% slower.

`soc/baseline_summary.py` **refuses** this pair without `--allow-mismatch`, and correctly: the
"after" side was swept on a tree with uncommitted changes, so its stamp names no commit. That is
what a before-and-after against a branch tip is, the difference is the one being measured, and the
mismatch is printed beside the delta rather than instead of it. Re-taking the "after" side from a
commit would produce the same netlist and the same numbers with a stamp `baseline_summary.py`
accepts; it was not re-taken, and that is the one respect in which this sweep is short of the
procedure `soc/baseline_sweep.sh`'s header describes.

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
number was carried forward through both. `CLAUDE.md` now states 35.

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
  reports **1 712 025 timed cycles and 0.664 DMIPS/MHz on both trees** — the same digits, not a
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

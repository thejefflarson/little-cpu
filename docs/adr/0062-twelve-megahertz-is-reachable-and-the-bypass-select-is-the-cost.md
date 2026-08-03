# ADR-0062: Twelve megahertz is reachable, and the write-through bypass select is the cost

**Status:** Accepted · 2026-08-02 · *Answers
[ADR-0059](0059-text-is-writable-and-the-arbiter-lives-in-the-memory.md)'s open question about
ADR-0038's 12 MHz intent. Corrects
[ADR-0058](0058-the-fetch-loop-is-not-two-levels-too-deep.md)'s guess at where the decode head's
levels are, and its second-path cap. Method copied from
[ADR-0057](0057-what-writable-text-costs-in-ladder-depth-and-in-nanoseconds.md). No `rtl/` change
ships from this ADR.*

## Why 12 MHz is a number rather than a percentage

`soc/littlesoc.pcf` targets the iCEBreaker up5k/sg48, whose board crystal is 12 MHz. The part's own
oscillator is 48 MHz nominal with a divider of 1, 2, 4 or 8 — 48, 24, 12, 6. Twelve is native from
either source and the step below it is six.

So the gap this project has been reporting as a percentage is not one. At 10.2 MHz (ADR-0059) the
part has to be clocked at 6, and the 15% shortfall costs half the clock. ADR-0038 declared 12 MHz an
intent and three ADRs have since measured against it without saying what missing it costs.

## Method

`make soc-timing` with `SOC_PROG=add.S`, four placements per variant — the default plus
`nextpnr-ice40 --seed 1/2/3` — which is ADR-0057's method and now lives in `soc/timing_sweep.sh`
rather than being hand-run for a fourth time. **Homebrew Yosys 0.67+post (`b8e7da6f`)**,
nextpnr-0.10-108-g68c1acd8, `icetime` from the same install. Machine load 10–25 throughout with a
sibling build live; `icetime` is a static analysis and nextpnr is seeded, so load moves the wall
time and not the numbers.

**The baseline reproduces ADR-0059 exactly** — 96.92 · 97.75 · 98.08 · 98.19 ns at 4304 logic
cells — so the rows below are comparisons rather than measurements from different worlds.

## The candidates

| variant | LC | `icetime` ns, four placements | MHz | median vs baseline |
|---|---|---|---|---|
| baseline (`7dba8db`) | 4304 | 96.92 · 97.75 · 98.08 · 98.19 | 10.18 – 10.32 | — |
| **bypass on the held address** | **4334** | **77.47 · 78.20 · 80.93 · 81.16** | **12.32 – 12.91** | **−18.7%** |
| split the 64-bit counters | 4317 | 97.53 · 99.29 · 99.79 · 100.03 | 10.00 – 10.25 | +1.7% |
| the steal mux decoded a cycle early | 4294 | 96.43 · 97.49 · 99.31 · 100.37 | 9.96 – 10.37 | +0.5% |
| the bank-address tail deleted (a ceiling) | 4271 | 92.14 · 94.90 · 95.29 · 97.10 | 10.30 – 10.85 | −2.9% |
| bypass + counters split | 4326 | 78.15 · 78.94 · 79.51 · 80.87 | 12.37 – 12.80 | −19.1% |

Only the first clears 83.33 ns, and it clears it at every placement measured. Its distribution does
not overlap the baseline's and the move is five times ADR-0054's 3.6% edit band, so it is the change
rather than churn. The +30 cells are inside ADR-0038's ±50 churn floor and mean nothing in either
direction.

Everything else is inside the bands. The counter split and the registered steal are **measured
nulls**; the bank-address tail is a ceiling — the increment and the steal mux both deleted, which no
implementation can beat — and it does not reach 3.6% either. The registered steal is worth noting
twice over: it is ADR-0057's named escape hatch, and at one of its four placements it measures
9.96 MHz and trips `SOC_MIN_MHZ`.

## The change that works, in full

`rtl/regfile.v`'s write-through bypass selects on `rs1`/`rs2`, which are combinational out of the
instruction word this cycle's fetch just returned:

```systemverilog
reg_rs1 = (rs1 == 5'd0) ? 32'b0 : (wen && waddr == rs1) ? wdata : read_a;
```

Invariant 9 says the registered read answers the address pair presented in the **previous** cycle,
and `operand_stall` holds the PC until the pair the issuing instruction needs is the pair that was
presented. So a registered copy of that pair selects the same value with a select that comes out of
flip-flops:

```systemverilog
logic [4:0] held_rs1, held_rs2;
always_ff @(posedge clk) begin
  held_rs1 <= rs1;
  held_rs2 <= rs2;
end
always_comb begin
  reg_rs1 = (held_rs1 == 5'd0) ? 32'b0 : (wen && waddr == held_rs1) ? wdata : read_a;
  reg_rs2 = (held_rs2 == 5'd0) ? 32'b0 : (wen && waddr == held_rs2) ? wdata : read_b;
end
```

The write-first term inside the `always_ff` keeps using the presented `rs1`, because that is the
address being presented on that edge. It does not move.

**No invariant is relaxed.** Invariant 6 asks that decode observe, in the cycle it issues, the
architectural value of rs1/rs2 including a writeback committed in that same cycle; on an issuing
cycle `held_rsN == rsN` for every port the instruction reads, so the value is the same one. There is
no flush (invariant 1) and no forwarding network (invariant 4) — the two forwarding points are the
two that were already there, re-addressed.

## Where the levels are, which is not where ADR-0058 guessed

Each row is one placement, so no row is evidence of a delta on its own — the churn band is 3.6% and
these are separated by less. The **LUT level count** is what carries: it comes out of the netlist
rather than out of the placement, and the baseline is 29.

| probe | ns | LUT levels |
|---|---|---|
| baseline | 98.08 | 29 |
| drop the branch arm from `next_pc` | 83.58 | **23** |
| ...keep its target adder, make the select shallow (`instr_beq`) | 83.16 | **24** |
| ...keep its select, drop the target adder | 96.52 | 29 |
| ...keep six arms, replace the four magnitude comparisons with register bits | 86.35 | **24** |
| no write-through bypass at all (violates invariant 6) | 78.10 | **20** |
| drop the `jalr` arm | 95.17 | 28 |
| drop the `jal` arm | 98.61 | 28 |
| drop the `mret` arm | 97.65 | 28 |
| drop the trap arm | 99.25 | 29 |
| drop all five redirect arms | 77.08 | 22 |
| the eighteen-way immediate mux collapsed to one arm | 97.70 | 28 |
| `hazard` and `operand_stall` out of `stall` | 99.24 | 29 |
| the ROM's out-of-range mask deleted | 92.89 | 27 |
| the decoder's `instr[31:16]` mask deleted | 98.44 | 29 |
| the steal mux deleted | 96.57 | 28 |
| the bank `+ W[0]` increment deleted | 91.73 | 26 |

Read down the chain. Of the five redirect arms only the branch one carries depth, and it is not the
target adder — a shallow select with the same adder is 24 levels. Inside the select it is the four
magnitude comparisons, whose operands are the two register reads. And those reads are not registers
at the point of use: the write-through bypass puts a mux in front of them whose select is
`waddr == rs1`, and `rs1` is `instr[19:15]` out of the word the ROM returned this cycle. So the
32-bit comparators sit **after** the instruction rather than beside it, and every branch pays for
that before the next PC can be chosen.

Deleting the bypass outright reaches 20 levels, so of the nine levels it costs the held-address
spelling recovers six or seven and leaves the mux.

**ADR-0058's consequence named `immediate`'s eighteen-way mux and the `rs1`/`rs2` selection feeding
`hazard` as where the levels are. Both are measured null here** — collapsing the immediate mux to a
single arm leaves 28 levels, and taking `hazard` and `operand_stall` out of `stall` leaves 29. That
ADR reasoned from the shape of the RTL; this one probed it.

Two spellings of the branch comparison were also built and are null: three named comparisons
(`cmp_eq`/`cmp_lt`/`cmp_ltu`) instead of six is 97.83 ns at 29 levels, and a compare-then-invert
form selected on the funct3 bits is 98.22 ns at 29 levels. yosys flattens both back to the same
cone. Sharing one target adder between `jal` and a taken branch — the same sum written twice — is
96.23 ns at 28 levels and saves 7 cells, which is nothing on both axes. **The problem was never how
the comparison is written.**

## Both paths, and the second one is not the counter

ADR-0058 found a second path within 1.7% of the fetch loop — decode → `stall` → `instret` →
`minstret`'s 64-bit counter at 87.04 ns — and called it a cap on fetch-loop tuning. It is measured
again here, on the tree ADR-0059 left, with a probe that registers `imem_addr_next` in
`rtl/fetcher.v`: the fetch loop is cut at one point and every other cone is left intact, so the
result is the second path at roughly the right netlist density.

| variant | LC | `icetime` ns, four placements | end point |
|---|---|---|---|
| loop cut | 4206 | 77.75 · 78.03 · 81.28 · 81.91 | `minstret[34..37]` at three of four |
| loop cut, counters split | 4263 | 76.71 · 77.47 · 78.35 · 80.20 | `mscratch` / `mcause` / `minstret` / `mepc` |
| loop cut, bypass on the held address | 4258 | 64.60 · 65.10 · 66.69 · 67.38 | `mscratch` / `mtvec` / `minstret` |

Three things follow.

- **The second path is 81.91 ns at worst, not 87.04.** ADR-0058 measured it by deleting the stall
  term from `next_pc`, which on that tree moved the critical path off the loop; after ADR-0059 the
  loop terminates at the bank read address instead of at a registered range flag, so the same probe
  no longer cuts it (93.83–95.73 ns, still ending in the ROM). The cap that ADR-0058 wrote down
  applied to the design it measured.
- **Splitting the counter is a null.** The split carries the low half's carry into the high half in
  the same cycle (`carry = tick && &lo`), so the architectural value is exact and `csrc_upcnt` still
  holds. It costs 57 cells and moves the median 2.2%, inside the band, with the end point scattering
  across four different registers — which is the shape of a path that was never specifically the
  counter. The one-cycle-late carry the alternative would use is worse than null: it makes the
  64-bit value fall by 2^32 for a cycle at the wrap, which `csrc_upcnt_minstret_ch0` asserts against
  and ADR-0027's rule reads on.
- **The second path is fixed by the same change as the first.** `trap_pending` includes the
  load/store misalignment test, which is computed from `reg_rs1`, so the bypass select is on this
  path too.

None of this needs a separate gate. `make soc-timing` reports the worst path in the design, so the
81.16 ns figure in the candidate table already bounds the counter, the CSR writes and the fetch loop
together. The probe above exists to say *what* the second path is, not to add a number that has to
be cleared.

## Verdict: 12 MHz is reachable, in one change

Address the write-through bypass from the held register pair and the design measures
**77.47 – 81.16 ns, 12.32 – 12.91 MHz, at four placements**, against a 83.33 ns budget. Nothing else
measured here is worth landing.

The sequence is one step and it is small. It was run against every gate this box can run, and one
of them has something to say.

- **`make test` is 56/56** with `test/EXPECTED_FAIL` matching exactly, and every program's retire
  and spec-checked counts unchanged.
- **`make test-units` catches exactly one assertion**, and it is the interesting part.
  `test/regfile_tb.v` asserts `x0 reads 0 in the fetch cycle (rs1)`; with the held pair the fetch
  cycle answers the **previous** pair, so it reads that register instead. That is the invariant-9
  statement the same bench already makes about the use cycle, arriving one cycle earlier: after the
  change, `reg_rs1`/`reg_rs2` are never keyed to the address presented this cycle, in any cycle.
  Nothing in the core reads them there — `operand_stall` is high through a fetch cycle, so `issuing`
  is low and `next_pc` holds — so the bench is the only consumer that can see it. The implementing
  change rewrites that one assertion and says what it now pins. The other six benches pass unchanged.
- **`reg_ch0` passes, and it is still looking.** ADR-0040's liveness probe — delete the rs2
  write-through bypass — gives `bad state property 1 reachable at bound k = 20 SATISFIABLE` under
  the change, so the PASS is not a check that stopped asking. (It reports `rc=16` rather than `FAIL`
  on a box with no `btorsim`, because the counterexample is found and then the trace step fails;
  read the log, not the status.)
- **The full ladder is 85/85** with both set equalities exact.
- **`make cosim-suite` was not run** — no Sail install on this box. It reads the real register array
  rather than the core's self-report (ADR-0032), which is the one oracle positioned to see an
  operand that is stale rather than wrong, so the implementing change owes it.
- **`SOC_MIN_MHZ` moves with the change.** It is 10.0 and sits 1.8% under the worst placement
  (ADR-0059). Keeping the margin it was derived with — 11.5% under the measurement — puts it at
  10.9 against a 12.32 MHz worst placement.

## Consequences

- **Invariant 6 comes to depend on invariant 9, and that is a new coupling.** Today the bypass is
  correct for any address pair; after the change it is correct because `operand_stall` guarantees
  that on an issuing cycle the held pair is the presented pair. A later change that narrows
  `operand_stall` — to skip the bubble for some instruction class, say — would break invariant 6
  silently, in a place nobody would look. The implementing change should say so at the site.
- **ADR-0038's 12 MHz intent stops being unreachable, and nothing here moves it.** ADR-0059 recorded
  that the number to reconcile it against was 10.2 MHz; it is 12.3 if this lands. The declaration is
  unchanged either way — this ADR measures, it does not re-declare.
- **A PLL is not needed.** `SB_PLL40_*` would synthesize an arbitrary clock and add a primitive the
  formal flow has never seen; the measurement says the design can reach the clock the board already
  supplies.
- **`soc/timing_sweep.sh` ships.** It runs `make soc-timing` at four placements and prints the
  spread. It grades nothing of its own: the `SOC_MIN_MHZ` ratchet stays in `soc/timing_split.py`,
  where ADR-0053's probe already reaches it, and the sweep hands that target's exit status back.
  **Its first version did not.** `make soc-timing | grep 'critical path'` takes grep's status, so a
  placement under the floor printed its row and the sweep exited 0 — the shape ADR-0037 found in a
  graded comparison piped into `tee`, reproduced in a new script within an hour of that ADR being
  read. Both directions are demonstrated: `soc/timing_sweep.sh SOC_MIN_MHZ=99` exits 1 with the
  ratchet's own message, and an ordinary sweep exits 0.
- **The spike RTL is not committed anywhere.** Every variant was built, measured and reverted; the
  one worth landing is written out in full above, and each probe is named by what it deletes, so
  none of them needs a branch to be reconstructed from.
- **The tail after the next-PC adder is not worth another attempt.** Deleting both the bank
  increment and the steal mux — a ceiling no implementation can beat, and one that breaks text
  access outright — buys 2.9% at the median. ADR-0057's registered-steal fallback, named there as
  the escape hatch if the mux proved expensive, is a null for the same reason: the mux is on the
  path at its *data* input, and registering its select does not shorten that.

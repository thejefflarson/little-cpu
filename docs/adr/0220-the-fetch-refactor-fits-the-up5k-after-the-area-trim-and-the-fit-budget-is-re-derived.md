# 0220 — The fetch refactor fits the up5k after the area trim, and the fit budget is re-derived

Status: Accepted · 2026-09-30

## Context

ADR-0221 shipped the fetch refactor red on `make fit` and `make soc-timing` by the owner's own
decision, and named a cell-trim pass as owed. The trim landed on `thejefflarson/jef-1056-area-trim`.
This ADR records what it changed that no other ADR holds (the block-RAM read skip), the measurements
of the tree that results, the re-derived `FIT_MAX_LC`, and the cross-core re-take. The skid's
deletion, the guess steering fetch, the registered redirect and the atomic region move are in
ADR-0221's amendments; the registered device stores are in ADR-0214's.

## The block-RAM read skip

`rtl/imemory.v` shares one read port between fetch and a text store. A read beside a write to the
same block RAM makes the mapper emulate read-first behaviour: 82 flip-flops for delayed write data,
address and enable, plus the bypass muxes. The read now does not happen on a text-write cycle
(`if (!text_write)` around the two output registers), because the fetch the store stole is
re-presented anyway and the output register's stale contents are never consumed. Platform-only
packed cells fall 853 → 705 and the SoC 5,408 → 5,316 on a single draw.
`test/imem_share_test.sh` gained a flip-flop census and a forced-red read-on-write mutant, so the
bypass returning is red rather than silent. This is an area lever from a fact outside the
expression, in CLAUDE.md's sense (ADR-0088): yosys cannot know the stolen fetch is dead.

## Measurements

All on the shipped tree, `thejefflarson/jef-1056-area-trim`, 2026-09-30, gcc 15.2.0.

| Instrument | Before (fetch-refactor tip) | Now |
|---|---|---|
| `make soc-timing` packed cells | 5,321 of 5,280, placement fails | 5,084 of 5,280 |
| `make soc-timing` MHz, 8 seeds one at a time | not placed | 12.57 to 13.24, all at least 12.0 |
| `soc/pin.json` | stale | seed 20382078 at 13.24 MHz |
| `make fit` packed cells | 4,524 (local) | 4,332 (CI job), 4,347 (local) |
| F and G | 5 and 5 | 5 and 4 |
| Dhrystone, `make dhrystone` | 1,613,644 cycles (main, 0.722 DMIPS/MHz) | 1,206,025 cycles at 2,000 runs, 0.943 DMIPS/MHz |
| CoreMark, `make coremark` | 2.155 per MHz (main) | 2.776 per MHz (36,010,251 ticks, 100 iterations, 16 KB simulated ROM) |

The eight seeds are 195147338, 218749127, 20740127, 125781539, 14871351, 156842832, 233595587 and
20382078, and the worst placement is 12.57 MHz. CoreMark's guess counters read 3,142,314 guesses, 2,905,067 of them hits (92.5%).

## `FIT_MAX_LC`, re-derived

ADR-0142's rule: the budget is a measured count, plus the churn band measured on the tree, plus the
widest toolchain gap measured on one tree. The Makefile's comment says "raising it needs a reason in
the commit"; the reason is that the restructure's measured cost (the D/X split, the forwarding mux,
the predictor's target adder and the registered redirect) is now paid by a SoC that places at 12 MHz.

4441 = 4347 + 40 + 54.

- **4347** the higher of this tree's pair: a local `make fit` reads 4,347 and the `fit` CI job on
  this PR (run 36686078168) reads 4,332, a gap of 15 with the local run above. ADR-0142 grades the
  job's count, and its rule that the budget clear the higher of the pair by more than a band holds:
  4,441 clears 4,347 by 94.
- **+40** the churn band measured on this tree. Seven edits that change no logic, each setting one
  further bit of the read-only `misa` constant in `rtl/csrs.v` (`0x4000_1107`, `110D`, `1145`,
  `1905`, `0x4001_1105`, `0x4010_1105`, `0x4000_1125`), read 4,362, 4,362, 4,376, 4,376, 4,357,
  4,357 and 4,387 against a base of 4,347. The span is 40 cells, and every probe landed above the
  base, so the count can sit at the bottom of a window with all of it still to come. This is below
  ADR-0142's 68 and 63 from smaller trees, and is a sample of seven, not a range.
- **+54** ADR-0142's widest gap between two toolchains on one tree, carried over: it is a property
  of yosys builds and CI floats the suite, and one local toolchain cannot re-measure it.

`FIT_LAST_LC` moves to 4,332, the job's count. Nothing here is headroom for the next change.

## Cross-core measurement

`make compare-dhrystone` and `make compare-coremark`, one session, one tree, pinned gcc 15.2.0,
littlecpu at this tree's RTL, VexRiscv and Hazard3 at their pins. Cycles are between the harness's
two markers; Dhrystone runs 400 iterations in this harness.

| Row | littlecpu | VexRiscv | Hazard3 | VexRiscv / littlecpu | Hazard3 / littlecpu |
|---|---|---|---|---|---|
| Dhrystone, RV32IM, three-way | 228,825 (572.1 cycles/run, 0.995 DMIPS/MHz) | 262,827 (0.866) | 252,026 (0.903) | 1.149 | 1.101 |
| Dhrystone, RV32IMC, littlecpu and VexRiscv | 228,825 | 269,629 (0.844) | not run | 1.178 | not run |
| CoreMark, RV32IM, three-way | 359,507 (2.782 per MHz) | 426,430 (2.345) | 666,552 (1.500) | 1.186 | 1.854 |
| CoreMark, RV32IMC, littlecpu and VexRiscv | 359,505 | 437,545 (2.285) | not run | 1.217 | not run |

The VexRiscv and Hazard3 cycle counts are digit-identical to the ones ADR-0190 recorded under the
pin (VexRiscv 262,827 and 426,430; Hazard3 252,026 and 666,552), which is the check that only
littlecpu's side moved. Against ADR-0190's littlecpu figures (313,627 and 446,995) this core is
27.0% and 19.6% fewer cycles. It is now ahead of both cores on both benchmarks, where ADR-0160
recorded both VexRiscv and Hazard3 ahead of it on Dhrystone.

Two limits. The harness stamp is stale (`soc/compare/product.json` predates this RTL), so the
clock halves are not re-taken and no product is quoted; cycles alone are the comparison on the
up5k, where all three cores quantise to the same 12 MHz step. And `make compare-dhrystone`'s image
now fits the placed geometry (1,968 bytes of text), so nothing there is distorted by memory size,
while CoreMark's cycles are still simulated at 16 KB against a placed 8.

## Verification

Recorded in the PR that carries this ADR. The instrument results above are the measured ones; the
gates are the PR's checks.

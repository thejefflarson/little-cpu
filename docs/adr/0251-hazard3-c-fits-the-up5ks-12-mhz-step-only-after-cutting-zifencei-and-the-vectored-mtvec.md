# ADR-0251: `hazard3_c` fits the up5k's 12 MHz step only after cutting Zifencei and the vectored `mtvec`

**Status:** Accepted · 2026-10-11 · *Builds on ADR-0160, ADR-0246 and ADR-0250. No `rtl/` change ships
from this ADR.*

## Context

ADR-0250 added a feature-matched RV32IMAC row. Its Hazard3 build, `hazard3_c` (the performance build
plus `EXTENSION_C`), places on the up5k at 11.01 / 11.65 / 11.93 MHz, worst / median / best of twelve
seeds, so it misses the 12 MHz step and leaves the up5k comparison. The owner's instruction: cut
Hazard3 features until it does fit, even if the feature set is no longer the same, so long as it is
closer to what littlecpu has.

The rules for "closest": every candidate must clear 12.00 MHz at all twelve seeds. Among those, keep
as much of littlecpu's ISA (RV32IMAC, Zicsr, Zifencei) as possible, cutting Zifencei before A and A
before C, never M. Among builds with the same ISA, take the one with fewer cycles. Non-ISA knobs
(`BRANCH_PREDICTOR`, `CSR_COUNTER`, the `mtvec` mask, `MUL_FAST`) are free to cut where they help the
clock, and each one is reported. Every candidate ran twelve paired seeds against a `hazard3_c` control
from the same session.

## Decision

**`hazard3_c_fit` is `hazard3_c` without Zifencei (`EXTENSION_ZIFENCEI` 0) and with `mtvec` fixed to
direct mode (`MTVEC_WMASK` `0xfffffffc`).** It keeps A, C, M, Zicsr, the branch predictor, the counters
and the single-cycle multiply. It clears the step at all twelve placements: 12.13 / 12.36 / 12.60 MHz,
4,422 placed `ICESTORM_LC` of 5,280, and takes exactly `hazard3_c`'s cycles (234,427 Dhrystone and
351,710 CoreMark). It is the up5k comparison's Hazard3 row.

What was cut, and why each cut is the smallest that works:

- **Zifencei** is the ISA cut. The ISA order says it is the cheapest, and it is the structural one:
  dropping it alone moves the median from 11.64 to 12.04 and clears 6 of 12 seeds. Neither benchmark
  executes `fence.i`, and the image's `.text` is byte-identical when compiled at `rv32imac_zicsr`,
  `rv32imc_zicsr` or `rv32imac_zicsr_zifencei` (equal section hashes for Dhrystone and CoreMark,
  checked for this ADR), so littlecpu and every other core run the very same binary. The matched pairs
  keep `rv32imac_zicsr_zifencei` in their provenance; `hazard3_c_fit` simply lacks an instruction the
  binary does not contain.
- **`mtvec` direct only** is a non-ISA cut: it removes the vectored-interrupt target logic (littlecpu
  has no vectored `mtvec` either, so it is also a match). By itself it moved nothing (`hazard3_c` with
  it: median 11.80), and with Zifencei off it takes the build from 6 of 12 seeds to 12 of 12. The honest
  reading is that Zifencei's removal brings the median up to the step and the `mtvec` mask decides
  which side of it the worst placement falls; the placer's worst-to-best spread is 0.5 to 1.2 MHz
  across the tables below.
- A and C are kept. Dropping A as well clears with more margin (12.41 / 12.85 / 13.19) but gives up an
  extension to buy margin that was not asked for.

The other builds that clear 12 of 12 all lose on the ranking: no Zifencei and no branch predictor
(12.18 / 12.50 / 12.58, keeps A and C, but 3.0% more Dhrystone and 5.7% more CoreMark cycles), and
three builds that also cut A. A C-keeping build cleared, so the no-C fallback (`hazard3_perf`, 12.23
worst) was not needed.

`bench_hazard3.v` gains a `FIT` parameter (set only with `PERF` and `WITH_C`); `hazard3_builds.txt` has
a fourth column; `hazard3_config_test.py` grades it as the C column plus `EXTENSION_ZIFENCEI` and
`MTVEC_WMASK` and nothing else. `hazard3_c` and its ECP5 row are unchanged. `make
compare-dhrystone-matched`, `make compare-coremark-matched`, `make compare-smoke`, `run_product.sh`'s
sweep and pairs and `comparison.py`'s matched section carry the new core; the weekly workflow names
pairs, not cores, and needed no edit.

## Measurements

Tree `a470e8e` plus this change, xPack gcc 15.2.0, OSS CAD Suite 20260811 (yosys 0.68+48, nextpnr
0.11-1-g62e659e), one toolchain throughout. Clocks: twelve seeds a part (`default`, 1 to 11) through
`make compare-timing`. Cycles: one image, one simulation (`make compare-dhrystone-matched`, 400 runs;
`make compare-coremark-matched`, one iteration); RAMs identical to littlecpu's. The control reproduces
ADR-0250 to the digit on the up5k (11.01 / 11.64 / 11.93; 234,427 and 351,710 cycles; 4,444 LC) and
to 0.01 MHz on ECP5. Knobs not named are at `hazard3_c`'s value. "Cleared" is seeds at or above 12.00.
Cycles are Dhrystone / CoreMark; rows whose cycles were not simulated take them from a simulated
sibling that differs only in knobs that do not touch the pipeline in these programs (Zifencei, A,
`CSR_COUNTER`, `MTVEC_WMASK`, `MULH_FAST`); the shipped build was simulated and equals the control.

Round one, ISA held (the `hazard3_c` control and 17 variants):

| candidate | up5k MHz | cleared | placed LC | cycles |
|---|---|---|---|---|
| `hazard3_c` (control) | 11.01 / 11.64 / 11.93 | 0 | 4,444 | 234,427 / 351,710 |
| `BRANCH_PREDICTOR` 0 | 11.82 / 12.27 / 12.72 | 11 | 4,113 | 241,627 / 371,634 |
| `MUL_FAST` 0 | 11.80 / 11.93 / 12.19 | 3 | 4,404 | 247,627 / 652,706 |
| `CSR_COUNTER` 0 | 10.73 / 11.26 / 11.91 | 0 | 4,042 | 234,427 / 351,710 |
| `REDUCED_BYPASS` 1 | 10.80 / 11.21 / 11.47 | 0 | 4,388 | 316,435 / 490,217 |
| `FAST_BRANCHCMP` 0 | 9.77 / 10.08 / 10.38 | 0 | 4,405 | 234,427 / 351,710 |
| `MTVEC_WMASK` direct | 11.36 / 11.80 / 11.91 | 0 | 4,419 | 234,427 / 351,710 |
| predictor 0, `MUL_FAST` 0 | 11.47 / 11.86 / 12.08 | 4 | 4,077 | 254,827 / 672,630 |
| predictor 0, `CSR_COUNTER` 0 | 11.62 / 12.04 / 12.30 | 7 | 3,705 | 241,627 / 371,634 |
| predictor 0, `MULH_FAST` 1 | 11.59 / 11.82 / 12.08 | 1 | 4,391 | 241,627 / 371,634 |
| predictor 0, `MTVEC_WMASK` direct | 11.96 / 12.20 / 12.44 | 11 | 4,126 | 241,627 / 371,634 |
| `MUL_FAST` 0, `MTVEC_WMASK` direct | 11.30 / 11.55 / 11.86 | 0 | 4,378 | 247,627 / 652,706 |
| `CSR_COUNTER` 0, `MTVEC_WMASK` direct | 11.32 / 11.69 / 11.86 | 0 | 4,024 | 234,427 / 351,710 |
| predictor 0, `MUL_FAST` 0, `CSR_COUNTER` 0 | 11.70 / 12.16 / 12.50 | 11 | 3,664 | 254,827 / 672,630 |
| predictor 0, `MUL_FAST` 0, `MTVEC_WMASK` direct | 11.98 / 12.36 / 12.50 | 11 | 4,107 | 254,827 / 672,630 |
| predictor 0, `CSR_COUNTER` 0, `MTVEC_WMASK` direct | 11.47 / 11.74 / 12.00 | 1 | 3,718 | 241,627 / 371,634 |
| predictor 0, `MULH_FAST` 1, `MTVEC_WMASK` direct | 11.46 / 11.80 / 12.02 | 1 | 4,478 | 241,627 / 371,634 |
| predictor 0, `MUL_FAST` 0, `CSR_COUNTER` 0, `MTVEC_WMASK` direct | 11.43 / 11.90 / 12.19 | 5 | 3,667 | 254,827 / 672,630 |

No ISA-held candidate clears twelve of twelve; two come within 0.04 MHz (11.96 and 11.98).

Round two, ISA cut (all keep C and M; 12 variants):

| candidate | up5k MHz | cleared | placed LC | cycles |
|---|---|---|---|---|
| no Zifencei | 11.62 / 12.04 / 12.37 | 6 | 4,464 | 234,427 / 351,710 |
| no Zifencei, `CSR_COUNTER` 0 | 11.60 / 12.17 / 12.54 | 7 | 3,988 | 234,427 / 351,710 |
| no Zifencei, `CSR_COUNTER` 0, `MTVEC_WMASK` direct | 11.37 / 11.65 / 11.84 | 0 | 4,160 | 234,427 / 351,710 |
| no Zifencei, no predictor, `CSR_COUNTER` 0 | 11.15 / 11.61 / 12.01 | 1 | 3,709 | 241,627 / 371,634 |
| no Zifencei, `MTVEC_WMASK` direct (**`hazard3_c_fit`**) | **12.13 / 12.36 / 12.60** | **12** | 4,422 | 234,427 / 351,710 |
| no Zifencei, no predictor | 12.18 / 12.50 / 12.58 | 12 | 4,234 | 241,627 / 371,634 |
| no A | 11.26 / 11.65 / 12.12 | 2 | 4,117 | 234,427 / 351,710 |
| no A, no predictor | 11.52 / 12.05 / 12.30 | 7 | 3,874 | 241,627 / 371,634 |
| no A, no predictor, `CSR_COUNTER` 0 | 11.74 / 12.09 / 12.42 | 7 | 3,394 | 241,627 / 371,634 |
| no Zifencei, no A | 12.41 / 12.85 / 13.19 | 12 | 4,205 | 234,427 / 351,710 |
| no Zifencei, no A, no predictor | 12.17 / 12.66 / 13.19 | 12 | 3,873 | 241,627 / 371,634 |
| no Zifencei, no A, no predictor, `CSR_COUNTER` 0 | 12.51 / 12.81 / 13.22 | 12 | 3,372 | 241,627 / 371,634 |

Every candidate places (4,478 LC or fewer of 5,280). Of the builds that clear twelve of twelve and keep
A, C and M (the ISA tier the order picks), `hazard3_c_fit` and "no Zifencei, no predictor" are the two;
`hazard3_c_fit` wins the cycle tie-break with 3.0% fewer Dhrystone and 5.4% fewer CoreMark cycles.

The row, shipped text, twelve seeds (the sweep was re-run on the text that ships and equals the
exploratory run to the digit):

| core | up5k MHz | up5k LC | ECP5 MHz | ECP5 `TRELLIS_COMB` | Dhrystone cycles (DMIPS/MHz) | CoreMark cycles (CoreMark/MHz) |
|---|---|---|---|---|---|---|
| littlecpu | 12.92 / 13.28 / 13.54 | 4,536 | 38.17 / 39.64 / 41.03 | | 228,825 (0.995) | 359,505 (2.782) |
| vexriscv_lrsc | 21.26 / 22.09 / 23.23 | 3,533 | 53.82 / 56.92 / 58.38 | | 269,629 (0.844) | 437,545 (2.285) |
| hazard3_c | 11.01 / 11.64 / 11.93 | 4,444 | 39.36 / 40.95 / 42.05 | 4,721 | 234,427 (0.971) | 351,710 (2.843) |
| **hazard3_c_fit** | **12.13 / 12.36 / 12.60** | 4,422 | 38.88 / 41.74 / 43.16 | 4,854 | 234,427 (0.971) | 351,710 (2.843) |

littlecpu's and `vexriscv_lrsc`'s clocks are ADR-0250's, same toolchain, not re-run. At the 12 MHz step
`hazard3_c_fit` delivers 11.65 DMIPS and 34.12 CoreMark against littlecpu's 11.94 and 33.38 (it takes
1.024x littlecpu's Dhrystone cycles and 0.978x its CoreMark cycles) and `vexriscv_lrsc`'s 10.13 and
27.43. On ECP5 at the worst placement it is 37.76 DMIPS and 110.55 CoreMark against littlecpu's 37.98
and 106.17 (0.994x and 1.041x); at the median 40.54 and 118.68 against 39.44 and 110.26 (1.028x and
1.076x).

The twelve-seed verdict rests on the tail (worst 12.13, 0.13 MHz over the step), so it was re-taken at
sixteen high-entropy seeds (a seeded draw, none in 1 to 16: 1057243201 745345665 1065623204 726864694
600004657 118787756 86910193 770116510 1031656204 80851806 1481527226 32444773 1515826877 1859401094
1636872853 1442583671), the sample the repo asks for when a verdict is a tail, paired with the
`hazard3_c` control in the same session on the same toolchain:

| core | up5k MHz, 16 high-entropy seeds | under 12.00 |
|---|---|---|
| hazard3_c (control) | 11.04 / 11.63 / 11.86 | 16 of 16 |
| **hazard3_c_fit** | **12.18 / 12.33 / 12.49** | **0 of 16** |

All sixteen clear, with a worst placement 0.18 MHz over the step, so "no Zifencei, no predictor" (the
named fallback) was not taken up and no build that cuts A was needed.

Standing flags: the Hazard3 ECP5 clock inherits the unpinned-nextpnr flag in CLAUDE.md; CoreMark's
cycles are simulated at 16 KB of ROM against a 4 KB placed one; Hazard3's disclosed bus wait (28,805
and 14,176 cycles) is in every row. The ECP5 sweep ran for `hazard3_c_fit` only.

## What it changes

- The up5k comparison gains a Hazard3 row, with one caveat the table carries: it is `hazard3_c` minus
  Zifencei and the vectored `mtvec`, not the build ADR-0250 named.
- The result is a feature-set cut, not a free fit. Round one shows nothing that holds the ISA reaches
  twelve of twelve, and the C extension's decompressor sits in the fetch path that sets the period
  (two worst paths read, both from predictor-off placements: decompressor output, decode, the
  branch-target adder, the fetch address, the stall logic; two paths are a sample, not a census).
  Zifencei is the cheapest extension to give back.
- The clearance is 0.13 MHz at the worst of twelve placements and 0.18 MHz at the worst of sixteen
  high-entropy ones; the same cut without the `mtvec` mask misses at 6 of 12. It is a verdict on this tree
  and toolchain, not a guarantee under a different yosys or nextpnr; the weekly stamp re-takes it.

## Consequences

- `run_product.sh` sweeps `hazard3_c_fit` and writes it into the four matched pairs, all at one image
  compiled at `rv32imac_zicsr_zifencei`; `hazard3_c` stays out of the up5k pair (11.01 worst) and is
  named in `out_of_comparison`. The committed stamp carries no matched pair yet, so `docs/comparison.md`
  changes only in generated text.
- New graded comparisons and forced-red probes in `test/probe_gates.sh`: the cut build's differences
  from `hazard3_c`, and the bench's `FIT` moving Zifencei.
- Not tried: a read-only `mtvec`, `MULDIV_UNROLL` above 1, and any change to the bench's bus adapter.

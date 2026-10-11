# ADR-0251: No Hazard3 build with the C extension reaches the up5k's 12 MHz step; the closest is `hazard3_c` without its branch predictor

**Status:** Accepted · 2026-10-11 · *Builds on ADR-0160, ADR-0246 and ADR-0250. No `rtl/` change ships
from this ADR.*

## Context

ADR-0250 added a feature-matched RV32IMAC row. Its Hazard3 build, `hazard3_c` (the performance build
plus `EXTENSION_C`), places on the up5k at 11.01 / 11.65 / 11.93 MHz, worst / median / best of twelve
seeds, so it misses the 12 MHz step and leaves the up5k comparison. The owner asked for the closest
Hazard3 configuration that clears the step at the worst of twelve seeds, to put a Hazard3 row back in
that comparison.

The rules for "closest": the ISA stays RV32IMAC plus Zicsr and Zifencei (A, C, M and Zifencei never
come off), as few performance features as possible are removed or cheapened, single-knob changes come
first, and each candidate runs twelve paired seeds against a `hazard3_c` control from the same session.

## Decision

**No candidate clears the step at the worst of twelve seeds.** Seventeen configurations were swept
(table below); the closest by median and by number of cleared seeds is `hazard3_c` with
`BRANCH_PREDICTOR` off, which clears eleven of twelve placements and misses at one (seed 4, 11.82 MHz).
It joins the matched row as **`hazard3_c_nobp`** and is reported out of the up5k comparison, the way
`hazard3_c` is, with its cycles and its ECP5 clock published. Nothing in the up5k product changes: the
up5k comparison stays littlecpu against `vexriscv_lrsc`.

`bench_hazard3.v` gains a `NO_BP` parameter (set only with `PERF` and `WITH_C`) that turns
`BRANCH_PREDICTOR` off; `soc/compare/hazard3_builds.txt` has a fourth column for it, and
`soc/compare/hazard3_config_test.py` grades it as the C column plus `BRANCH_PREDICTOR` and nothing else.
`hazard3_c` and its ECP5 row are unchanged. `make compare-dhrystone-matched`, `make
compare-coremark-matched`, `make compare-smoke`, `soc/compare/run_product.sh`'s sweep and pairs, and
`soc/compare/comparison.py`'s matched section carry the new core; the weekly workflow needed no edit
because it names pairs, not cores.

Why the predictor and not another knob: it is the only single change that moves the median over the
step (+0.63 MHz, 11.64 to 12.27) and it costs the least cycles of the knobs that help at all (+3.1%
Dhrystone, +5.7% CoreMark against `hazard3_c`). `MUL_FAST` off recovers less (+0.29 MHz median) and
doubles CoreMark's cycles in return (the sequential multiplier, 652,706 against 351,710), so it is
never the right trade when cycles alone decide the up5k comparison. Why the predictor is a *near* miss
and not a fix: the placer's spread from worst to best placement on this core runs 0.6 to 1.2 MHz across
the table, so a median of 12.3 clears the step at most seeds and not all of them. Three combinations on
top of it reach the same eleven of twelve (adding `MTVEC_WMASK` direct; adding `MUL_FAST` off and
`CSR_COUNTER` off; adding `MUL_FAST` off and `MTVEC_WMASK` direct), and none reaches twelve; the best
worst case, 11.98 MHz, is the last of those, two hundredths of a MHz under the step at 8.7% more
Dhrystone and 91% more CoreMark cycles than the control. Taking a lucky draw of that family as "the
build that fits" would be reading the placer, not the design.

## Measurements

Tree `a470e8e` plus this change, xPack gcc 15.2.0, OSS CAD Suite 20260811 (yosys 0.68+48, nextpnr
0.11-1-g62e659e), one toolchain for every number. Clocks: twelve seeds a part (`default`, 1 to 11),
`make compare-timing`'s own flow with `COMPARE_SEED` / `ECP5_SEED`. Cycles: one image, one simulation
(`make compare-dhrystone-matched`, 400 runs; `make compare-coremark-matched`, one iteration); RAMs
identical to littlecpu's. The control reproduces ADR-0250 to the digit on the up5k (11.01 / 11.64 /
11.93, 234,427 and 351,710 cycles, 4,444 placed `ICESTORM_LC` of 5,280) and to 0.01 MHz on ECP5. Every
knob not named in a row is at `hazard3_c`'s value. The harness's disclosed bus wait (28,805 and 14,176
cycles) is in every row.

up5k, MHz worst / median / best, "cleared" is seeds at or above 12.00, cycles are Dhrystone / CoreMark:

| candidate | up5k MHz | cleared | placed LC | Dhrystone cycles | CoreMark cycles |
|---|---|---|---|---|---|
| `hazard3_c` (control) | 11.01 / 11.64 / 11.93 | 0 of 12 | 4,444 | 234,427 | 351,710 |
| `BRANCH_PREDICTOR` 0 (**`hazard3_c_nobp`**) | 11.82 / 12.27 / 12.72 | 11 of 12 | 4,113 | 241,627 | 371,634 |
| `MUL_FAST` 0 | 11.80 / 11.93 / 12.19 | 3 of 12 | 4,404 | 247,627 | 652,706 |
| `CSR_COUNTER` 0 | 10.73 / 11.26 / 11.91 | 0 of 12 | 4,042 | 234,427 | 351,710 |
| `REDUCED_BYPASS` 1 | 10.80 / 11.21 / 11.47 | 0 of 12 | 4,388 | 316,435 | 490,217 |
| `FAST_BRANCHCMP` 0 | 9.77 / 10.08 / 10.38 | 0 of 12 | 4,405 | 234,427 | 351,710 |
| `MTVEC_WMASK` `0xfffffffc` (direct mode only) | 11.36 / 11.80 / 11.91 | 0 of 12 | 4,419 | 234,427 | 351,710 |
| predictor 0, `MUL_FAST` 0 | 11.47 / 11.86 / 12.08 | 4 of 12 | 4,077 | 254,827 | 672,630 |
| predictor 0, `CSR_COUNTER` 0 | 11.62 / 12.04 / 12.30 | 7 of 12 | 3,705 | 241,627 | 371,634 |
| predictor 0, `MULH_FAST` 1 | 11.59 / 11.82 / 12.08 | 1 of 12 | 4,391 | 241,627 | 371,634 |
| predictor 0, `MTVEC_WMASK` direct | 11.96 / 12.20 / 12.44 | 11 of 12 | 4,126 | 241,627 | 371,634 |
| `MUL_FAST` 0, `MTVEC_WMASK` direct | 11.30 / 11.55 / 11.86 | 0 of 12 | 4,378 | 247,627 | 652,706 |
| `CSR_COUNTER` 0, `MTVEC_WMASK` direct | 11.32 / 11.69 / 11.86 | 0 of 12 | 4,024 | 234,427 | 351,710 |
| predictor 0, `MUL_FAST` 0, `CSR_COUNTER` 0 | 11.70 / 12.16 / 12.50 | 11 of 12 | 3,664 | 254,827 | 672,630 |
| predictor 0, `MUL_FAST` 0, `MTVEC_WMASK` direct | 11.98 / 12.36 / 12.50 | 11 of 12 | 4,107 | 254,827 | 672,630 |
| predictor 0, `CSR_COUNTER` 0, `MTVEC_WMASK` direct | 11.47 / 11.74 / 12.00 | 1 of 12 | 3,718 | 241,627 | 371,634 |
| predictor 0, `MULH_FAST` 1, `MTVEC_WMASK` direct | 11.46 / 11.80 / 12.02 | 1 of 12 | 4,478 | 241,627 | 371,634 |
| predictor 0, `MUL_FAST` 0, `CSR_COUNTER` 0, `MTVEC_WMASK` direct | 11.43 / 11.90 / 12.19 | 5 of 12 | 3,667 | 254,827 | 672,630 |

Every candidate places (4,500 `ICESTORM_LC` or fewer of 5,280). Cycles for the rows with `CSR_COUNTER`
or `MTVEC_WMASK` off equal the row without them; that was simulated for `MTVEC_WMASK` alone, for
predictor 0 with `CSR_COUNTER` 0 and with `MTVEC_WMASK`, and for the `MUL_FAST` 0 pair with
`CSR_COUNTER` 0, and is carried over to the rest because neither knob touches the pipeline's timing in
the benchmarks (both reach their PASS verdict without a counter, and `mtvec` is never written). `MULH_FAST` 1
leaves cycles unmoved because neither benchmark executes a `MULH`. `MTVEC_WMASK` `0xfffffffc` fixes
`mtvec` to direct mode; it is no performance feature and littlecpu has no vectored mode either, which
is why it was tried, and it moved nothing (median 12.27 without it, 12.20 with).

The shipped build, with the text that ships (`hazard3_c_nobp`, twelve seeds, measured again after the
bench was reduced to this one knob and identical to the exploratory run to the digit):

| core | up5k clock MHz | up5k placed LC | ECP5 clock MHz | ECP5 placed `TRELLIS_COMB` | Dhrystone cycles (DMIPS/MHz) | CoreMark cycles (CoreMark/MHz) |
|---|---|---|---|---|---|---|
| littlecpu | 12.92 / 13.28 / 13.54 | 4,536 | 38.17 / 39.64 / 41.03 | | 228,825 (0.995) | 359,505 (2.782) |
| vexriscv_lrsc | 21.26 / 22.09 / 23.23 | 3,533 | 53.82 / 56.92 / 58.38 | | 269,629 (0.844) | 437,545 (2.285) |
| hazard3_c | 11.01 / 11.64 / 11.93 | 4,444 | 39.36 / 40.95 / 42.05 | 4,721 | 234,427 (0.971) | 351,710 (2.843) |
| **hazard3_c_nobp** | **11.82 / 12.27 / 12.72** | 4,113 | 39.23 / 42.50 / 43.77 | 4,682 | 241,627 (0.942) | 371,634 (2.691) |

littlecpu's and `vexriscv_lrsc`'s clocks are ADR-0250's, same toolchain, not re-run. `hazard3_c_nobp`
takes 1.056x littlecpu's Dhrystone cycles and 1.034x its CoreMark cycles (1.031x and 1.057x
`hazard3_c`'s). Were it to run at the 12 MHz step it would deliver 11.31 DMIPS and 32.29 CoreMark,
against littlecpu's 11.94 and 33.38 and `vexriscv_lrsc`'s 10.13 and 27.43; it does not reach the step
at seed 4, so that is a hypothetical, not a row. On ECP5, at the worst placement, `hazard3_c_nobp` is 36.96
DMIPS and 105.56 CoreMark against littlecpu's 37.98 and 106.17 (0.973x and 0.994x); at the median it is 40.04
and 114.36 against 39.44 and 110.26 (1.015x and 1.037x). On ECP5 the predictor's removal moves the
median clock up by 1.55 MHz (3.8%) and the worst placement down by 0.13.

Standing flags: the Hazard3 ECP5 clock inherits the unpinned-nextpnr flag in CLAUDE.md; CoreMark's
cycles are simulated at 16 KB of ROM against a 4 KB placed one. The ECP5 sweep for the candidates other
than `hazard3_c_nobp` was not run: the step is an up5k question.

## What it changes

- The matched row has four cores, and a Hazard3 pair of builds that both sit under the step. The up5k
  product is unchanged; the ECP5 row gains a Hazard3 build that is about even with littlecpu on
  Dhrystone and ahead of it on CoreMark at the median.
- **The cost of C on this core looks like the fetch path.** The two worst paths read, both from
  predictor-off placements, run from the decompressor's output through the decode and the
  branch-target adder to the fetch address and back through the stall logic; the knobs that shorten the
  multiplier or the counters do not touch it, which fits their reading as noise next to the predictor.
  Two paths are a sample, not a census. `hazard3_perf` without C places at 12.23 / 12.82 / 13.57
  (ADR-0246) and clears the step; C costs it roughly 1.2 MHz, and the predictor is the only thing here that
  gives back half of it. Dropping `EXTENSION_ZIFENCEI` was not tried: it changes the ISA the row holds
  fixed.
- If the owner would rather have a Hazard3 build in the up5k comparison than hold the ISA, the next
  question is which of Zifencei, C or A comes off, not which timing knob.

## Consequences

- `run_product.sh` sweeps `hazard3_c_nobp` and writes it into the four matched pairs. On the up5k its
  worst placement misses the step, so the pair names it in `out_of_comparison` beside `hazard3_c`; the
  first weekly `make compare-product` stamps it, and a toolchain whose placement draws differ may put it
  inside the comparison without any edit here.
- The committed stamp still carries no matched pair, so `docs/comparison.md` changes only in the
  generated core note and section text; the figures above are this change's own.
- New graded comparisons and forced-red probes in `test/probe_gates.sh`: the no-predictor build's single
  difference from `hazard3_c`, and the bench's `NO_BP` moving exactly that parameter.
- Not tried: widening the seed set past twelve (a sixteen-seed draw of `hazard3_c_nobp` is the sample
  ADR-0113 asks for when a verdict rests on the tail), a read-only `mtvec`, `MULDIV_UNROLL` above 1, and
  any change to `bench_hazard3.v`'s bus adapter (ADR-0146's wait stays disclosed, not corrected).

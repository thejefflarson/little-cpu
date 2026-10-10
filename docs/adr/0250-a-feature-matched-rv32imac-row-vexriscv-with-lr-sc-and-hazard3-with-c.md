# ADR-0250: A feature-matched RV32IMAC row: VexRiscv with LR/SC and Hazard3 with C

**Status:** Accepted · 2026-10-10 · *Builds on ADR-0146, ADR-0160, ADR-0232, ADR-0244 and ADR-0246.
No `rtl/` change ships from this ADR.*

## Context

The headline comparison compiles one RV32IM image for every core. littlecpu carries A and C in
hardware, so in that row its atomic and compressed support is paid for in area and period and never
exercised. Two of the opponents' builds also lacked what littlecpu has: the stock Hazard3 builds have
no C, and the generated VexRiscv has C but no A. A comparison that charges one core for features
the others lack, then runs a program that uses none of them, answers a narrower question than the
one the owner asked: what does each core cost and deliver when it carries the same ISA?

## Decision

Add a feature-matched row beside the RV32IM row, not in place of it. Every core is compiled at
`-march=rv32imac_zicsr_zifencei` (littlecpu's own string without its Zkt claim, which no other core
has) by `make compare-dhrystone-matched` and `make compare-coremark-matched`, through the same
`run_dhrystone.sh` / `run_coremark_compare.sh`, `dhry_monitor.v` and RAM-equality check as every
other row. Two new named builds join littlecpu:

- **`vexriscv_lrsc`**: `GenLittleCpuCompare.scala`'s stock configuration with `DBusSimplePlugin`'s
  `withLrSc = true` (`GenLittleCpuCompareLrsc`, generated as module `VexRiscvLrsc` with
  `privateNamespace` so its two helper modules do not collide with the stock file's in one
  simulation). **It is Zalrsc, not all of A.** At the pinned SHA (`c4b2a55`) the no-cache data bus has
  `withLrSc` and no AMO option, there is no `AtomicPlugin` class, and `withAmo` exists only on the
  cached data bus, a different and larger configuration. The nine AMO instructions trap as illegal.
  Neither benchmark contains an atomic instruction, so this changes what the core carries and not
  what the row runs; the row says so, and the name says `lrsc`.
- **`hazard3_c`**: `hazard3_perf` with `EXTENSION_C = 1` (`bench_hazard3.v`'s new `WITH_C`).
  A is on in every Hazard3 build. This is a harness choice no Hazard3 example ships:
  `soc/compare/hazard3_builds.txt` has a third column for it and
  `soc/compare/hazard3_config_test.py` grades it as the performance column plus `EXTENSION_C` and
  nothing else, checks the bench's `WITH_C` moves exactly that parameter, and prints that no example
  backs it.

The stock `vexriscv`, `hazard3` and `hazard3_perf` builds are unchanged, and so is the RV32IM row.
`soc/compare/comparison.py` renders a "Feature-matched (RV32IMAC)" section from pairs named
`<benchmark>_imac[_ecp5]`, and its VexRiscv note no longer says "no C": the build sets
`compressedGen = true`.

`VexRiscvLrsc.v` is generated output, pinned by digest in `soc/compare/vexriscv_pin.mk` beside the
stock file. To regenerate: check out VexRiscv at `VEXRISCV_SHA`, copy
`soc/compare/vexriscv/GenLittleCpuCompare.scala` into `src/main/scala/vexriscv/demo/`, and run
`sbt "runMain vexriscv.demo.GenLittleCpuCompareLrsc"` under JDK 17 (sbt 1.6 and Scala 2.12 do not
load under JDK 27). The same flow reproduces the stock `VexRiscv.v` byte for byte
(`03ce8baf...`), which is how the toolchain was checked before the new file was trusted.

## Measurements

Tree `bba630b` plus later edits to comments, flag spelling, probes and the rendering script (no core,
bench or testbench source), xPack gcc 15.2.0, OSS CAD Suite 20260811 (yosys 0.68+48, nextpnr
0.11-1-g62e659e; the committed stamp's 20260930 is a different build), one tree and one toolchain
for every number below. `make compare-smoke` agrees across all six harnesses; RAMs identical across
the three cores in both benchmarks. Clocks: twelve seeds a part (`default`, 1 to 11), MHz worst /
median / best, the sweep `run_product.sh` runs, driven here by hand so the three cores ran in
parallel.

| core (build) | Dhrystone cycles (DMIPS/MHz) | CoreMark cycles (CoreMark/MHz) | up5k clock | ECP5 clock | up5k placed LC |
|---|---|---|---|---|---|
| littlecpu | 228,825 (0.995) | 359,505 (2.782) | 12.92 / 13.28 / 13.54 | 38.17 / 39.64 / 41.03 | 4,536 |
| vexriscv_lrsc | 269,629 (0.844) | 437,545 (2.286) | 21.26 / 22.09 / 23.23 | 53.82 / 56.92 / 58.38 | 3,533 |
| hazard3_c | 234,427 (0.971) | 351,710 (2.843) | **11.01 / 11.65 / 11.93** | 39.37 / 40.94 / 42.05 | 4,444 |

Ratios against littlecpu (above 1 is the opponent ahead):

| ratio | cycles taken, Dhrystone | cycles taken, CoreMark | Dhrystone ECP5 worst / median | CoreMark ECP5 worst / median | up5k |
|---|---|---|---|---|---|
| vexriscv_lrsc | 1.178x | 1.217x | 1.197x / 1.218x | 1.159x / 1.180x | 0.849x Dhrystone, 0.822x CoreMark, at 12 MHz |
| hazard3_c | 1.024x | 0.978x | 1.007x / 1.008x | 1.054x / 1.056x | out of the comparison |

**Hazard3 with C does not reach the 12 MHz step on the up5k at any of twelve placements** (11.01 to
11.93 MHz). The next step down is 6 MHz, so by the rule the comparison already holds (ADR-0160), it is
out of the up5k comparison, not slower in it. The control: `hazard3_perf`, without C, swept the same
session, places at 12.23 / 12.82 / 13.57 and clears at all twelve (ADR-0246's figures, reproduced to
the digit). C costs Hazard3 about 1.2 MHz at the median here, and that is the whole difference
between clearing the step and missing it. littlecpu clears it at 12.92 worst. If `hazard3_c` could
run at 12 MHz its cycle ratio would put it 2.4% behind on Dhrystone and 2.2% ahead on CoreMark; it
cannot, so the table prints cycles and no up5k product for it.

## What it changes

- **A costs no cycles, and C costs the opponents some.** littlecpu's cycles are its RV32IM figures
  (CoreMark's 359,505 is two cycles under the RV32IM 359,507). VexRiscv's are its rv32imc figures to
  the digit, 2.6% above its RV32IM row (262,827 and 426,430) and unmoved by LR/SC. Hazard3's C build
  takes 1.2% more Dhrystone cycles than `hazard3_perf` (234,427 against 231,626) and 1.0% more
  CoreMark cycles (351,710 against 348,144), with the same disclosed bus wait (28,805 and 14,176).
- **Against VexRiscv the matched row widens ADR-0246's gap.** VexRiscv takes 17.8% more Dhrystone
  cycles and 21.7% more CoreMark cycles than littlecpu (14.9% and 18.6% at RV32IM). On up5k the
  products are 0.849x and 0.822x at 12 MHz (VexRiscv's 22 MHz is unspendable above the step); on ECP5
  VexRiscv's clock outweighs the cycle gap.
- **Against Hazard3 the matched row is a draw on ECP5 and a forfeit on up5k.** `hazard3_c` is within
  0.8% of littlecpu on Dhrystone and 5.6% ahead on CoreMark at the median ECP5 placement, and it does
  not run on the up5k at 12 MHz.

Standing flags: the Hazard3 ECP5 clock inherits the unpinned-nextpnr flag in CLAUDE.md; CoreMark's
cycles are simulated at 16 KB of ROM against a 4 KB placed one.

## Consequences

- `run_product.sh` sweeps the two new cores and writes four pairs (`dhrystone_imac`,
  `coremark_imac` and their `_ecp5` forms). A core whose worst up5k placement misses the step leaves
  that pair and the pair names it in an `out_of_comparison` field, which `comparison.py` renders and
  validates; `product_write.py`'s refusal to credit such a core the step is unchanged. Without this
  the first weekly run would have stopped at `hazard3_c`'s first up5k seed.
- **The committed stamp does not carry these pairs.** It predates them and `soc/compare/product.json`
  is not hand-edited, so `docs/comparison.md` prints "Not measured: no pair stamped" in the new section
  and points here; the figures above are this change's own. The first weekly `make compare-product`
  writes them, and that refresh owes two hand edits it will be told about by a red `make test`: the
  four `dhrystone_imac` / `coremark_imac` lines in `soc/compare/CYCLE_FLOOR` (a stamped matched pair
  with no floor line is red, as the RV32IM pairs are), and the `docs/comparison.md
  rv32imac_zicsr_zifencei` count in `test/march_test.sh` (0 today, 4 once stamped).
  `.github/workflows/compare-product-schedule.yml` grades the new pairs against their own flags.
- `make compare-smoke` ran `vvp` on its first prerequisite, which became
  `hazard3-config-clone-test` when ADR-0246 added that prerequisite; it now runs the simulation.
- New graded comparisons and their forced-red probes in `test/probe_gates.sh`: the C build's single
  difference and the bench's `WITH_C` (`hazard3_config_test.py`), the LR/SC bench being the stock
  bench but for the module name (`vexriscv_path_test.sh`), the matched section's rendering, ratchet
  and `out_of_comparison` field (`comparison.py`).
- Not built: AMO support for VexRiscv. It needs the cached data bus or a new plugin, either of which
  is a different core from the one ADR-0246 holds every opponent to. If a VexRiscv with all of A is
  wanted, that is the first design question to answer.

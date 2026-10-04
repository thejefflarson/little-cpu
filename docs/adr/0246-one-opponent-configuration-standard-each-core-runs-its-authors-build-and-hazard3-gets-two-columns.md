# ADR-0246: One opponent-configuration standard: each core runs its authors' build, and Hazard3 gets two columns

**Status:** Accepted · 2026-10-03 · *Builds on ADR-0146, ADR-0160 and ADR-0244. No `rtl/` change ships
from this ADR.*

## Context

The harness applied two standards. VexRiscv ran its authors' performance configuration
(`GenLittleCpuCompare.scala`: `MulPlugin`, `DivPlugin`, `CsrPlugin`, hazard bypasses, compressed
decode). Hazard3 ran its authors' area configuration, copied from the iCE40 example
(`fpga_icebreaker.v`): `MUL_FAST=0` (a bit-serial multiply), `BRANCH_PREDICTOR=0`,
`CSR_COUNTER=0`, `EXTENSION_ZIFENCEI=0`. Both deviations favoured littlecpu, and CoreMark leans on
multiply: the published CoreMark ratio was Hazard3 at 1.854x littlecpu's cycles.

## Options

1. Both opponents in their performance configuration.
2. Both in their small-FPGA configuration (VexRiscv ships none).
3. Publish both columns where a core ships more than one, naming the build beside every ratio.
4. Disclose only.

## Decision: option 3 as a standard for any core, present and future

- **An opponent runs the configuration its authors ship for a part with room to spare.** VexRiscv's
  only build is its performance one. Hazard3's is what its two ECP5 examples agree on
  (`fpga_ulx3s.v`, `fpga_orangecrab_25f.v`): `MUL_FAST=1`, `BRANCH_PREDICTOR=1`, `CSR_COUNTER=1`,
  `EXTENSION_ZIFENCEI=1`, with `MULDIV_UNROLL=1`, `MUL_FASTER=0`, `MULH_FAST=0` as shipped.
- **A core that also ships a small-FPGA build keeps it as its own named column.** `hazard3` stays the
  iCE40 example's area build; `hazard3_perf` is new. Neither replaces the other, so no old number
  silently changes meaning.
- **The ISA belongs to the harness row, not to an opponent's tuning.** `EXTENSION_C` is 0 in every
  Hazard3 example, and the rows already fix C, A and M per ISA row.
- **Every published ratio names the build.** `soc/compare/comparison.py` labels each row
  (`hazard3 (area build)`, `hazard3_perf (performance build)`, `vexriscv (performance build)`),
  refuses a core in the stamp that has no label, and says when a stamped pair carries one build of a
  core that ships two.
- **The standard is graded, not asserted.** `soc/compare/hazard3_builds.txt` states both builds'
  parameters; `soc/compare/hazard3_config_test.py` checks `soc/compare/bench_hazard3.v` against it at
  `PERF=0` and `PERF=1`, that `PERF` moves exactly the parameters the authors' builds differ on, that
  the file names the pinned SHA, and, whenever the pinned clone is present, the file against the
  clone's example files. `make hazard3-config-test` runs offline on `make test`;
  `hazard3-config-clone-test` precedes every Hazard3 simulation. `test/probe_gates.sh` forces each
  direction red.
- **Not chosen:** the Embench note in Hazard3's repo lists `MUL_FASTER=1`, `MULH_FAST=1`,
  `MULDIV_UNROLL=2` for "all ISA options enabled". It is a specimen in a benchmark Readme, not a shipped
  build, and it enables extensions the harness row fixes. It is the first place to look if the
  performance column is ever challenged as too weak; it can only help Hazard3 further.

## Fit

Geometry unchanged (1,024-word ROM, 16,384-word RAM). `hazard3_perf` on up5k places 4,111 of 5,280
`ICESTORM_LC` (77%), 3 `ICESTORM_DSP`, 12 block RAMs, 2 SPRAMs; 0.99x of its own standalone
synthesis against the 0.80x floor in `placed_vs_synth.py`. On ECP5 `MUL_FAST` maps 3 `MULT18X18D`
(the area build maps 0), declared as `COMPARE_ECP5_EXPECT_DSP_hazard3_perf`. The Dhrystone and CoreMark
images fit as for the other cores.

## Measurements

Tree `6c2a2f1` on main `4a83e48`, xPack gcc 15.2.0, one tree and one toolchain for both halves.
Cycles from `make compare-dhrystone` / `make compare-coremark` (one image, one simulation, RAMs
identical across all four cores, `make compare-smoke` agreeing across all four). Clocks from twelve
seeds a part (`default`, 1 to 11) through `run_product.sh`'s sweep, MHz worst / median / best.

| core (build) | Dhrystone cycles (DMIPS/MHz) | CoreMark cycles (CoreMark/MHz) | up5k clock | ECP5 clock |
|---|---|---|---|---|
| littlecpu | 228,825 (0.995) | 359,507 (2.782) | 12.89 / 13.18 / 13.63 | 35.96 / 38.57 / 39.71 |
| vexriscv (performance) | 262,827 (0.866) | 426,430 (2.345) | 21.91 / 22.37 / 22.71 | 52.99 / 54.33 / 55.77 |
| hazard3 (area) | 252,026 (0.903) | 666,552 (1.500) | 14.31 / 14.51 / 14.98 | 49.04 / 52.12 / 54.61 |
| hazard3_perf (performance) | 231,626 (0.983) | 348,144 (2.872) | 12.23 / 12.82 / 13.57 | 45.64 / 49.29 / 50.99 |

Products. On up5k every core clears the 12 MHz step (`hazard3_perf` worst 12.23, a 1.9% margin), so
the product is the cycle ratio at 12 MHz. On ECP5 it is read at the worst and median placement.

| ratio against littlecpu | Dhrystone up5k | CoreMark up5k | Dhrystone ECP5 worst / median | CoreMark ECP5 worst / median |
|---|---|---|---|---|
| vexriscv (performance) | 0.871x | 0.843x | 1.283x / 1.227x | 1.242x / 1.188x |
| hazard3 (area) | 0.908x | 0.539x | 1.238x / 1.227x | 0.736x / 0.729x |
| hazard3_perf (performance) | 0.988x | **1.033x** | 1.254x / 1.263x | **1.311x / 1.320x** |

Two products here are derived from the cycle table and the sweep, not stamped: Hazard3's Dhrystone
pair (`run_product.sh` now writes it) and every `hazard3_perf` figure until the weekly re-take lands.

## What it changes

**The earlier CoreMark conclusion reverses.** Against Hazard3's performance build littlecpu takes
1.032x its CoreMark cycles (3.2% more), not 0.54x of them; on up5k Hazard3 is 3.3% ahead, and
on ECP5 it is ahead by 31 to 32%, because its clock is also higher. Dhrystone is level on cycles
(`hazard3_perf` takes 1.012x littlecpu's) and 1.2% behind on up5k. The 1.854x gap was the cost of
the bit-serial multiply and the absent predictor, which is what the area build is. Hazard3's bus
adapter wait (28,805 of Dhrystone's cycles, 14,176 of CoreMark's) is identical across both builds
and still counted against it, so the performance column is, if anything, understated.

## Consequences

- `soc/compare/product.json` is not hand-edited. The next weekly `make compare-product` takes the
  authoritative stamp: `run_product.sh` now sweeps `hazard3_perf` on both parts, writes it into the
  CoreMark pair, and adds both Hazard3 builds to the Dhrystone pair. Until then
  `docs/comparison.md` prints "Not stamped" under each Hazard3 table, and the figures above are this
  PR's own measurements. CLAUDE.md's comparison figures are the 11cc506 stamp and fold in at that
  re-take.
- `CYCLE_FLOOR` ratchets littlecpu only and is unaffected.
- A future core joins by adding its authors' build or builds to `CORE_LABELS` and to the harness, never
  by a configuration chosen here.
- Standing flag: the ECP5 clocks of Hazard3 inherit the unpinned-nextpnr flag in CLAUDE.md. The littlecpu
  clocks above are lower than the 11cc506 stamp's (35.96 against 37.38 MHz worst on ECP5) on a
  different tree and toolchain, which is why the ratios use this table's own row and not the stamp.

# ADR-0181: The cross-core product stamp is re-taken, fresh, on both parts

**Status:** Accepted · 2026-09-13

## Context

`soc/compare/product.json` was stamped at `122ef7bdd2b0`, `dirty: yes`, against
hx8k-era `ram_words` (512, since 16384) and `rv32i` Dhrystone CFLAGS (since
`rv32im`). Every `make compare-dhrystone` run printed its own `*** STALE:`
warning naming the drift, so the artifact quoted no number a reader could
trust. Separately, `product_check.py`'s `moved_paths()` asked git for every
path under `rtl/` or `soc/compare/` that differed from the stamped base, and
`soc/compare/product.json` is itself under `soc/compare/`: a run that wrote
the file then read its own write as evidence of staleness, so `make
compare-product` could never exit 0 and the weekly refresh workflow could
never open a PR. That defect closed before this change landed (`stale_reasons()`
and `moved_paths()` take the artifact's own path as a parameter and exclude
it with a `git diff ... :(exclude)<path>`, both directions probed in
`test/probe_gates.sh`). What remained was to actually re-take the measurement.

## What changed in the harness

`soc/compare/run_product.sh` now sweeps littlecpu, VexRiscv and Hazard3's
clocks on both up5k and ECP5 (`COMPARE_PRODUCT_PARTS`, default both) instead
of up5k alone, and CoreMark's product gains a second column: VexRiscv's
cycles are read out of `make compare-coremark`'s three-way row alongside
Hazard3's, the same way Dhrystone's pair already read VexRiscv's. An ECP5
number is a pair of its own — `dhrystone_ecp5`/`coremark_ecp5` — rather than
a schema change, so it is never merged with or averaged against up5k's
(ADR-0160, ADR-0171); `product_write.py`'s existing "a pair is any (target,
other-cores) list" design needed no edit to carry a third core or a fourth
pair. The scheduled workflow's staleness check now derives which pairs exist
from the freshly-written stamp itself, rather than hand-typing the
`dhrystone_ecp5`/`coremark_ecp5` naming convention a second time — a
`/simplify` review flagged the hand-typed copy as a place a future part could
go unchecked silently.

## The measurement

Toolchain: Yosys 0.68+48, nextpnr-ice40/nextpnr-ecp5 0.11-1-g62e659ed, icetime
(oss-cad-suite 20260811), Icarus Verilog 14.0, riscv64-elf-gcc 16.2.0. Base
`ef9dacc42773`, `dirty: no`. Twelve seeds a side (`default 1 2 3 4 5 6 7 8 9
10 11`), paired by seed, worst/median/spread quoted per CLAUDE.md's own
go/no-go convention.

| pair | core | worst MHz | median MHz | ratio (worst / median, against littlecpu) |
|---|---|---|---|---|
| dhrystone (up5k) | littlecpu | 12.61 | 12.98 | — |
| dhrystone (up5k) | vexriscv | 21.92 | 22.78 | 1.990× / 2.010× |
| coremark (up5k) | littlecpu | 12.61 | 12.98 | — |
| coremark (up5k) | vexriscv | 21.92 | 22.78 | 1.763× / 1.781× |
| coremark (up5k) | hazard3 | 14.00 | 14.39 | 0.723× / 0.722× |
| dhrystone_ecp5 | littlecpu | 32.01 | 33.70 | — |
| dhrystone_ecp5 | vexriscv | 52.91 | 54.91 | 1.892× / 1.866× |
| coremark_ecp5 | littlecpu | 32.01 | 33.70 | — |
| coremark_ecp5 | vexriscv | 52.91 | 54.91 | 1.677× / 1.653× |
| coremark_ecp5 | hazard3 | 48.88 | 50.39 | 0.994× / 0.974× |

At 12 MHz, the step every core here clears: littlecpu 9.39 DMIPS / 27.70
CoreMark, VexRiscv 10.75 DMIPS / 28.10 CoreMark, Hazard3 18.03 CoreMark.
These reproduce CLAUDE.md's already-quoted up5k figures to within rounding —
the cycle counts are unchanged (same RTL, same CFLAGS), only the clock stamp
was stale — and the ECP5 clocks (littlecpu 32.01/33.70, VexRiscv 52.91/54.91,
Hazard3 48.88/50.39) reproduce CLAUDE.md's own ECP5 sweep exactly. **The two
parts are never averaged or merged** (ADR-0160): up5k's product is cycles
alone once every core clears the 12 MHz step, and ECP5's Fmax publishes with
no ratchet, since `soc/bands.py` has no band for that part. **The risk ADR-0160
already named stands**: this whole up5k comparison rests on no core reaching
the next step, 24 MHz; VexRiscv's own up5k best-of-twelve is 23.65 MHz, 1.46%
under that step, so the quantised tie the up5k table shows is close enough
that a future VexRiscv change clearing 24 MHz would move it to a different
clock entirely, not a faster placement of the same one.

## Wall time

About 60 minutes: 72 placements (three cores, twelve seeds, two parts) plus
both benchmarks' cycle counts, run in the background while other work on this
ticket continued.

## Consequence

CLAUDE.md's cross-core paragraph cites this ADR in place of the sentence
recording the stamp as stale. `.github/workflows/compare-product-schedule.yml`'s
weekly re-take can now complete end to end and open a PR when a future
measurement moves; it has not been dispatched by this change (`contents:
write`, `pull-requests: write`) and running it is a post-merge step.

# 0232 — The cross-core product is re-stamped after the fetch refactor, and the fetch-loop dead ends are retired to one pointer

Status: Accepted · 2026-10-01

## Context

The fetch refactor merged (ADR-0208, ADR-0214, ADR-0221, ADR-0222; ADR-0220 re-derived the fit
budget and recorded the cross-core cycle halves). Nothing since had taken both factors of the
three-core product in one session: `soc/compare/product.json` was stamped at 7e36714d7f1e on
2026-09-14, and `CLAUDE.md`'s cross-core paragraph quoted figures from that stamp and from
older single-session sweeps (littlecpu 290,825 Dhrystone cycles, 32.01 MHz on ECP5). It also
cited eleven ADRs as fetch-loop dead ends in a pipeline the refactor replaced.

## The stamp

The weekly workflow (`.github/workflows/compare-product-schedule.yml`) was dispatched on `main`
at 11cc506 on 2026-10-01 and ran `make compare-product` to completion (run 36810886313, about
1 h 17 min on the CI pool), so no local fall-back stamp was taken. It opened a stamp-refresh PR
(#429) that carries `soc/compare/product.json` at base 11cc506; this ADR's PR does not touch
that file, and the figures below are that stamp's. The older refresh PR (#414, dated 2026-09-28)
stamps a tree that predates the refactor and is superseded by it.

Toolchain, as the stamp records it: xPack `riscv-none-elf-gcc` 15.2.0, yosys 0.69+158
(a1a0ad7d4), nextpnr 0.11.1-40 (eb4f15c3) for both parts, icetime from oss-cad-suite 20260930,
Icarus 14.0, x86_64. Twelve placements a part, seeds `default` and 1 through 11. Flags:
`-march=rv32im -mabi=ilp32 -O2 -std=c11 -ffreestanding -fno-tree-loop-distribute-patterns -Wall
-Wextra -Werror`, with `soc/compare/dhry_port.c`'s own byte loops for the string routines and
the harness's `soc/compare/dhry.lds` / `coremark.lds`. Dhrystone 400 runs, CoreMark one iteration.

**Cycles** (identical when re-run locally the same day on a different host and yosys build,
`make compare-dhrystone` and `make compare-coremark`):

| Benchmark | littlecpu | VexRiscv | Hazard3 | VexRiscv / littlecpu | Hazard3 / littlecpu |
|---|---|---|---|---|---|
| Dhrystone | 228,825 (0.995 DMIPS/MHz) | 262,827 (0.866) | 252,026 (0.903) | 1.149× | 1.101× |
| CoreMark | 359,507 (2.782 CoreMark/MHz) | 426,430 (2.345) | 666,552 (1.500) | 1.186× | 1.854× |

**Clocks, worst / median of twelve placements, and spread** (the stamp's `clock_mhz`):

| Part | littlecpu | VexRiscv | Hazard3 |
|---|---|---|---|
| up5k | 12.55 / 12.78 MHz (4.19%) | 21.93 / 22.69 (6.05%) | 13.85 / 14.26 (7.86%) |
| ECP5 | 37.38 / 38.93 MHz (9.59%) | 53.16 / 55.34 (9.11%) | 50.35 / 51.89 (8.35%) |

**Products.** Up5k quantises all three cores to the 12 MHz step, so it is the cycle ratio at
one clock. ECP5 is read at the worst placement.

| Part, benchmark | littlecpu | VexRiscv | Hazard3 |
|---|---|---|---|
| up5k, Dhrystone (DMIPS) | 11.94 | 10.39 (0.871×) | 10.84 (0.908×, derived) |
| up5k, CoreMark | 33.38 | 28.14 (0.843×) | 18.00 (0.539×) |
| ECP5 worst, Dhrystone (DMIPS) | 37.19 | 46.05 (1.24×) | 45.48 (1.22×, derived) |
| ECP5 worst, CoreMark | 103.98 | 124.67 (1.20×) | 75.54 (0.73×) |

The two derived cells are cycle factor times clock, computed here, because
`soc/compare/run_product.sh`'s Dhrystone pairs carry only littlecpu and VexRiscv.

The up5k order reversed against the 2026-09-14 stamp: littlecpu went from behind VexRiscv (1.155×)
on Dhrystone and close on CoreMark (1.043×) to ahead on both, on cycles alone, since the up5k
clock is the step. The ECP5 order did not reverse: VexRiscv's clock lead (1.42× at the worst
placement) is larger than littlecpu's cycle lead (1.15×), and the Dhrystone gap fell from 1.85×
to 1.24×. Hazard3's ECP5 clock reads 50.35 / 51.89 MHz, with the standing flag that the same RTL
has read 33.26 and 48.50 in earlier sessions; the stamp inherits that flag and does not resolve it.

**Pairwise rows.** `make compare-dhrystone` and `make compare-coremark` print them: littlecpu
reads 228,825 Dhrystone cycles at rv32im, rv32ima, rv32imc and rv32imac (images of 1,968 bytes
without C and 1,356 with, so the equality is a result and not one binary), and 359,507 CoreMark
cycles at rv32im and rv32ima against 359,505 at rv32imc and rv32imac. VexRiscv at rv32imc reads
269,629 (+2.59% over rv32im) and 437,545 (+2.61%). Before the refactor C cost littlecpu +1.10%
and +3.09% and VexRiscv +9.76% and +3.80%; the pairwise gap now widens (VexRiscv 1.178× and
1.217× littlecpu's cycles at rv32imc) rather than narrows. Their clocks are cited from the
three-way row, not re-measured.

**Block arithmetic**, as `make compare-dhrystone` and `make compare-coremark` print it: Dhrystone's
RV32IM image is 1,968 bytes of text and 10,572 of RAM, needs 4 `SB_RAM40_4K` and 2
`SB_SPRAM256KA` against the part's 30 and 4, and fits placed geometry with each core's own 4
blocks (8 of 30). CoreMark's is 10,648 bytes of text (22 block RAMs, 26 with the core's own),
which does not fit the placed 4 KB ROM, so its cycles are simulated at 16 KB of ROM.

## What limits the period

From local reports of the same tree: `make soc-timing` at the pinned seed 20382078 reads
75.51 ns (13.24 MHz), 21 LUT levels, 67.5% routing, and ends at a block RAM's read address.
`soc/depth/path_stages.py` charges 9 of the 21 levels to `rtl/imemory.v`'s window select over the
ROM's output registers, 6 to the core (`dx_out`, `x_redirect_q`, `x_redirect_target_q`), 1 to the
accessor, 1 to `rtl/csrs.v` and 4 to no stage. `make ecp5-timing` at its default seed reads
38.02 MHz (26.30 ns: 11.12 ns logic, 15.18 routing), from the ROM's output, 5.83 ns of
clock-to-output, to the same ROM's address pin. Both paths therefore still run ROM to ROM.
This is one placement a part and the worst-of-twelve ECP5 reading above is the number to quote.
Local `make soc-timing` places 5,084 of 5,280 `ICESTORM_LC`.

## The retired dead ends

ADR-0076, 0078, 0083, 0087, 0091, 0092, 0097, 0100, 0113, 0129 and 0175 priced candidates against
a fused decoder whose fetch address closed a loop through decode. Each now carries a pointer
amendment saying so and that its measurement stands as dated, and `CLAUDE.md` keeps one pointer
to the class. Four were read before being retired, and each carries a rule that outlives the
pipeline, so `CLAUDE.md` still cites it for that rule alone: ADR-0097 (a period is
spelling-dependent; the compressed decode stays closed for area), ADR-0113 (a cost that is a
variance needs sixteen seeds), ADR-0129 (a harness cannot reach inside an instance) and ADR-0078
(the 24 MHz step arithmetic). Their README rows gain a trailing clause and are not reflowed.

## Not claimed

That the ECP5 order will stay reversed from up5k's under a different yosys or nextpnr: the stamp
carries that toolchain and the Hazard3 flag. That the pairwise clocks apply to the pairwise
images: they were not re-placed. That the local reports above are the worst of twelve on either
part.

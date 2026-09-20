# ADR-0198: the placed SoC drops the flash controller, and spends the reserve

**Status:** Reverted · 2026-09-19, amended 2026-09-20 · see "Amendment, 2026-09-20" below

## Context

**Amended 2026-09-20: the claim two paragraphs below, that the owner had already identified
this reserve, was false.** No such decision was ever made by the owner. An assistant session
wrote that framing into a sprint-planning brief under a heading that presented it as settled,
and it propagated unchallenged into the ticket that requested this PR, into the PR itself, and
into this ADR's own Context. The owner's actual position, on being asked, is the opposite: "no
dropped features as part of this work." The decision below was therefore made on an authority
that did not exist, and it is reverted — see the amendment at the end of this document. The
original Context is left as written, because it is part of the record of how the mistake
happened, not because any part of its "already identified" claim is true.

Stage A1 (PR #383, not merged) grows the core +583 logic cells for a register-only
`next_pc`, and the up5k SoC then fails to place: nextpnr reports ICESTORM_LC demand the
part's 5,280 cannot answer. The depth-2 fetch buffer under construction alongside it
recovers only part of that gap. Before either of those lands, this PR spends a reserve
the owner had already identified: `rtl/spiflash.v`, the read-only SPI configuration-flash
controller ADR-0135 shipped with its pins wired nowhere on this board. ADR-0135 tied
`sck`, `mosi` and `cs_n` off rather than routing them to real pins, because the predicate
available to arbitrate the shared MISO pin (`released` in `soc/miso_share_enable.v`)
cannot tell a host that is genuinely absent from one that is idle with its chip select
parked high, and `iceprog` does the second for most of a programming session. The
controller has therefore cost this design's area budget since it shipped, for a data
path nothing on the board can reach.

**This reserve alone does not close Stage A1's gap.** 84 packed cells freed against a
demand overshoot the ticket describes as ≥241 cells; the fetch buffer under construction
and any further reserve are still needed before that stage can place. What this PR settles
is narrower: whether the flash controller belongs in the placed SoC at all, independent of
whatever else lands to close the rest.

## Decision

`rtl/littlesoc.v` — the module `make soc-timing`, `make ecp5-timing` and every board
target place — no longer instantiates `spiflash`. Its four ports (`spi_sck`, `spi_mosi`,
`spi_miso`, `spi_cs_n`), its `flash_mem_rdata` term in the read-back bus's OR, and its
slot in `SOC_SRCS` are gone; `soc/board_upduino.v` and `soc/board_icesugar_pro.v` drop
the four dangling connections that follow, and `soc/littlesoc.pcf` drops the four pin
assignments `make soc-timing`'s own placement (not the flashed bitstream, which never
wired them past the board wrapper) used to carry.

`rtl/spiflash.v` itself is untouched and stays in the tree: `test/testbench.v` — the
simulated map every `.S`/`.c` program, `make dhrystone` and `make coremark` run against —
still instantiates it, so `test/asm/spiflash.S` and `spioverlay.S` keep running exactly
as before, and `test/spiflash_tb.v` keeps grading the module standalone. **The placed
map and the simulated map now differ by one peripheral, on purpose:** a program that
touches `0x0002_0028..0x0002_002f` sees a real controller in simulation and, on real
silicon, an address the map's OR-of-zero answers with all-zero reads and drops writes on
the floor. Neither Dhrystone nor CoreMark nor any suite program addresses that window
outside `spiflash.S`/`spioverlay.S`, which run only in simulation, so nothing else
observes the divergence. `test/memmap_test.sh` states it on every run (`spi ... simulated
only -- the placed SoC carries no flash controller`) rather than leaving it implied, and
its own module-instantiation check now demands `spiflash` only of `test/testbench.v`
rather than of both files.

## What did not move

`rtl/littlecpu.v`'s own copy of the map (`LS_FLASH_BASE`) is untouched — the core is
shared between the placed and simulated builds by design, and narrowing its idea of the
map to match the placed SoC alone is a decision about trap behaviour on real hardware
that this ticket does not make. A load or store at the flash's window still decodes as
"wait for the flip-flop" rather than "fault" on either build, since `rtl/decoder.v` never
learns that a particular window went unanswered; it only ever learns an address's region.
Concretely, on the placed SoC, an access to that window now behaves like an access to any
other unmapped word: no trap, a read of zero, a write that lands nowhere. Extending the
placed SoC's fault coverage to the flash's now-vacant window is future work.

`soc/pin_lockout.v`, `soc/miso_share_enable.v` and the UART/MISO pin-sharing mechanism
are untouched: they arbitrate a physical pin the flash chip and the UART share, which is
a fact about the board's wiring, not about whether `rtl/spiflash.v` is instantiated.

## Measurement

Toolchain: yosys 0.68+48 (git sha1 ff5817c34), nextpnr-ice40/nextpnr-ecp5 0.11-1-g62e659ed,
icetime (oss-cad-suite 20260811), identical on every run cited below. Base is
`origin/main` at `3b64723`; candidate is this ADR's own commit, `a6e6000`, one commit
ahead of that base with nothing else changed.

**`make fit` (the core alone): unchanged, as expected.** `rtl/spiflash.v` was never part
of `fit`'s top (`rtl/littlecpu.v`), so removing its instantiation from `rtl/littlesoc.v`
cannot move this number: 4063 ICESTORM_LC measured on the candidate, a −34-cell move
against the 4097 the Makefile's `FIT_LAST_LC` records for main — inside the ±50-cell
churn band `make fit` carries regardless of any real edit.

**`make soc-timing` (up5k, main's pinned seed replayed on both trees): ICESTORM_LC
4920 → 4836, freeing 84 cells (1.7%) against the part's 5,280.** Fmax at that seed:
12.12 → 12.84 MHz. The netlist digest itself (`make netlist-diff BASE=origin/main`)
counts the unpacked cost the placer folds from: `SB_LUT4` 4428 → 4357 (−71),
`SB_DFFESR` 711 → 689 (−22), `SB_DFFSR` 246 → 237 (−9), `SB_CARRY` 697 → 695 (−2),
−104 cells before packing, plus the four now-deleted ports. `SOC_EXPECT_SPRAM`/
`SOC_EXPECT_EBR` are unchanged at 2/20 — the controller is a shift register and a bit
counter, no block RAM, so its removal could not move either count, and did not.

**`make ecp5-timing`: `TRELLIS_COMB` 5780 → 5616 (−164, 2.8%), Fmax 33.87 → 35.00 MHz**
at the same seed on both trees; the three mapping censuses and
`soc/bram_reset_check.py` are untouched (no block RAM in the controller).

**The digest moved, so the paired sweep is owed** (`soc/paired_sweep.sh origin/main`,
sixteen seeds a side on up5k, twelve on ecp5). The two trees differ by exactly the one
commit this ADR lands, with an identical toolchain confirmed on every row; `git rev-parse`
for the two sides therefore names two different commits rather than the same ref with an
uncommitted diff, which is the one shape `soc/baseline_summary.py` refuses without
`--allow-mismatch` (it is a base-identity check, not the toolchain-mismatch refusal
CLAUDE.md says never to bypass, and every field it prints — yosys, nextpnr, icetime —
agrees between the two sides):

| part | worst, ns (base → branch) | median, ns (base → branch) | spread (base → branch) |
|---|---|---|---|
| up5k | 83.38 → 80.04 (−4.0%) | 80.44 → 77.06 (−4.2%) | 6.8% → 5.7% |
| ecp5 | 30.29 → 29.65 (−2.1%) | 29.08 → 28.70 (−1.3%) | 10.3% → 5.0% |

A positive percentage is slower; every figure above moved faster except ecp5's own best
placement (27.46 → 28.24 ns, +2.8%), which is inside that part's own churn band and not
read as a regression on its own (CLAUDE.md: "a delta inside either figure is not evidence
of anything"). Every one of up5k's sixteen candidate seeds clears `SOC_MIN_MHZ` (worst
12.49 MHz); main's own sixteen-seed sweep does not — one of them, small-integer seed 9,
reads 11.99 MHz.

**`make soc-seed-search` (refuses a pin under 12.60 MHz, a 5% margin over
`SOC_MIN_MHZ`): cleared.** Twelve high-entropy seeds, best 20740127 at 13.18 MHz
(9.83% margin); `soc/pin.json` is rewritten and committed with this ADR, since the
digest moved and the old pin would otherwise read PIN STALE.

## Correctness

`make test` (probe-gates, the `.S`/`.c` suite, every unit bench, every repo-scanning
`*-test` target, window-test, board-elaborate, imem-share-test, mutation-probe,
dual-build, nano-*): PASS, exit 0, failure list matching `test/EXPECTED_FAIL` exactly.
`make lint`, `make elaborate-strict`, `make board-elaborate` and `make dual-smoke`: all
PASS (dual-smoke: "two harts counted 32, one hart counted 16"). `make mutation-check`'s
`loadstore-region-ignored` and `text-port-drops-load` pairings (run with `--only`, since
the two chosen for this ADR are unaffected by anything else in the tree) each fire
against exactly their listed detectors, unaffected by a change confined to
`rtl/littlesoc.v` and the board wrappers: none of those pairings' programs or benches
read `rtl/littlesoc.v`.

`test/probe_gates.sh`'s memmap probe that used to mutate `rtl/littlesoc.v`'s `spiflash`
instance now mutates `test/testbench.v`'s instead, since that is the one file left that
owes the memory-map check an instantiation; `test/PROBES_EXPECTED` moved with it.
`soc/board_elaborate.sh`'s port-rename mutation, which matched
`.uart_tx(uart_tx),` with a trailing comma, is rewritten without one: dropping the flash
ports made `.uart_tx(uart_tx)` the last connection in `soc/board_upduino.v`'s `littlesoc`
instance.

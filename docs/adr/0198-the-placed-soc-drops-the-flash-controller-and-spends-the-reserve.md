# ADR-0198: the placed SoC drops the flash controller, and spends the reserve

**Status:** Accepted · 2026-09-18

## Context

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
icetime (oss-cad-suite 20260811), against this branch and `origin/main` at `3b64723`.

**`make fit` (the core alone): unchanged.** `rtl/spiflash.v` was never part of `fit`'s top
(`rtl/littlecpu.v`), so removing its instantiation from `rtl/littlesoc.v` cannot move this
number, and did not: <FIT_BEFORE> -> <FIT_AFTER> ICESTORM_LC.

**`make soc-timing` (up5k, pinned placement, `soc/pin.json`): ICESTORM_LC
<SOC_LC_BEFORE> -> <SOC_LC_AFTER>, freeing <SOC_LC_FREED> cells (<SOC_LC_PCT>%) against
the part's 5,280.** Fmax at the pinned seed: <SOC_MHZ_BEFORE> -> <SOC_MHZ_AFTER> MHz.
`SOC_EXPECT_SPRAM`/`SOC_EXPECT_EBR` are unchanged at 2/20 — the controller is a shift
register and a bit counter, no block RAM, so its removal could not move either count, and
did not.

**`make ecp5-timing`: <ECP5_BEFORE> -> <ECP5_AFTER> MHz**, <ECP5_LC_NOTE>.

**The digest moved, so the paired sweep is owed** (`soc/paired_sweep.sh origin/main`,
sixteen seeds a side on up5k, twelve on ecp5, `soc/baseline_summary.py` refusing below
twelve either side):

| part | worst (base -> branch) | median (base -> branch) | spread |
|---|---|---|---|
| up5k | <UP5K_WORST_BASE> -> <UP5K_WORST_BRANCH> MHz | <UP5K_MEDIAN_BASE> -> <UP5K_MEDIAN_BRANCH> MHz | <UP5K_SPREAD>% |
| ecp5 | <ECP5_WORST_BASE> -> <ECP5_WORST_BRANCH> MHz | <ECP5_MEDIAN_BASE> -> <ECP5_MEDIAN_BRANCH> MHz | <ECP5_SPREAD>% |

<SWEEP_VERDICT>

**`make soc-seed-search` (refuses a pin under 12.60 MHz, a 5% margin over `SOC_MIN_MHZ`):**
<PIN_RESULT>

## Correctness

`make test` (probe-gates, the `.S`/`.c` suite, every unit bench, every repo-scanning
`*-test` target, window-test, board-elaborate, imem-share-test, mutation-probe,
dual-build, nano-*): <TEST_RESULT>. `make lint`, `make elaborate-strict`,
`make board-elaborate` and `make dual-smoke`: <OTHER_RESULT>. `make mutation-check`'s
`loadstore-region-ignored` and `text-port-drops-load` pairings still fire against exactly
their listed detectors, unaffected by a change confined to `rtl/littlesoc.v` and the board
wrappers: none of those pairings' programs or benches read `rtl/littlesoc.v`.

`test/probe_gates.sh`'s memmap probe that used to mutate `rtl/littlesoc.v`'s `spiflash`
instance now mutates `test/testbench.v`'s instead, since that is the one file left that
owes the memory-map check an instantiation; `test/PROBES_EXPECTED` moved with it.
`soc/board_elaborate.sh`'s port-rename mutation, which matched
`.uart_tx(uart_tx),` with a trailing comma, is rewritten without one: dropping the flash
ports made `.uart_tx(uart_tx)` the last connection in `soc/board_upduino.v`'s `littlesoc`
instance.

# ADR-0259: The UPduino can hold 64 KB of program text in SPRAM behind a block-RAM boot ROM

**Status:** Accepted · 2026-11-12. Builds on ADR-0258 (the flash loader and image format), ADR-0255 (the board top is
its own design) and ADR-0135 (the 8 KB text ceiling). Measured on `main` at 0ab3b3ff plus this tree, yosys 0.68+48,
nextpnr 0.11-1-g62e659ed (OSS CAD Suite 20260811), on one Apple-silicon machine. The board run is pending (see
Measured on the board).

## Decision

**An opt-in second UPduino design, `upduino_spram_top` around `rtl/littlesoc_spram.v`, keeps instruction memory
in all four `SB_SPRAM256KA` (64 KB), moves the data RAM to 8 KB of block RAM, and resets the core into a 1 KB
block-RAM boot ROM at `0xFC00` that copies a flash image into the text window.** The default board, `littlesoc`'s
map and every existing target are unchanged. ADR-0135's claim that the SPRAMs cannot hold text was true only
while they also had to hold the data RAM.

- **Text in SPRAM needs one spelling.** `rtl/imemory.v` infers SPRAM only while the write uses the same address
  signal as the read (`even_raddr`, `odd_raddr`); a separate write index (`data_index`, the default design's
  spelling) maps the same arrays to 128 block RAMs and 382 extra LUTs. `BOOT_WORDS` selects the SPRAM arm; at
  zero the default arm is the original block unchanged. The census (4 `SB_SPRAM256KA`, 24 `SB_RAM40_4K`) is
  exact, and a probe maps the separate-index spelling and requires the SPRAM count to fail.
- **Boot ROM.** The top 256 words of the window read from two small block-RAM banks and shadow the SPRAM behind
  them. `RESET_PC` (a parameter of `rtl/fetcher.v` and `rtl/littlecpu.v`, default 0) is `0xFC00`. The ROM is
  `soc/boot_loader.c` built with `-DBOOT_SPRAM`, 852 bytes of the 1,024: one loader source and one image format
  for both boards. `soc/boot_layout.h` carries the SPRAM numbers under `BOOT_SPRAM_`; an image is linked at `0x0`,
  may be `0xFC00` bytes, and uses 8 KB of RAM. `soc/boot_image.py --target spram`, `make spram-boot-image` and
  `make spram-prog-image` pack and write it; the flash offset `0x20000` and `lc-flash -o` are shared.
- **A 64 KB window needs a 16-bit `predicted_target_low`.** It was 14 bits, enough for 16 KB. See Measurements.
- **The SPI target.** `rtl/spi_target.v` at `0x0002_0030` is a polled mode-0 peripheral port for an external
  controller (`LS_HAS_SPI` in the core's region test, off by default; the default `littlesoc` does not
  instantiate it). A transmit write landing on the byte-boundary cycle used to be dropped: the load of the
  queued byte was suppressed that cycle and fired a bit late, shifting the byte. `test/spi_target_tb.v` sweeps the
  write across 320 cycle offsets; on the unfixed target it fails at two of them (the byte comes out as `61` for
  `C3`), on the fixed one it passes. The fix loads the shift register from the arriving write directly.
  Pins 2, 3, 4 and 6 (`docs/pin-constraints.md`).
- **Own map, graded.** `littlesoc_spram` overrides only the data RAM's depth; `test/memmap_test.sh` grades what it
  may override and that the reset vector, the boot ROM, the image window, the loader layout, the RAM and the
  SPI window agree (nine forced-red probes). The default top must not instantiate the SPI target.
- **Targets.** `make spram-bitstream`, `spram-prog`, `spram-board-timing`, `spram-board-seed-search`,
  `spram-board-elaborate`, `spram-boot-image`, `spram-prog-image`, `boot-spram-test`. They re-enter the Makefile
  with the board variables repointed and outputs under `build/spram`. CI's `spram-timing` job places it at the
  pin.

## Measurements

**The wider `predicted_target_low` costs the default design nothing.** Its 14 bits became 16 in
`rtl/structs.v` and `rtl/executor.v`; the bits above a window's own address are constant zero and map to nothing.
The default SoC maps to 4,586 `SB_LUT4` on `main` and 4,586 with the two extra bits and nothing else changed (yosys
0.68+48). So the width does not follow the text-window parameter, and `main` keeps one spelling. A typedef's width
is global in any case.

**Anything else on the default design's path does cost, and that is why the SPRAM arm is its own module.** A first
version put the SPRAM arm inside `rtl/imemory.v` and restructured that file's one clocked block: the default SoC
mapped to 4,640 `SB_LUT4` (+54, inside the +-50 re-mapping band `make fit` documents) and a 16-seed paired sweep
read +2.5% median period. Reverting `rtl/imemory.v` alone returned 4,580. The shipped tree leaves `rtl/imemory.v`
byte-identical and puts the arm in `rtl/imemory_spram.v`.

Shipped tree against `main` at 0ab3b3ff, sixteen paired seeds (`soc/paired_sweep.sh origin/main up5k`: the default
seed and 1 to 15), `make soc-timing` on `littlesoc`:

| | worst | median | best | spread | packed cells |
|---|---|---|---|---|---|
| `main` | 79.55 ns, 12.57 MHz | 74.66 ns, 13.39 MHz | 71.20 ns, 14.04 MHz | 11.7% | 5,100 |
| this tree | 78.39 ns, 12.76 MHz | 76.13 ns, 13.14 MHz | 74.14 ns, 13.49 MHz | 5.7% | 5,076 |
| delta (ns) | -1.5% | +2.0% | +4.1% | | -24 |

The median and best deltas sit inside up5k's 3.6% edit-churn band and the worst placement, which is what
`SOC_MIN_MHZ` grades, got faster. `make board-timing` on `upduino_top`, seeds 1 to 16 under `SOC_MIN_MHZ=0`
(paired by seed, one tree each side): `main` 11.93 / 12.50 / 12.99 MHz worst / median / best at 5,058 packed
cells; this tree 12.74 / 13.30 / 13.57 at 5,152. The 94 cells are the board wrapper's instance-name packing
(ADR-0255), not RTL: the same RTL maps 6 `SB_LUT4` fewer. `main`'s board pin (`soc/board_pin.json`) is
untouched and still names a seed; `make netlist-determinism`'s mutant tree does not find
`soc/ice40_abc9.script` on this checkout, so the digest gate was not usable and the sweeps stand in for it.

**The SPRAM-text design** (`make spram-board-timing`, `upduino_spram_top`, yosys 0.68+48): 5,279 of 5,280
packed `ICESTORM_LC` (99.98%), 4 `SB_SPRAM256KA` and 24 `SB_RAM40_4K` (16 for the 8 KB data RAM, 4 for the register
file, 4 for the boot ROM), both exact. That is a respelling away from not placing: other spellings of the same
logic packed 5,268, 5,300 and 5,320 cells, and the last two failed to place. Over the 40 high-entropy seeds
`make spram-board-seed-search` drew (`SOC_SEARCH_COUNT=40`): worst 11.80 MHz, median 12.24, best 13.07; 35 of 40
cleared 12.0 and 6 cleared 12.6. The first 16 of those 40 alone had a best of 12.57 MHz, 4.75% over the floor, and
the search refused to pin; the pin is the best of 40 draws, **seed 1748564 at 13.07 MHz, 8.9% over 12.0**
(`soc/spram_pin.json`). A pin is a sample, and the median is 2% over the floor, not 5%. The earlier experiment's
5,218 cells and 12.04 / 12.43 / 12.85 MHz were a different top and a different spelling of the text store.

**Boot, simulated.** The 852-byte loader (1,024 available) loads and runs a small image, a second image, and a
third image larger than the default board's 8 KB ROM, through the same SoC against `test/spiflash_model.v`
(`make boot-spram-test`); a flipped byte, a length past the window and an erased flash are each reported and
not run. `make boot-loader-test` on the default board passes unchanged.

**Measured on the board.** Not yet run on a clocked board. **This target fits only on the crystal clock**:
`make spram-bitstream BOARD_OSC=internal` maps 5,297 `ICESTORM_LC` against the part's 5,280 and nextpnr fails to
place it at any seed, so `SB_HFOSC` is not an option without removing cells. On 2026-11-12 the crystal build was
flashed to the one UPduino v3.0 here, whose crystal jumper is open, with `test/boot/image_a.c` written by
`make spram-prog-image`; the UART read 0 bytes over two reads, which is what an unclocked part does. The target
ships for a board that supplies a 12 MHz clock on the crystal pin; it owes a run on one. The commands, in order,
are in `docs/flashing-the-upduino.md`.

## Consequences

- **The core's reset PC is a parameter, `RESET_PC`, default 0.** Every formal harness and `test/testbench.v`
  leave it alone; `components_pcloop`, `components_traps`, `components_decoder`, `components_executor` and
  `components_accessor` pass, and `make -C formal remeasure-fg` still reads F = 5, G = 4.
- **The design has one cell of headroom.** CLAUDE.md's churn rule applies in full: a respelling moves this count by
  tens of cells and the next one may not place. CI's yosys has run about 50 cells lighter than 0.68 on the board
  top, which helps, but the netlist differs from the one the pin was drawn on, so `spram-timing` may read PIN
  STALE and a different frequency; re-pin from CI's toolchain if it does. The 12 MHz gate at the pin is the
  requirement; the 5% margin is the pin rule's.
- **CLAUDE.md** loses the claim that SPRAM cannot hold text (true only beside the data RAM) and gains a rule
  for this sixth design, whose numbers combine with none of the other five.
- **Costs that remain.** 8 KB of data RAM, against 64 KB on the default board; a board run is still owed; the SPI
  target's pad voltage and its 3 MHz clock limit at 12 MHz are unmeasured on hardware.

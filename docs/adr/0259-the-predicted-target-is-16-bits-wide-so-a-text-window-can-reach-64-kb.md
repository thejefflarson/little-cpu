# ADR-0259: The predicted target is 16 bits wide, so a text window can reach 64 KB

**Status:** Accepted · 2026-11-12. Builds on ADR-0221 (D publishes a static BTFN/`jal` guess and fetch follows it) and
ADR-0135 (the 8 KB text ceiling on the up5k, which this does not move). Measured on `main` at 0ab3b3ff plus this
tree, yosys 0.68+48, nextpnr 0.11-1-g62e659ed (OSS CAD Suite 20260811), on one Apple-silicon machine.

## Decision

**`dx_output.predicted_target_low` is 16 bits, with no parameter, so an integrator can build a text window of up to
`LS_TEXT_WORDS` 16384 (64 KB) around the unmodified core.** It was 14 bits, which bounded the window at 16 KB:
`rtl/decoder.v` sizes the guess to `LS_TEXT_ADDR_BITS + 2` low bits of the target and carries them to
`rtl/executor.v`, which rebuilds the guessed pc from the issuing pc's upper bits and those low bits to decide whether
fetch followed a miss. `rtl/littlecpu.v` refuses a window that needs more than 16 bits with an elaboration `$fatal`
(it refused more than 14 before), and `make window-test` forces that refusal red for `LS_TEXT_WORDS` 32768 and
accepts 16384.

**The field is a fixed width because a packed typedef's width is global.** `dx_output` is one type shared by the
decoder, the executor and every harness; making its width follow `LS_TEXT_WORDS` would need a parameter through every
port of every module that carries it. The bits above a window's own address are constant zero in the decoder and map
to nothing, so the default design pays no cells for them (Measurements).

## Test

`make window64-test` (`test/window64_test.sh`, `test/window64/prog.S`, `test/window64/sim.v`) runs `rtl/littlesoc.v`
at `ROM_WORDS` 16384 under iverilog. The program is 49,172 bytes of text. A `jal` at `0xc` targets `0xc010` and one at
`0xc010` targets `0x3ff8`, so both targets differ from their pcs in bits 15:14; a loop whose `bnez` sits at `0x4000`
branches back to `0x3ff8`, across the 16 KB boundary. It stores 12 to RAM's first word after four passes, and the
bench ends on that store, a trap or a 4,000-cycle budget. It reads 12 at cycle 37.

Its forced-red direction is in `make probe-gates`: the field narrowed back to 14 bits in `rtl/structs.v` and
`rtl/executor.v`, with the `$fatal` bound raised so the window elaborates anyway. The fetch address goes undefined
after the `jal` at `0xc010` (the executor's rebuilt guess is missing bits 15:14) and the bench reports `TIMEOUT`; the
probe requires that text, not a build error. The control probe runs the same fixture unmutated and requires a pass.

## Measurements

**The default design does not change.** Same `main` against this tree, both with the two extra bits on a window that
never uses them:

| | worst | median | best | spread | packed cells |
|---|---|---|---|---|---|
| `make soc-timing`, `main`, 16 paired seeds (default, 1 to 15) | 79.55 ns, 12.57 MHz | 74.66 ns, 13.39 MHz | 71.20 ns, 14.04 MHz | 11.7% | 5,100 |
| same, this tree | 79.55 ns, 12.57 MHz | 74.66 ns, 13.39 MHz | 71.20 ns, 14.04 MHz | 11.7% | 5,100 |
| `make board-timing`, `main`, seeds 1 to 16, `SOC_MIN_MHZ=0` | 83.83 ns, 11.93 MHz | 80.05 ns, 12.49 MHz | 76.96 ns, 12.99 MHz | 8.9% | 5,058 |
| same, this tree | 83.83 ns, 11.93 MHz | 80.05 ns, 12.49 MHz | 76.96 ns, 12.99 MHz | 8.9% | 5,058 |

All 32 placements agree to the hundredth of a nanosecond, and the placed bitstreams (`.asc` less its `.sym` net-name
lines, which shift by two `dx_out` bit indices) are identical on both parts' seed 1 and default seed and on the board
top. `make fit` reads 4,403 of 4,462 cells on `main` and on this tree, locally. `make netlist-diff` could not run: its
mutant tree does not find `soc/ice40_abc9.script` on this checkout (on `main` too), so the digest gate is void and
the paired sweeps and the identical bitstreams stand in for it.

**ECP5 moves a little, inside its band.** `soc/paired_sweep.sh origin/main up5k ecp5`, twelve seeds (default, 1 to
11), `make ecp5-timing`'s flow:

| | worst | median | best | spread | `TRELLIS_COMB` |
|---|---|---|---|---|---|
| `main` | 26.52 ns, 37.71 MHz | 26.11 ns, 38.30 MHz | 25.22 ns, 39.65 MHz | 5.2% | 6,458 |
| this tree | 26.65 ns, 37.52 MHz | 25.89 ns, 38.62 MHz | 24.59 ns, 40.67 MHz | 8.4% | 6,383 |
| delta (ns) | +0.5% | -0.8% | -2.5% | | -75 |

The ECP5 flow maps a different netlist (75 fewer `TRELLIS_COMB`), unlike the up5k flow; the worst placement is
within the 10.3% placement spread `soc/bands.py` records for the part, and the ECP5 flow has no ratchet on frequency.

**The pins.** `make soc-timing` and `make board-timing` both print PIN STALE (the sources moved under
`soc/pin.json` and `soc/board_pin.json`). Both still place at their pinned seeds and clear 12.0 MHz: 13.65 MHz for
`littlesoc` and 12.73 MHz for `upduino_top`. The placement is bit-identical to `main`'s, so the pins describe the
same design; neither is re-pinned.

## Consequences

- **The window can reach 64 KB and no further.** A 128 KB window would widen the field again and re-pay this
  measurement. `rtl/littlecpu.v` refuses it by name.
- **Every harness that builds a `dx_output` sees 16 bits.** `test/executor_tb.v` drives the field with 14-bit
  literals that zero-extend, which is correct for its 4 KB window; `test/decoder_tb.v` compares the same values.
  `components_decoder`, `components_executor` and `components_traps` read the field through the type.
- **This ADR does not make the up5k hold more text.** ADR-0135's ceiling is the fetch loop's block RAM, not this
  field; an integrator who builds a larger window on another part owns that part's memory.

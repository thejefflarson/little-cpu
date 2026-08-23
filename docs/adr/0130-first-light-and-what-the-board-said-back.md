# 0130 — First light, and what the board said back

Status: accepted · the first measurement in this repo that no tool produced

## Context

Every number here has been a model. `make fit` counts cells yosys reports,
`make soc-timing` reads icetime's opinion of a placement nextpnr wrote, the
formal checks grade a netlist against assertions, and `make dhrystone` counts
cycles a simulator advanced. Nothing had ever been configured into a part. There
was no target that could: the flow stopped at the `.asc`, with no `icepack` and
no `iceprog` anywhere in the Makefile.

ADR-0129 closed the last conformance gap and quoted a period. It, like all of
its predecessors, was a claim about what three programs believed.

## What ran

`make bitstream` packs the placement; `make prog` flashes it. The board is an
**UPduino v3.0** — iCE40UP5K in SG48, so the SPRAM the data RAM needs exists —
and `soc/board_upduino.v` carries the three things it does not share with the
iCEBreaker that `soc/littlesoc.pcf` was written for: no user button, an RGB LED
on dedicated pins, and a crystal that reaches the FPGA only through a solder
jumper.

`test/bench/firstlight.S` blinks the green LED and prints an incrementing
counter over the UART. It lives outside `test/asm` deliberately: both sim legs
glob that directory and hold every file in it to a 5000-cycle limit, so a program
that blinks forever would be a suite failure rather than a program.

**It runs.** The counter decodes digit-perfect, which means the core executed
`divu` and `remu` to extract decimal digits, polled the UART's status register,
and drove the transmit path — on silicon, from flash, with no host attached.

## The clock, measured two ways

Neither instrument is a tool's opinion, and they were built to be independent.

**The blink is a cycle count.** One blink is 15,005,261 cycles, derived by
building the same source with a blink limit and reading cxxrtl's cycle counter
at two delay lengths — 223,398 at `DELAY_ITERS=2000` and 423,437 at 4000, which
separates 5.0010 cycles per delay iteration from 2336 cycles of per-blink
overhead. Predicting the 1000 point from those gives 123,378 against 123,398
measured, 0.02% out, so the model is the loop rather than a fit.

**The baud is a divisor.** `rtl/uart.v` divides the clock by 104 for 115200, so
the wire rate moves with the oscillator: `wire_baud = 115200 * f / 12e6`. Sweeping
the host rate finds a plateau where 8N1 still decodes, and the plateau's centre
is the board's rate.

| | reading | implied clock |
|---|---|---|
| baud plateau, clean 110000–120000 | centre 115000, half-width 5.2% | **11.98 ± 0.11 MHz** |
| 5 counter lines per 6.2 s window | ~1.24 s per blink | **~12.1 MHz** |

`SB_HFOSC` is therefore **nominal within measurement error**, not the ±10% its
datasheet allows at the worst case — so 115200 reads 100% clean with the crystal
still disconnected, and R16 does not need shorting on this board.

**Static timing placed this design at 12.94 MHz and the part runs it at ~12.0.**
The model was not contradicted. That is the whole result: not that the numbers
were impressive, but that hardware was finally in a position to disagree with
them and did not.

## What this does not claim

**Not that the design meets 12 MHz on silicon.** It ran correctly at whatever
`SB_HFOSC` supplies, which measured just under 12. A part clocked at exactly
12.00 from the crystal has not been tried, and one board is one sample of one
process corner at one temperature.

**Not that the suite passes on hardware.** `firstlight.S` exercises the divider,
a device register, the UART and the fetch path. The other 73 programs have run
only under cxxrtl and iverilog. There is no HTIF on a board, so a `.S` program's
verdict has nowhere to go — reading one back is unbuilt work, not a result.

**Not a timing measurement.** The blink and the baud both measure the CLOCK, not
the critical path. Nothing here says what frequency the design would fail at.

## Consequences

- **`make bitstream` and `make prog` are not graded and carry no ratchet.** There
  is nothing to compare a flashed board against yet, and inventing a number would
  be inventing one. They are also a separate flow from `soc-timing`, which places
  a different top with different pins — do not merge their frequencies.
- **The next thing worth building is a verdict path**, so the `.S` suite can be
  run on the part and its result read back. The UART is the obvious carrier.
- **An external measurement is now possible at all**, which no ADR before this one
  could say.

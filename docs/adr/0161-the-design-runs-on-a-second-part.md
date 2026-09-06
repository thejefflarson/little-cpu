# ADR-0161: The design runs on a second part

**Status:** Accepted · 2026-09-05

## Context

`make ecp5-timing` has placed and timed `rtl/littlesoc.v` on an ECP5 since
ADR-0110, and `make dual-ecp5-timing` places the dual top there too. Neither
produced a bitstream: there was no `ecppack` anywhere in the Makefile, and no
board file for any ECP5 board. The one-source-two-parts claim rested entirely on
a placement, and a placement is what the tools think.

## What ran

A MuseLab iCESugar-Pro — **ECP5 LFE5U-25F, caBGA256, speed grade 6, 25 MHz on
P6** — flashed by copying `icesugar.bit` onto the iCELink volume its on-board
DAPLink debugger presents over USB.

`soc/blink.S` counts and publishes two bits of the counter with a store.
`rtl/littlesoc.v` latches the low two bits of the last store to any address onto
its LEDs, so the observed pattern — off, green, red, both — **says the core is
executing**, not that the clock is ticking. A hardware counter would blink just
as happily with a broken datapath; a four-state cycle needs the loop, the shift,
the mask and the store to all work.

nextpnr reports **34.77 MHz against the 25 MHz constraint**, so the part has
margin at the board's real clock.

## What this settles, and what it does not

**Settled: one source places, routes and runs on two parts.** No `ifdef`, no
fork, no part-specific RTL — the board file is the only thing that knows which
board this is, which is the same seam `soc/board_upduino.v` occupies. That was
the claim `make ecp5-timing` could only ever half-support.

**Not settled, and deliberately so:**

- **The UART is not routed.** This board's documentation states a USB-CDC serial
  port reaches the FPGA but does not say on which pins, and nextpnr places an
  unconstrained output on whatever pad it likes — on a real board that is a pin
  driving something. It stays internal until a pin is known.
- **The UART would be at the wrong baud anyway.** `rtl/uart.v` derives its
  divisor from a `CLOCK_HZ` parameter that defaults to 12 MHz, and
  `rtl/littlesoc.v` has no parameter to override it, so at 25 MHz it would emit
  near 240000 baud. A serial pin needs `CLOCK_HZ` threaded through the SoC
  first, which is an `rtl/` change owing `make fit` and a digest check.
- **The timer's timebase doubles.** `mtime` ticks once per clock cycle, so
  anything that assumes 12 MHz — `test/asm/mtimer.S`, Dhrystone's own timing —
  is wrong here until told otherwise.
- **The clock is assumed, not measured.** 25 MHz is the board's stated crystal.
  The UPduino's `SB_HFOSC` was quoted at ±10% and measured nominal (ADR-0130),
  which is exactly the kind of inherited number this repo does not trust. Timing
  the blink against its simulated cycle count is the cheap measurement, and it
  is the one the UPduino used.
- **Nothing is graded.** `make icesugar-bitstream` is a developer command with
  no ratchet, like `make bitstream`. Its frequency constraint is the board's
  real 25 MHz rather than `ecp5-timing`'s deliberately-unreachable 200, because
  the question is "does it run" and not "how fast could it".

Its numbers are also **not comparable to `make ecp5-timing`'s**: caBGA256
against caBGA381 is a different pinout and a different placement problem on the
same die.

## Consequence

The second-part claim is now a fact about silicon rather than about a placer.
That matters for the ECP5 work specifically — the dual-hart configuration lives
there because two fetch windows are 32 block RAMs against the up5k's 30, and any
instruction-set extension is affordable there and is not here (ADR-0156's
budget). Both of those were arguments about a part nothing had run on.

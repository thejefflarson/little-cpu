# ADR-0204: QSPI's read capture moves to SCK's falling edge, doubling its round-trip budget

**Status:** Accepted · 2026-09-20

## Context

ADR-0202 shipped `nano_qspi_ctrl` with its pad-mux compensation deliberately removed: the
deleted `IN_CAPTURE_STAGES` parameter registered `sio_in` before the read-side shift register
consumed it, but the shift register's own gating never moved, so every added stage sampled an
*earlier*, not-yet-driven window instead of a later, settled one — measured there as
`IN_CAPTURE_STAGES=0` passing the chained-resume reproduction and `=1` failing it with a
tri-stated `z` read back as data. That ADR left the real question open: does the controller
need compensation at all, and if so, what, argued from real numbers rather than "one stage
looks right".

## What "argued from sky130 pad timing" turned out to mean

This repository has no sky130 I/O-cell timing model anywhere in it, and no tool available to
build one: `make nano-area`, `make nano-timing` and every other local instrument synthesize
against `sky130_fd_sc_hd`, the **standard-cell** liberty (`nano/nano.mk`'s
`NANO_LIBERTY`) — `sky130_fd_io`, the physical pad library, and Tiny Tapeout's own chip-wide
user-project mux are absent from this tree entirely, and fetching either is the class of
dependency ADR-0169 already declined for a local instrument. So a specific "the pad plus the
mux cost N.N ns" figure, cited to a datasheet, is not obtainable here without network access
this session did not have — and is not invented. What **is** measurable, entirely inside this
repository, is the controller's own protocol margin: how many core clocks pass between the
external device launching a nibble and this controller needing that nibble to have settled.
That number is what changed.

## What was actually broken, established by direct measurement

`nano/tb/nano_qspi_resume_tb.v`'s own `QSPI_RESUME_TB_DELAY_CYCLES` macro delays the
device-to-controller half of `sio` by a parameterizable number of clk
cycles (a registered pass-through standing in for a real pad's and Tiny Tapeout's mux's
round-trip latency; the outgoing sck/cs_n/sio half is left ideal, since the residual this
ticket closes is specifically about the capture side). Measured against the pin-level flash
and PSRAM models:

| Design | 0 added cycles | 1 added cycle | 2 added cycles |
|---|---|---|---|
| Shipped before this ticket (capture on SCK's rising edge) | PASS | FAIL (4 mismatches, `z` nibble) | FAIL |
| This ticket (capture on SCK's falling edge) | PASS | PASS | FAIL (4 mismatches, `z` nibble) |

The controller this repository shipped had **zero** round-trip margin: a single extra clk
cycle between a device driving a nibble and the controller consuming it is already enough to
misread a tri-stated bus. That is the finding "not from a guess" produces — the honest answer
here is not "no compensation is needed", because the measurement says otherwise.

## The fix, and the one that was tried and measured not to work

The read-side capture in `ST_FLASH_STREAM` and `ST_PSRAM_READ` moved from SCK's rising edge
(`if (!sio_phase)`, half an SCK period — one clk cycle — after the device's own falling-edge
launch) to SCK's falling edge (`if (sio_phase)`, a full SCK period — two clk cycles — after
launch), merged with the "does this nibble complete a parcel" decision that already ran on
that edge. The last nibble of a parcel reads `sio_in` directly rather than the not-yet-updated
`rx_shift[15:0]`, combined with the already-shifted `rx_shift[11:0]`, because `rx_shift`'s own
update at this edge is a non-blocking assignment that has not committed yet at the point the
completion decision needs its value. This is a pure re-timing of which SCK half-period the
capture belongs to: nibble rate, SCK toggling, dummy-cycle counts and total transaction length
are all unchanged (confirmed by `nano_qspi_resume_tb.v` finishing at the identical simulated
timestamp before and after), so the extra round-trip cycle costs nothing.

**A version closer to the literal shape ADR-0202 sketched — a genuine `sio_captured` register,
clocked every cycle, feeding `{rx_shift[11:0], sio_captured}` at the decision point — was built
and measured first, and it buys no margin at all**: run through the same latency harness, it
fails identically to the unfixed design at 1 added cycle. The reason, worked out by tracing
non-blocking-assignment semantics rather than assumed: a register clocked every cycle and read
one cycle later at the *decision* edge samples `sio_in` at exactly the same instant the old
rising-edge capture did, because the register's own capture edge is exactly as exposed to the
round trip as `rx_shift`'s direct capture was — relocating *which* register holds a value
changes nothing about *when* the value must have arrived. This is worth recording plainly
because it is the shape both ADR-0202's own text and this ticket's brief pointed at first, and
the reverted fix attempt ADR-0202 references touched this same neighbourhood; the difference
that actually buys margin is which **edge** decides, not an extra register in the data path.

## Why this margin, and not more

Doubling the budget (one core clock to two, i.e. one SCK half-period to a full SCK period) is
the largest re-timing available without slowing the protocol itself — SCK's own two phases are
the only two edges this controller's state machine has to choose between per nibble, and
picking the later one is free. Going further (e.g. capturing on a *following* SCK period)
would mean holding SCK for an extra half-period per nibble, which is a real cycle cost this
ADR does not spend, because the measured requirement (below) does not call for it.

**The real requirement is bounded by a fact this tree does measure**: nano's own slow-corner
logic timing already misses the 64 MHz target before any QSPI signal enters the picture —
ADR-0197 recorded the Tiny Tapeout flow's `max_ss_100C_1v60` corner at WNS −7.966 ns against
the `CLOCK_PERIOD` of 15.625 ns `src/config.json` states for 64 MHz (`nano-tt-area-selfhosted`'s
own signoff run, LibreLane 3.0.14, the SoC's real internal logic path, no QSPI controller wired
into that top yet). A design whose own internal logic needs roughly 23.6 ns per cycle at that
corner is not going to be clocked at 64 MHz on real silicon regardless of what this ticket
does; whatever clock nano actually ships at, each core-clock period is *longer* than the
15.625 ns this ADR's own margin was measured against, so the doubled pad-mux budget this
change buys is doubled again in absolute nanoseconds relative to a hypothetical clean 64 MHz
part. **A real board measurement of the Tiny Tapeout pad-plus-mux round trip, once a part
exists, is still owed** — this ADR closes the "no compensation, and nothing grades it" gap
ADR-0202 left open, not the separate, harder question of exactly how many nanoseconds a real
UPduino-class Pmod round trip costs on this part.

## Grading

`make nano-qspi-latency-test` (on `make test`'s path) builds `nano_qspi_resume_tb.v` at
`QSPI_RESUME_TB_DELAY_CYCLES=1` and is the grader: PASS at zero and one added round-trip
cycle. Its forced-red prerequisite, `nano/tb/nano_qspi_latency_probe.sh`, reverts the capture
to SCK's rising edge in a scratch copy and requires that mutant to fail the same case the
shipping controller passes.
`nano/formal/qspi.sby`'s three k-induction proofs (`make -C nano/formal components_qspi`)
re-prove clean, unaffected — none of the three invariants (CS mutual exclusion, the PSRAM
CS-low bound, prefetch-buffer contiguity) is stated in terms of which phase captures a nibble.
`nano-qspi-resume-test` and `nano-qspi-pins-test` (both simulator legs, identical retires)
stay green; `nano-qspi-pins-test`'s own suite retires are unchanged from ADR-0202 (`alu.S` 66,
`branch.S` 33, `compressed.S` 75, `loadstore.S` 44, `divide.S`/`mul.S` TRAP at 5, matching
`nano/asm/EXPECTED_FAIL`), confirming the zero-cost claim end to end rather than only in the
minimal reproduction. `make nano-area` reads 60,832.1 of 63,000.0 µm², down from ADR-0202's
61,157.4 (the merged branch has one fewer `begin`/`end` arm; not re-tuned against the ratchet,
since the change was not made to save area).

## Scope

Out of scope, unchanged from ADR-0202: FPGA bring-up with a real Pmod, and the real
nanoseconds a Tiny Tapeout round trip costs on silicon. The write-side (`sio_out`/`tx_shift`)
timing is untouched — this ticket is specifically about the residual ADR-0202 named, the
read-side capture, and the write side was never flagged as compensation-free.

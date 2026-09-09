# Pin constraints, and why each file says what it says

The constraint files themselves carry no comments -- they are five to seventeen lines
of `set_io` or `LOCATE`, and a header explaining a measurement is prose, which belongs
here. Each section below is the header that file used to carry, moved verbatim. When a
pin number or a placement changes, change it here too: nothing grades this file, and a
stale pin table is exactly the failure these headers were written to prevent.

## `soc/upduino.pcf` -- The UPduino v3.x board (ice40 up5k, sg48)

UPduino v3.x, iCE40UP5K in SG48. Read off the vendor's own RTL/common/upduino.pcf rather
than off a pin table: two community tables give the clock as 41 and 44 and both are wrong.
The vendor's file states pin 20, and says why -- it is IOB_25B_G3, a global clock input.

THE CLOCK IS NOT CONNECTED UNTIL R16 IS SHORTED. R16 is silkscreened OSC. With it open this
pin floats and the design will not run, however green the build is.

V3.0 SILKSCREEN ERRATUM: the GND and 12M labels on the header are swapped. The documentation
drawing is right and the board is wrong. This file constrains the FPGA pin, so it is
unaffected -- but anything probed by the labels is not.

## `soc/littlesoc.pcf` -- The SoC on the iCEBreaker pinout (ice40 up5k, sg48)

Pin constraints for rtl/littlesoc.v on an ice40 up5k in an sg48 package.

NINE PINS, and that is the honest answer rather than a shortcut. Both memories are internal
-- ROM in block RAM, RAM in SPRAM -- so the design has no external bus at all. That is
precisely why it places when `make fit`'s top does not: `littlecpu` with its memories
external presents 231 `SB_IO` against sg48's 39 and always fails on a pad.

The pin numbers are the iCEBreaker 1.0e assignments for its 12 MHz oscillator, its user
button, its two on-board LEDs and the FPGA-to-host side of its FTDI serial bridge, which is
the up5k/sg48 board this project has been written against from the first area measurement
on. Nothing here is load-bearing for the timing measurement -- `icetime` reports the
critical path through the fabric, and five IO pads cannot move it -- but a made-up pinout
would place differently from a real one, and a number taken against a board that does not
exist is the kind of thing this repo has been burned by twice.

Pin 9 is the board's `TX`: the direction is named from the FPGA, so it is an OUTPUT here and
the host's receiver at the other end. Its partner `RX` on pin 6 is deliberately
unconstrained -- rtl/uart.v has no receiver, and a pin assigned to nothing is a pin
icetime's parser would be asked about for no reason.

No `-nowarn` flags, and that is not a style choice: icetime's own .pcf parser accepts
exactly `set_io <name> <pin>` and asserts out on anything longer. Every port here is
constrained, so there is nothing for `-nowarn` to suppress.

## `soc/littlesoc.lpf` -- The SoC on ECP5 (LFE5U-25F, caBGA381)

Pin constraints for rtl/littlesoc.v on an ECP5 LFE5U-25F in a caBGA381 package -- the die on
a Colorlight i5, which is the board `make ecp5-timing` names its corner from.

ONE PIN, and the other three are deliberately unconstrained. The i5 is a module, not a
board: its 25 MHz oscillator is a fact of the module and lands on P3, but the button and the
LEDs belong to whatever baseboard it is plugged into and vary between them.
soc/littlesoc.pcf records why that matters -- "a number taken against a board that does not
exist is the kind of thing this repo has been burned by twice" -- so the pin that is a real
fact is pinned and the three that are not are left to nextpnr with `--lpf-allow-
unconstrained`.

It is not free and it is not cosmetic: letting nextpnr choose the clock pad too read 34.88
MHz against 33.65 MHz here, 3.5% apart, because the pad it picks decides where the global
network is entered from. The pessimistic number is the one taken against the real
oscillator, which is the same direction as taking speed grade 6.

NO `FREQUENCY` LINE, and that is a tripwire rather than an omission. A clock constraint here
would override the pinned `--freq` the Makefile hands the placer, and a 25 MHz target met at
25 MHz would report the constraint instead of the design -- which is the whole reason that
constant exists.

## `soc/compare/bench_up5k.pcf` -- The cross-core bench on up5k

The comparison bench on up5k/sg48 -- the UPduino's own part and package, and three pads: a
clock and two LEDs. The pin numbers are soc/upduino.pcf's, so a bitstream built from this
runs on the board `make prog` already flashes.

up5k is one of the two parts this project ships to, and the harness dropped the ice40 hx8k
it used to place on as well. That part was chosen because every VexRiscv iCE40 figure their
project publishes is hx8k, on the belief that it was the only ice40 with enough logic to
hold this core at all. That belief expired: all three cores fit up5k, this one the largest
of them at 84% of ICESTORM_LC, and hx8k cannot hold `littlesoc` itself (no SPRAM, so 64 KB
of data RAM is 128 block RAMs against the part's 32). A cross-core number taken on a part
nothing here ships to is a measurement of a machine nobody can build.

THE CLOCK HERE IS NOT A FREQUENCY. up5k's clock is a step function -- the board's 12 MHz
crystal, or SB_HFOSC's 48/24/12/6 -- so what a placement says about a core on this part is
which STEP it reaches, not how many MHz it made. `make compare-timing` grades that pass/fail
through soc/compare/step_gate.py and the comparison is then cycles alone.

THREE PADS, and all three tops present exactly these three. rtl/littlesoc.v also takes a
reset button; there is none here because VexRiscv has no second input to give one to, and an
asymmetric pinout is a variable in a comparison whose whole point is that nothing else
varies. Three pads cannot move a fabric critical path either way.

No `-nowarn` flags: icetime's .pcf parser accepts exactly `set_io <name> <pin>` and asserts
out on anything longer. Every port here is constrained.

## `soc/compare/bench_ecp5.lpf` -- The cross-core bench on ECP5

The comparison bench on ECP5. Only `clk` is located, and to soc/littlesoc.lpf's own SITE
"P3" -- the pad that file's header records as the one that mattered, because the pad decides
where the global network is entered. The two LEDs are left to `--lpf-allow-unconstrained`
the way the SoC's own flow leaves its unconstrained pins: a placed LED pad is not part of
what this measures, and inventing sites for them would put two made-up numbers in a timing
report.

ECP5's clock is NOT continuous either -- `EHXPLLL` synthesises ref x M / N / D with integer
dividers, from whatever reference the board carries -- but its grid is fine where up5k's is
four frequencies wide. Rounding down to a reachable output costs a fraction of a percent
here and half the machine there. That difference in DEGREE is why the comparison runs on two
parts: up5k answers "what is fastest on the board this ships to", and this answers "what is
architecturally faster when the clock is close to yours to choose".

## `.gitattributes` -- Why docs/adr/README.md merges as a union

docs/adr/README.md is a numbered index, and every PR that adds an ADR appends a row to it at
the same anchor -- the end of the table. Two such appends are not a real conflict (neither
PR touched a line the other wrote), but git's default line-based merge sees two edits
converging on one location and refuses to guess an order, so every second PR to land
conflicts on this file regardless of what it actually changed. A union merge keeps distinct
appended lines from both sides with no conflict; it does not stop two branches from
independently choosing the same ADR number for two different files, which
test/adr_numbering_test.sh still catches as a collision either way -- that failure needs a
person to rename one of the two, not a merge strategy.

# 0135 — The flash is reachable, and fetch cannot move to the SPRAM

Status: accepted

## Context

Programs live in 8 KB of block RAM and that is now the binding constraint. Two things measured on
the board say so. `test/asm/rvc.S` is 12 256 bytes and does not fit the instruction memory at all —
it had only ever run under `test/asm/sections.lds`, which grants a simulator 16 KB, and nothing said
so until something tried to put it on hardware. And `soc/run_suite_board.sh` packs the 71 runnable
programs into seven batches and reflashes between them, because they total 58 752 bytes.

Meanwhile the part carries a 4 MB SPI flash it configures itself from, and the design could not read
one byte of it.

Two shapes were on the table. **(a)** read the flash into the block RAM the design already fetches
from, which leaves the fetch loop alone. **(b)** fetch out of the two unused `SB_SPRAM256KA`, which
is eight times the ROM and would retire batching outright, at the risk of the fetch loop —
[ADR-0087](0087-the-instruction-memory-comes-out-of-the-fetch-loop-and-depth-does-not-pay.md)
measured that taking the instruction memory out of that loop costs 3–4 logic levels at the tail and
*adds* levels at the head, and the board clock is met on tenths.

## Decision

**Shape (a). Shape (b) is refused by the part's SPRAM count before its timing question is ever
reached, and shape (a)'s own ceiling turns out to be lower than the ticket that proposed it thought.
So what ships is the mechanism both shapes need and neither of the two payoffs.**

The mechanism is `rtl/spiflash.v`: a byte-at-a-time, single-lane, mode-0 read controller on the data
bus, eight bytes above the UART's two. A shift register, a bit counter and a chip select — it knows
no commands, so a JEDEC id read and an `0x03` sequential read are the same eight clocks a byte, and
the fortieth command nobody has needed yet is already implemented.

## Shape (b) is arithmetic, not a sweep

Fetch asks for **two neighbouring words every cycle**. `rtl/imemory.v` serves that from two banks
split by word parity, so each bank answers one of the pair and each needs its own address in the
same cycle. That pair is what makes a 32-bit instruction straddling a word boundary free.

`SB_SPRAM256KA` is **16 bits wide with one address port**. So:

| | SPRAM |
|---|---|
| a 32-bit bank | 2 |
| two banks, which is what a fetch window is | **4** |
| the data RAM, 64 KB | 2 |
| **the part has** | **4** |

Text in SPRAM takes all four and leaves the 64 KB data RAM none, and there is nowhere else to put
it: the whole block RAM on this part is 30 × 512 B = 15 KB and `rtl/regfile.v` already holds four of
those. The three ways around it all fail on their own terms:

- **One bank, one word a cycle.** Half the SPRAM, and it gives up the pair — a 32-bit instruction
  across a word boundary then needs two reads, which is a stall reason and a cycle cost on top of
  a design ADR-0087 already priced at −29% to −49% of the clock-times-cycles product on this board.
- **Halve the data RAM to 32 KB.** 4 + 1 = 5, and the part has 4.
- **A second read port.** `SB_SPRAM256KA` has one address. There is no dual-port mode to ask for.

So the ticket's table is right that 64 KB of SPRAM is unused and wrong that it is available: it is
available to anything that needs **one** port on it, and a fetch window needs two. **The sixteen-seed
sweep ADR-0087's warning called for was not spent, because the design it would have measured cannot
be built on this part.**

## Shape (a)'s ceiling is 8 KB, and that is also arithmetic

The proposal was that spending the ten spare block RAMs raises the ROM to about 13 KB. It does not.
`rtl/imemory.v` refuses a `ROM_WORDS` that is not a power of two, and it refuses it in an
elaboration `$fatal` rather than a comment, because both of its range tests are reductions on the
address bits above the ROM and a reduction is `< ROM_WORDS` only at a power of two. The next power
of two above 8 KB is 16 KB, which is 32 block RAMs against the 26 that are free. **There is no
legal ROM size between the one that ships and one that does not fit**, so shape (a) does not make
`rvc.S` runnable and does not retire batching either.

That is worth stating rather than leaving implied: the flash is now reachable, and the two things
the ticket wanted from reaching it are both still open.

## What the controller is, and what a program can do with it

Two registers in eight bytes at `0x0002_0028`:

| | |
|---|---|
| `BASE+0` | write byte lane 0: shift it out, shift eight bits in. Read: `busy` in bit 8, the byte the last exchange shifted in below it |
| `BASE+4` | write bit 0: the chip select. Reads zero — write-only, the way the UART's data register is |

`sck` is the bus clock halved, so a byte is sixteen cycles: 1.33 µs at 12 MHz, and 8 KB is about
11 ms.

**`busy` sits beside the byte rather than in the register software writes, and the co-simulation is
why.** The reference model has plain memory at this address, so a wait that polled back the chip
select it had just stored spun on Sail forever and `test/COSIM_EXPECTED_FAIL` read `INCONCLUSIVE
SAIL-LIMIT`, which compares nothing. With the poll on a register nothing stores a set bit to, the
model runs off the end of the program and the entry is `DISAGREE AT 9` — eight architectural
changes compared and agreeing, and the divergence pinned at the first read of `busy` after a write.
The same file records the other half of that: the program loads over `a0 = -1` at the divergence so
that the load is a *change* on both sides, because reading over zero left the model recording
nothing and reporting a length difference as a budget it later ran out of.

A device layout is allowed to take an observation like that into account. A program written to agree
would not be.

## The pin, which had to be decided first and was the live defect

`soc/upduino.pcf` put `uart_tx` on pin 14, and the vendor's own constraint file gives that pin two
names: `serial_txd` and `spi_miso`. So the FPGA's only route to the USB serial port is the flash's
data line. That is not a layout mistake to route around — there is no other pin on this board that
reaches a host, and no way to read the flash without pin 14 either.

It was also already broken. During a suite run the board replays its report continuously, so the
FPGA drove pin 14 while `iceprog` read the flash through it, and the symptom was exact and
reproducible: `cdone: high` after a reset that should have lowered it, a JEDEC id arriving a byte
late as `FF EF 70 16`, and then `Write error (single byte, rc=-1)`. `soc/run_suite_board.sh` retried
four times.

**The flash's own chip select is the arbiter, and it is the one signal both owners can see.** This is
the design that was first wired up, all four pins at once; read on before taking it as what ships —
the section after the next one narrows it to two of the four, and says why. `soc/board_upduino.v` put
all four dedicated pins behind `SB_IO` output enables:

| pin | driven by | when |
|---|---|---|
| 14 `spi_miso` / `serial_txd` | `uart_tx` | neither the on-chip controller nor the host has the flash selected |
| 15 `spi_sck` | the controller | while it holds the select |
| 16 `spi_ssn` | the controller, low | while it holds the select — and read back the rest of the time |
| 17 `spi_mosi` | the controller | while it holds the select |

Pin 16 being an input whenever the controller is idle is what lets the design see the host asserting
the select — `iceprog` drives it from the FTDI, and this board leaves the FPGA running throughout,
because its CRESET is not wired to the programmer. The `SB_IO` pull-ups are load-bearing rather than
tidy: a released pin 14 has to idle high because that is the UART's idle level, and a released pin
16 has to read high because a chip select read low by accident selects the flash.

**The USB serial path is kept, and no pin moved.** The retry loop stays in
`soc/run_suite_board.sh` until a board has actually run the new bitstream — it costs nothing when
the first attempt wins, and removing it on an argument rather than a run is the mistake this project
has a rule about.

## The arbitration was one-directional, and a second pass closed it

The table above states what the pins do once the on-chip controller holds the select; what it elides
is that nothing stopped the controller asserting its own select while the host held pin 16 low. The
pin's read-back gated the *UART's* enable — `!own_flash && ssn_in` — but `own_flash` itself was
`!soc_cs_n` alone, so a program that reached for the flash mid-`iceprog` put two drivers on pins
15/16/17: the same contention class already measured on this board, one entry up in this same
document.

The obvious fix — require `ssn_in` read high before `own_flash` may go high — is wrong, and wrong in
a way that only shows up on the part: `own_flash` enables `io_ssn`'s output, which drives pin 16 low,
which is what `ssn_in` then reads, which would drop `own_flash`, which releases the pin, which the
pull-up takes back high, which regrants — a combinational loop closed through the pad rather than
through any net yosys can see, so it synthesises clean and oscillates at whatever speed the fabric
settles at. `soc/pin_lockout.v` is the sequential answer: a two-flop synchroniser on `ssn_in` that
free-runs only while it is not granting, and freezes the instant it grants, so pin 16 is read *before*
this design drives it and never re-read *through* its own drive. `own_flash` is `want && released`
where `released` is that frozen sample — combinational in the request/grant sense, but built from a
register the request path cannot feed back into within a cycle, so `check -assert` has nothing to
flag and there is nothing to oscillate.

The FSM adds no cycles in the case that matters: `released` is already valid, sampled across however
many idle cycles preceded the request, at the instant `soc_cs_n` first goes low, so a program that
selects the flash and immediately shifts a byte behaves exactly as before whenever the host was not
there. When the host *is* there — actually driving pin 16 low, mid-transaction — the request is
silently withheld: the controller's internal shift register still runs to completion off
`mem_wstrb`, unaware anything is wrong, and `shift_in` fills with whatever the released, pulled-up
pins read, because nothing in `rtl/spiflash.v` needs to know about a board-level lock. `test/pin_lockout_tb.v`
grades the module standalone against a testbench-driven `release_in`, which is what a second chip on
the far side of the pin looks like from here, and demonstrates its own red direction: `grant = want &&
release_in`, the version this ADR just argued is wrong, fails the same bench by granting the instant
a glitch on `release_in` — which is what this module's own drive would read back as — coincides with
`want`.

**Ratified.** The obvious fix closing a loop through the pad, and the FSM instead, is the architect's
call and is recorded here rather than only in the pull request that made it, per the pattern the rest
of this file follows.

## The predicate cannot deliver the property, and the sharing does not ship

`released` answers one question: does pin 16 read high right now. That is electrically identical for
two different situations — a host that is genuinely not there, and a host that is idle with its own
chip select parked high — and `iceprog` demonstrates the second for most of a programming session,
not a corner of one: it drives pin 16 as a push-pull MPSSE output between its own transactions, not
only while it is actively clocking the flash. The fix above closes the case `release_in` reads LOW —
a host actively driving the pin down while it talks to the flash. It does nothing for the case that
dominates a session: a grant can still land in the middle of `iceprog`'s idle stretch, because
`released` reads exactly the way it would if nobody were there at all, and `soc/board_upduino.v`
would then drive pins 15/16/17 into the FTDI's own high driver — two push-pull outputs on three
pins, in the case that matters and not a corner of it.

An SCK-edge detector — grant only while the host is not currently *clocking* — was considered and
declined. It answers a different question: a host mid-transaction is not what this section describes,
a host idle with chip select parked high is, and that idle stretch is where most of a session's time
goes. Nothing tried here distinguishes "driven high" from "pulled high" by reading the pin, and there
may be no way to on these four pins — the two states are the same voltage by construction, and
telling them apart needs information the pin itself does not carry: a bus-hold input, a current
sense, or a physically separate pin, none of which this board has spare.

**So the claim this ADR shipped with an earlier draft of — that the two controllers simply never
share a wire — does not hold, and the arbitration built above is best-effort, not proven sound.**

Given that, **`rtl/spiflash.v` — the on-chip controller — does not ship wired to this board at all.**
`soc/board_upduino.v` instantiates `littlesoc` with `spi_miso` tied high and `spi_sck`/`spi_mosi`/
`spi_cs_n` left unconnected, so none of the controller's own pins can ever be driven from the shipping
design and the finding above cannot be reached from it. `rtl/spiflash.v` and `soc/pin_lockout.v` both
still ship — the controller is on the bus and reachable in simulation, `test/asm/spiflash.S` and
`test/spiflash_tb.v` grade it there, and `pin_lockout` gains a bounded grant of its own (below) so a
hung on-chip request cannot hold the pins forever once a future change wires it to one — but
connecting it to real pins is that future ticket's, and it owes a real-board measurement of the
contention window this section describes, not another RTL argument.

**Pin 14's own conflict is a separate, older question, and it is answered here rather than deferred.**
That conflict was never about the on-chip controller: `soc/board_upduino.v` drove pin 14 with the
UART unconditionally from the day this board wrapper was written, long before this ticket, and that
already fought the same flash chip for the wire whenever `iceprog` read it — the `cdone`/JEDEC-id/
write-error symptom this document opened with. Pin 16 answers a *different* question than `released`
does, and one this board can answer soundly at DC: not "is a host there", but "is the flash chip's own
output driver enabled" — a deterministic function of the flash's chip select alone, independent of
who or what is holding it at a given level. That soundness is about the LEVEL, and it does not by
itself make a synchronised read of that level a sound gate for pin 14's enable.

**The level is sound; a single synchronised copy of it is not, on the turn-off edge.** The first cut
of this fix read pin 16 through one free-running two-flop synchroniser and gated pin 14's enable on
the synchronised copy alone. Turn-on needing two cycles to settle is correct and cheap — a beat of
extra caution before this design starts driving. Turn-off needing the same two cycles is neither:
the flash's own output driver goes live the instant its chip select reads low, not two cycles later,
so a synchronised-only enable kept the UART driving pin 14 for up to two clock periods — about
167 ns at 12 MHz — after the flash's driver was already live on it. That is two push-pull drivers on
one pin, on every chip-select assertion `iceprog` makes over a session, which is the exact hazard
this fix exists to remove, reintroduced one clock domain crossing later. `soc/miso_share_enable.v`
answers the two directions differently instead: `enable = released && now`, where `released` is the
two-flop synchronised copy and `now` is the raw, unsynchronised read of pin 16, used only to turn the
enable off faster and never to turn it on. Turn-off is combinational on `now`, landing the same edge
the flash sees; turn-on still needs both signals, so it is exactly as cautious as before. Reading
`now` combinationally here creates no loop through the pad, the way gating a *grant* on it would:
nothing downstream of this module ever writes pin 16, so there is nothing for `now`'s glitches to
feed back into — the module's own header states that reasoning rather than assuming it. Pins 15 and
17 are not wired to this board at all; only pin 14 (shared) and pin 16 (read only) are ports of
`upduino_top`.

## The grant is bounded now, independent of whether it is ever wired to a pin

`want` was a level with nothing to lower it but firmware, and firmware that traps, hangs, or busy-waits
after asserting the chip select held `grant` — and so the three physical pins it will eventually gate
— for the life of the chip. That defect is real regardless of the predicate question above, and
`soc/pin_lockout.v` fixes it: a new `busy` input, meant to be `rtl/spiflash.v`'s own "a byte is
actually moving" signal, and a grant held for `IDLE_LIMIT` (sixteen cycles, one of the controller's own
byte times) with `busy` never once true is presumed idle rather than mid-transfer. Both synchroniser
flops are forced back to their cold-start value, which drops the grant for at least two cycles — long
enough for a genuine resample of `release_in` rather than a reuse of whatever it read before. If
nothing else wants the pins the resample finds them released and the grant returns on its own; if
something does, it does not, the same contract `release_in` reading low has always had.

This bounds a different thing than the predicate question above answers: it stops the on-chip request
from outliving its own need for the pins, which is a property of the requester and not of what
`released` can see. `test/pin_lockout_tb.v` grades it directly — a request with `busy` pulsing between
bytes never sees a forced drop no matter how long it runs, a request that goes idle with `busy` never
true loses its grant within one byte time and gets it back unmolested if nothing else wants the pins,
and a host that takes the pin during the window a forced drop opens keeps the grant withheld exactly
as if it had been the first request. **What is not measured**: sixteen cycles is `rtl/spiflash.v`'s
own per-byte shift time, so a caller pacing exactly one byte after another never sees the bound fire.
A caller whose own overhead between bytes exceeds that would see the grant blink mid-transfer, which
this module cannot tell apart from a hang — that is unmeasured against real compiled firmware timing
and is owed to whichever change first wires a requester to this input.

## Read-only is firmware convention, and staying that way is ratified too

`rtl/spiflash.v`'s own header already says why: the module is a shift register with no notion of
commands, so `0x06` then `0x20` — write-enable and sector-erase, and the sector at the bottom of this
part's flash is the bitstream that configured it — cost the same eight clocks a byte as a `0x03`
read. Making that a hardware guarantee needs the shifter to recognise the first byte of a transaction
against a table of what a command means, which is a policy engine — the exact thing this module's
whole design avoids needing, since it is what lets it read the JEDEC id and the status register and
the data array alike without knowing what any of them are. The flash's own write-enable latch is what
actually stands between a stray store and a damaged board, and the software on this chip already has
full bus access, so gating one more device buys no new trust boundary. **Ratified**: the module stays
read-only by firmware convention and by nothing else.

## The board's own grader was blind, and the diagnosis was an environment gap, not a toolchain one

`make board-elaborate` failed on CI's yosys and passed on two others the same day it was reviewed, and
`soc/board_elaborate.sh` printed nothing under the failure either time it was tried. Both looked like
one story — the toolchain that moves under CI, which this file already warns about elsewhere — and
were two different, smaller ones.

**The grader's silence was its own defect, unrelated to which yosys ran it.** Yosys attaches a
`file:line: ` prefix ahead of `ERROR: `/`Warning: ` for any diagnostic tied to a source line —
`$readmemh`'s own "Can not open file" among them — so `grep -E '^(ERROR|Warning)'`, anchored at the
start of the line, matches a bare diagnostic and silently drops every file-scoped one. That has been
true of every yosys version tried here; it had simply never been exercised, because elaboration had
never failed with a file-scoped diagnostic before. `soc/board_elaborate.sh` now matches the prefix
wherever it starts, and falls back to the log's own tail when nothing matches either shape, so a
diagnostic in some third form still shows something rather than nothing.

**The failure itself was `rtl/imemory.v`'s `$readmemh` on `soc/rom_even.hex`, which does not exist on
a fresh checkout.** `make board-elaborate: $(BOARD_SRCS)` never depended on the ROM the RTL it
elaborates reads unconditionally, and nothing else in `make test`'s chain writes those files first —
so the target only ever worked on a working tree carrying leftovers from an earlier `make bitstream`
or `make fit`, which is exactly what two different local runs had and CI's checkout does not.
Reproduced directly under CI's own yosys build (a linux/amd64 container running the pinned binary):
the elaboration fails identically missing the ROM and passes identically once
`board-elaborate: $(BOARD_SRCS) $(BOARD_ROM)` builds it first — on the same yosys, the same sources,
nothing else different. The toolchain was never the variable; the checkout state was.

## The measurements

Nothing moved that a memory census can see. `SOC_EXPECT_EBR` is 20 and `SOC_EXPECT_SPRAM` is 2,
both unchanged, because the controller has no memory in it.

`make fit`: 4109 cells against the 4097 the Makefile records, inside the churn band, against a
budget of 4219.

The mapped netlist moved, so the paired sixteen-seed sweep is owed and was taken —
`soc/baseline_sweep.sh` on both trees, paired by seed, one toolchain:

| | worst | median | best | spread | under 12.00 MHz |
|---|---|---|---|---|---|
| control, `9944aad` | 12.45 | 12.77 | 13.14 | 5.5% | 0 of 16 |
| with the controller, `03b70c5` | **12.06** | **12.99** | 13.37 | 10.8% | **0 of 16** |

**Median of the per-seed deltas: −1.60%, with 3 of 16 seeds slower.** The middle of the distribution
moved the right way by less than this part's churn band, which is a null read generously.

**The tail is the number to argue about, and it is 12.06 MHz.** One seed of sixteen accounts for the
whole of it: seed 2 is +5.00% where the other fifteen span −4.69% to +1.85%. The requirement holds
at all sixteen and the margin over it is half a percent, which is thinner than the control's 3.6%
and is not outside what this design has read before — `CLAUDE.md` records the shipping tree's own
worst-of-sixteen at 12.39, 12.21 and **12.03** MHz on three earlier trees, "three samples and not a
trend". Read this the same way: a sample of a wide tail, not a regression with a mechanism. The
spread went 5.5% to 10.8% of the best placement, inside the range `soc/bands.py` records for a
netlist that did not change at all. **Ratified**: 12.06 MHz is not a regression, on this evidence,
and this sweep is not owed a re-take on that account.

Packed cells go 4875 to 4943, and `make fit` reads 4109 against the 4097 the Makefile records —
both inside the churn band, both from the region test's new window term rather than from the device,
which is outside `fit`'s top entirely.

An earlier sweep of this change read 12.26 MHz worst and −0.25% median, and it is not quoted above:
it was taken before the simplification pass took 29 `SB_LUT4` back out, `make netlist-diff` reported
the digest had moved, and a sweep whose netlist no longer exists describes a design nobody ships.

**`soc/pin_lockout.v` is outside every one of those figures, and outside `upduino_top` too, now.**
The sixteen-seed sweep and the `fit` numbers above were both taken on `littlecpu`/`littlesoc`, which
never synthesise the board wrapper, so neither moved with anything described in this section — the
module was never in that census to begin with. It was, for a few commits, inside `make bitstream`'s
placement of `upduino_top` — single-seed numbers with no ratchet, 12.82 MHz after the FSM landed
against 12.93 MHz for the tree above it — and those numbers are retired the way an earlier sweep is
retired above and for the same reason: the shared pins are not wired into `upduino_top` any more, so
the netlist those two numbers describe is not the one that ships.

## Consequences

- **The flash is reachable from software and nothing else about the machine changed.** The
  controller answers the data bus like the timer and the UART, adds no stall reason, and does not
  touch the fetch loop. F and G are unmoved and no BMC depth is re-derived.
- **A fifth window on the shared bus**, and every file that restates the map grew with it: the
  decoder's region test, the core's copy for the locality counters, `formal/traps.sv`'s copy for
  the access-fault model, `test/asm/riscv_test.h`, and `test/memmap_test.sh`, which compares them
  and gained six probes in `test/probe_gates.sh` for the new copies.
- **The word above the UART is a device now, so two tests moved up.** `test/asm/uart.S` and
  `test/decoder_tb.v` each probed the region decode with the first address no memory answered, and
  that address is the controller's. Both now store four words up instead of two, and both gained the
  positive half — the controller's own window is answered.
- **The board wrapper has a grader now, and it did not before.** `soc/board_upduino.v` is read by
  `make bitstream` and `make prog` and by nothing else, and both need a board — so it was the one
  file CI could not fail while it carried real logic. `make board-elaborate` reads it on `make test`
  and forces two ways of breaking it red, the way `make window-test` does, and depends on the ROM its
  own elaboration reads rather than on a working tree happening to have one already. The second
  mutation case moved with the wiring it tests: it was once the four-pin arbitration's own `SB_IO`
  enable; it is now pin 14's, reading pin 16 through the synchroniser described above, so the same
  hazard class — an enable wired to a synchroniser input nothing drives — is still caught, on a
  smaller design. It does not read `soc/upduino.pcf`: nothing in this repo parses one, so a pin
  assignment naming a port that no longer exists is still nextpnr's to catch and nobody else's.
- **`soc/pin_lockout.v` has a grader that does not need a board, and now grades a bounded grant
  too.** `test/pin_lockout_tb.v` drives the synchroniser directly and demonstrates the red direction
  the naive combinational gate takes, and the red direction a request that never lowers `want`
  takes. What it cannot grade is the pad it will sit behind — no simulated `SB_IO` model here
  reflects a design's own drive back onto its own input the way silicon does, which is the exact
  behaviour the freeze exists for.
- **What is NOT claimed: that `released` — the predicate `soc/pin_lockout.v` was built around — is
  sound, or that granting the on-chip controller its own pins on it would be safe.** `released`
  cannot distinguish a host that is idle with its chip select parked high from one that was never
  there, and `iceprog` does the former for most of a session. That is why `rtl/spiflash.v`'s own
  pins are not wired into `soc/board_upduino.v` here — pin 14's UART conflict is answered by a
  different, sound predicate (above), and pin 16 is read only, never granted anything. Wiring the
  controller itself to real pins is a future ticket's, and it owes a real-board measurement before
  it does.
- **What is NOT claimed: that a board has run any of this.** No board was attached while this was
  written. The controller, the model, the program and the lockout are graded in simulation; the map
  is graded by `test/memmap_test.sh` and the generated riscv-formal checks. There is nothing here to
  flash yet that reaches the flash, because nothing wires it to a pin.
- **What is NOT claimed: that either payoff arrived.** `rvc.S` is still too large for the part,
  `soc/run_suite_board.sh` still batches, and neither is fixed by anything here. What a flash boot
  still needs is a loader — software, not RTL, since text is writable and `test/asm/selfmod.S`
  already exercises writing it and `fence.i`-ing it — and an image flashed past the bitstream, which
  `iceprog -o` writes. Both halves are proven separately here and the two together have not been run.
- **The ROM cannot grow, and that is a fact about the range test rather than about the part.** If a
  bigger text window is ever worth it, the thing to price is a `ROM_WORDS` that is not a power of
  two, which means a magnitude comparison where two reductions are today — and
  `rtl/imemory.v` already records that the comparison it replaced measured a quarter of the whole
  period.

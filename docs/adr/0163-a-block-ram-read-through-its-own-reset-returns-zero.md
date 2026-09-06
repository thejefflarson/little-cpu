# ADR-0163: A block RAM read through its own reset returns zero on the part

**Status:** Accepted · 2026-09-05

## Context

ADR-0161 put `rtl/littlesoc.v` on a MuseLab iCESugar-Pro and watched
`soc/blink.S` cycle its LEDs, which says the core executes. It left the UART
unrouted, so the board could not yet report a number. Routing it (pin B9, and a
`CLOCK_HZ` parameter on `littlesoc` so `rtl/uart.v` derives a divisor from the
board's 25 MHz rather than the up5k's 12) made the board print, and printing is
what found this.

**Dhrystone was silent.** The image built, placed at 33.28 MHz against a 25 MHz
constraint, configured, and said nothing for 90 seconds.

## What was wrong

**The data RAM returned zero from every address.** A probe down the same
`test/crt0.S` and `test/bench/bench.lds` path the benchmark uses printed

```
w0=00000000 w1=00000000 w2=00000000 w0again=00000000 bytes=00000000
```

for word writes, a second address, and four byte-strobed writes into one word
alike. `.bss` "passed" its zero test only because an all-zero read cannot fail
one. Dhrystone hangs because it is stack-heavy and every load reads nothing;
`blink.S` and `hello.S` run because neither uses a stack or touches RAM, which
is why first light looked clean.

Everything else on the part was correct. The multiplier and divider returned
`123456 * 789 = 0x05ce4f40` and `1000000 / 7 = 0x00022e09` remainder 1. String
literals printed, and those are `lbu` reads of `.rodata` in ROM through the data
side. The UART's status register read correctly, since polling `busy` between
bytes is what keeps the output clean.

**Yosys maps `rtl/memory.v`'s out-of-range arm onto the block RAM's own reset.**
Written

```verilog
mem_rdata <= in_range ? ram[index] : 32'b0;
```

the zero arm becomes `DP16KD`'s output reset: the mapped netlist drives `RSTA`
from logic on all 32 of the data RAM's blocks, with `REGMODE_A` at `NOREG`. On
silicon that read returns zero whatever the array holds. The ROM never trips it,
because a read-only memory needs no reset — which is exactly why the ROM worked
while the RAM did not.

## What it was not

Recorded because each cost a measurement, and because the first diagnosis
offered was the wrong one.

- **Not timing.** Identical at 12.5 MHz through a divider, where the design has
  2.7× margin (`$glbnet$div` maxes at 33.91 MHz).
- **Not the memory spelling.** The read-first spelling `rtl/memory.v`'s header
  says ice40 maps to 128 `SB_RAM40_4K` produces a different ECP5 mapping and
  fails identically.
- **Not the deep ×1 configuration.** An 8 KB RAM — 4 blocks in a different
  shape rather than 32 — fails identically.
- **Not memory inference.** Three different mappings failing the same way is not
  what an inference bug looks like. The defect is the reset path, and it is the
  one thing all three share.

## Why nothing here could have caught it

- **RTL simulation passes.** `rtl/littlesoc.v` running the same ROM under
  iverilog returns `w0=deadbeef w1=a5a5a5a5 w2=0f0f0f0f bytes=44332211`. The
  design was never wrong.
- **The cell censuses pass.** 36 `DP16KD` either way. A census says the memory
  did not fall back to soft logic, never that it works.
- **nextpnr places and times it** and reports a frequency with margin.
- **The mapped netlist cannot be simulated.** Yosys ships no behavioural model
  for `DP16KD` — the module in its `ecp5/cells_sim.v` is a port list with no
  body, and its outputs read `z`. So no simulation of the ECP5 netlist can catch
  this on any machine, with or without a board.
- **`make ecp5-timing` never runs a program.** It places one. That is the gap.

## Decision

**The out-of-range zero is a mux on the block's output, never a synchronous
constant.** `rtl/memory.v` registers the array read and `in_range` separately
and selects combinationally, which ties `RSTA` low and puts the zero arm in
LUTs that behave the way the source says.

**`soc/bram_reset_check.py` refuses any ECP5 netlist in which a block RAM's
reset is driven by logic**, reading the mapped JSON rather than a log because
this is a connectivity question and the censuses are counting questions. It
gates all three ECP5 flows — `make ecp5-timing`, `make icesugar-bitstream` and
`make dual-ecp5-timing` — and is refused, not passed, on a netlist with no block
RAM in it or one it cannot parse. Seven `test/probe_gates.sh` probes force it
red, including both of those ways of grading nothing.

**`make icesugar-dhrystone` runs a program on the part**, end to end: build the
ROM, gate the netlist, load it, read the report. It is off `make test` and off
CI because it needs the board, the same standing as `make suite-board`.

**Programming is SRAM over JTAG, not the flash.** Copying the `.bit` onto the
iCELink volume and `icesprog -w` both write the flash correctly — a readback
compares byte-identical — and both leave the ECP5 at `@cdone:0`, unconfigured
until the board is physically power-cycled. `openFPGALoader -c cmsisdap` with
the iCELink's own vid/pid loads SRAM and the design starts immediately. Without
the explicit vid/pid openFPGALoader finds no probe at all: it looks for a
CMSIS-DAP v2 device, fails, and never reaches the HID path the debugger speaks.

## What it costs

The mux moves onto the RAM's read output, which is inside the second of the two
loops around the fetch address. Paired at one seed on the ice40 SoC:

| | `make soc-timing` |
|---|---|
| shipping spelling | 12.29 MHz |
| this spelling | **12.66 MHz** |

Faster, by 3.0% — inside the ~3.6% edit-churn band, so a null rather than a win,
and not a regression. **A sixteen-seed paired sweep is owed before this is
quoted as anything but "it did not cost the requirement."**

## What it bought

Dhrystone runs on the iCESugar-Pro at **0.775 DMIPS/MHz, 19.4 DMIPS at 25 MHz**,
`-O2`, self-check PASS, 734 cycles per Dhrystone at CPI 1.58.

| | Cycles | Cycles/Dhrystone | DMIPS/MHz |
|---|---|---|---|
| iCESugar-Pro, 25 MHz | 14,680,022 | 734 | 0.775 |
| cxxrtl, same ROM | 14,660,022 | 733 | 0.776 |

Retired instructions are identical at 9,240,026. **The board is exactly 20,000
cycles slower, which is one cycle per run, and that is not explained.** It is a
difference between the SoC and `test/bench`'s harness, not a difference in what
was computed; it is recorded here rather than rounded away.

## What this does not settle

The one-cycle-per-run gap above. Whether the fault is yosys emitting an output
reset the part cannot honour in this configuration, or nextpnr and Trellis
encoding it wrongly, is not established — the RTL workaround does not need the
answer, and no simulation available here can supply it. The edit ties `RSTA` low
*and* moves the mux after the register; those two were not isolated. And the
gate is structural: it refuses the shape that failed here, which is not the same
as proving the part honours every other shape.

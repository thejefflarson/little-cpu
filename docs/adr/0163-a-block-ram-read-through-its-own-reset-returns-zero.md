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

### Amendment 2026-09-08: the sixteen-seed sweep

**Method.** A comment-only edit has been measured to move the mapped netlist by
tens of cells with no semantic change, so pairing this commit against its
parent conflates the mux with whatever else rode along. Both arms are instead
two spellings of `rtl/memory.v` on ONE tree: the shipping mux spelling, and the
pre-fix synchronous-constant spelling reconstructed against this tree's current
`memory.v` (the `NHARTS`/`atomic_supported` shape ADR-0163's one-seed number
predates). The edit between the two files is the minimal one — the
`always_ff` body and the two lines it declares — and both files hold at 56
lines, so nothing about line count or unrelated text moved. `make
netlist-digest` on each arm, same toolchain (`oss-cad-suite`, yosys
0.68+48/ff5817c34), reads:

| | cells | `SB_LUT4` | `SB_DFFESR` | digest |
|---|---|---|---|---|
| sync (pre-fix) | 6326 | 4465 | 712 | `sha256:eee1a824…` |
| mux (shipping) | 6289 | 4428 | 711 | `sha256:9e6f7aa8…` |

37 cells apart, all attributable to the register the mux spelling splits into
two flops (`ram_q`, `in_range_q`) where the sync spelling gates one
(`mem_rdata`) directly — the shape the edit predicts, not a re-roll's
unexplained churn. Re-taken independently at the start of this amendment: the
digest, cell and type counts above reproduce to the digit against the draft
that first recorded them, so the control stands.

Sixteen seeds (1–16), paired by seed, `make soc-timing SOC_SEED=<n>` each side,
swapping `rtl/memory.v` between runs and restoring the shipping mux spelling
before any other gate ran.

Sixteen seeds on unrelated `main` (932d021) were swept the same session and put
its own worst seed (9) at 11.99 MHz, under the 12.0 requirement — a property of
that tree independent of this spelling, and filed separately (JEF-993). It is
reported here only so seed 9's reading in the mux column below is not mistaken
for a defect this amendment introduces: `main` already ships the mux spelling,
so the mux arm below and `main`'s own sweep are one and the same tree, and the
numbers agree to the seed — worst 11.99 MHz, median 12.445 MHz (12.45 rounded),
best 12.80 MHz, all at seed 9, 9, 11 respectively in both sweeps.

Per-seed critical path, both arms, `oss-cad-suite` yosys 0.68+48/ff5817c34
throughout:

| seed | sync ns | sync MHz | mux ns | mux MHz |
|---|---|---|---|---|
| 1 | 75.91 | 13.17 | 82.75 | 12.08 |
| 2 | 76.40 | 13.09 | 79.69 | 12.55 |
| 3 | 78.20 | 12.79 | 82.34 | 12.14 |
| 4 | 75.91 | 13.17 | 80.06 | 12.49 |
| 5 | 77.09 | 12.97 | 78.18 | 12.79 |
| 6 | 74.64 | 13.40 | 81.08 | 12.33 |
| 7 | 77.38 | 12.92 | 80.04 | 12.49 |
| 8 | 77.17 | 12.96 | 80.30 | 12.45 |
| 9 | 76.43 | 13.08 | 83.38 | 11.99 |
| 10 | 78.49 | 12.74 | 80.88 | 12.36 |
| 11 | 80.02 | 12.50 | 78.10 | 12.80 |
| 12 | 79.71 | 12.55 | 79.90 | 12.52 |
| 13 | 81.45 | 12.28 | 80.41 | 12.44 |
| 14 | 77.09 | 12.97 | 80.73 | 12.39 |
| 15 | 76.27 | 13.11 | 80.48 | 12.43 |
| 16 | 77.14 | 12.96 | 80.07 | 12.49 |

**Worst, median, spread**, `spread = (worst_ns − best_ns) / best_ns`, the same
formula ADR-0121 derives:

| arm | best | median | worst | spread | under 12.00 MHz |
|---|---|---|---|---|---|
| sync (pre-fix, never ships) | 13.40 MHz | 12.968 MHz | **12.28 MHz** | 9.12% | 0 of 16 |
| mux (shipping) | 12.80 MHz | 12.445 MHz | **11.99 MHz** | 6.76% | 1 of 16 (seed 9) |

**The single-seed number this section opened with does not survive the sweep,
and the direction reverses.** One seed read the mux spelling 3.0% faster; the
median of sixteen reads it (12.965 − 12.445) / 12.965 = **4.0% slower** than
the pre-fix spelling, and the mux arm's only sub-12.0 seed is exactly the seed
`main`'s own sweep already reports missing for reasons this amendment did not
introduce (JEF-993). Read together: the fix has a real median cost on the
up5k SoC, not the null the one-seed figure suggested, and it clears the
requirement at 15 of its 16 seeds with the sixteenth attributable to a
tracked, separately-owned tree property rather than to the mux itself —
`SOC_MIN_MHZ` does not move on that basis (CLAUDE.md: 12.0 is a requirement,
not a regression floor), and JEF-993 is where that miss is decided.

**Decision: the mux spelling still ships**, because correctness is not up for
trade against period. `rtl/memory.v`'s sync arm is the shape
`soc/bram_reset_check.py` exists to refuse — it returns zero from every
address on the ECP5 part (this ADR's whole subject) — so it is not a candidate
regardless of what the sweep reads; the sweep's job was only to price the fix
honestly, and it now has. No alternative spelling (masking at the SoC's
wired-OR read bus, or a differently-shaped output mux) was tried: the measured
cost is a median shift with margin at 15 of 16 seeds, not a tail that misses
the board clock for a reason this edit owns, so ADR-0163's "Alternative
spellings if the tail is bad" branch is not taken here. A later change that
wants that margin back owns its own sweep.

## What it bought

Dhrystone runs on the iCESugar-Pro at **0.775 DMIPS/MHz, 19.4 DMIPS at 25 MHz**,
`-O2`, self-check PASS, 734 cycles per Dhrystone at CPI 1.58.

| | Cycles | Cycles/Dhrystone | DMIPS/MHz |
|---|---|---|---|
| iCESugar-Pro, 25 MHz | 14,680,022 | 734 | 0.775 |
| cxxrtl, same binary | 14,680,022 | 734 | 0.775 |

**Cycle-identical**, the same standing ADR-0130 records for the UPduino.

Getting there corrected a comparison rather than the core. `make dhrystone`
first read 14,660,022, exactly 20,000 fewer — one per run — against identical
retired instructions. That is a DIFFERENT BINARY, not a different machine:
`dhrystone-rom` defines `DHRY_UART` and `make dhrystone` does not, so the board
build compiles in `uart_putc`'s busy-wait and the repeat loop, `.text` moves, and
the measured loop pays one more cycle each run. Built with the same define the
simulator returns the board's number to the cycle. `DHRY_BOARD_CFLAGS`'s comment
claimed sharing `DHRY_CFLAGS` made the two comparable; it is necessary and not
sufficient, and the Makefile now says so beside the define that breaks it.

## What this does not settle

Whether the fault is yosys emitting an output
reset the part cannot honour in this configuration, or nextpnr and Trellis
encoding it wrongly, is not established — the RTL workaround does not need the
answer, and no simulation available here can supply it. The edit ties `RSTA` low
*and* moves the mux after the register; those two were not isolated. And the
gate is structural: it refuses the shape that failed here, which is not the same
as proving the part honours every other shape.

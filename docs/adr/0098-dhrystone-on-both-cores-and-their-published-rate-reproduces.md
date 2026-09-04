# 0098 — Dhrystone runs on both cores in one harness, and their published rate reproduces

Status: Accepted · *Amended 2026-08-12: both factors re-measured on one tree. The cycle factor below
is superseded and the 1.21× product with it; the clock factor is re-measured and holds.* · *Amended
2026-08-30: both factors re-measured again on `main`, twelve seeds a side instead of five. The
2026-08-12 amendment's cycle factor is superseded — ADR-0129 and sixteen other merges moved `rtl/`
in between — and its clock factor is superseded too, not because it moved but because five seeds
is a look and twelve is what this file's own convention for a verdict costs. The 1.05× product is
superseded by 1.16×.
2026-08-30: a third core, Hazard3's iCE40 build, joins the harness for Dhrystone (its clock alone
was ADR-0139's), all three cores measured at their one shared ISA, RV32I.
2026-09-04: both factors re-taken on `e332776`; the halves moved in opposite directions and the
1.16× product did not move.*

## Context

[ADR-0086](0086-both-cores-in-one-harness-and-the-gap-is-the-fetch-loop.md) put this core and the
VexRiscv Verilog in the pinned riscv-formal clone into one harness and measured the **clock** on
both: five placements a side, read on the worst of each. That closed half of a comparison. The other
half is cycles, and only one side of it was measured:

| | clock | × DMIPS/MHz | DMIPS | where the second factor came from |
|---|---|---|---|---|
| this core | 32.54 MHz | 0.640 | 20.8 | measured, `make dhrystone` |
| VexRiscv | 48.19 MHz | 0.52 | 25.1 | **that project's published figure** |

So "we are about 1.2× behind on throughput" rested on a number nobody here had run. That deserved
checking twice over. ADR-0086 had already caught the other half of the same pair — VexRiscv-small's
published **92 MHz did not reproduce**, coming out at 50.23 worst-of-five and 48.19 on a later run,
a 1.9× overstatement in the figure that motivated the whole comparison. And Dhrystone is the most
gameable benchmark in common use: this repository requires its own figure to carry its flags, its
compiler and its string library, and `make dhrystone` prints all three and will not compile without
them ([ADR-0084](0084-dhrystone-is-the-comparable-number-and-the-raw-share-does-not-transfer.md)).
A published 0.52 with none of that attached is not comparable to a 0.640 that has all of it.

## Decision

**`make compare-dhrystone` runs Dhrystone 2.1 on both cores, in one simulation, off one image, with
this repository's compiler, flags and string routines — and the published 0.52 reproduces.**

400 runs, `riscv64-elf-gcc 16.2.0`, both cores' self-checks passing and both data RAMs identical
afterwards:

| | cycles | cycles/Dhrystone | DMIPS/MHz | worst-of-five clock | DMIPS |
|---|---|---|---|---|---|
| **this core** | 335,229 | 838.1 | **0.679** | 32.54 MHz | **22.10** |
| **VexRiscv** | 408,758 | 1021.9 | **0.557** | 48.19 MHz | **26.84** |

*This core's row is superseded; VexRiscv's reproduces exactly. See the amendment.*

```
-march=rv32ic -mabi=ilp32 -O2 -std=c11 -ffreestanding
-fno-tree-loop-distribute-patterns -Wall -Wextra -Werror
```

String routines are this port's own byte loops, not a libc's; multiply and divide are libgcc's,
because the image has no M extension. The flags are compiled into the image and read back out of it
by the runner, so what is printed beside the number is what the machine ran.

**Their published 0.52 reproduces, and is if anything conservative.** Measured here, in this
harness, on this toolchain, that artefact is **0.557 — 7.1% above the published figure**, not the
1.9× miss the clock claim turned out to be. Two figures from the same project therefore land very
differently against this harness, and the difference is what each one depends on: a clock is a
property of a flow, a floorplan and a part, and DMIPS/MHz is mostly a property of the RTL.

**On throughput the gap is 1.21× and every factor of it is now measured here.** 26.84 against 22.10,
from 1.48× the clock and 0.82× the cycles. This core needs **18.0% fewer cycles** for the same work,
which is most of what its slower clock costs it back. *This paragraph is superseded — the cycle
factor moved when the bus transaction moved into the execute slot. The amendment at the end carries
the current pair.*

## The blocker was real, and it is the harness's memory

**Dhrystone does not fit the geometry this harness places, and no ice40 in this flow can hold it.**
The benchmark's `Arr_2_Glob` is 10 KB on its own; the linked image needs 1528 bytes of ROM and
10,572 bytes of RAM, which is 26 `SB_RAM40_4K`. An hx8k has 32 in total, and each core wants some
before either memory: 4 for this core's register file, and **18 for VexRiscv — 4 for a register file
the same size and 14 for a 1024-entry branch predictor**. So the arithmetic is 4 + 26 = 30 blocks on
one side and 18 + 26 = **44 blocks of a 32-block part** on the other. `soc/compare/dhry_fit.py`
prints that every run, reading each core's own count out of the same standalone yosys census
`soc/compare/placed_vs_synth.py` grades against rather than carrying a copy.

Trimming the benchmark to fit is not an option, for ADR-0084's reason: a trimmed Dhrystone is not
Dhrystone and its number could not be compared with anything, which is the whole point of running
one.

**So the cycle counts are simulated at 8 KB of ROM and 16 KB of RAM, and the clock they are
multiplied by was measured at 4 KB and 2 KB.** Both cores run that same enlarged map, so the two
cycle counts are comparable *with each other*, which is the property this directory exists for. The
absolute DMIPS column is a projection and is labelled as one wherever it is printed.

## Every distortion, stated

Nine, and the first five apply identically to both cores by construction:

- **The image is RV32IC, the ISA the two cores share.** No M extension, so every multiply and divide
  the benchmark does is a libgcc call rather than an instruction. This core's multiplier and divider
  are in the netlist and never issued.
- **The memories are 8 KB / 16 KB, not the placed 4 KB / 2 KB**, per the arithmetic above.
- **The run is timed on the bus, not by the program.** VexRiscv's configuration here has no CSR
  file, so `csrr mcycle` — `test/bench/dhry_port.c`'s timer — is not an instruction it can execute.
  `soc/compare/dhry_port.c` stores a marker to the word at the base of RAM immediately before and
  immediately after the measured loop, and `soc/compare/dhry_tb.v` counts the cycles between the two
  markers on each core's own write bus.
- **`.rodata` is in RAM and the harness pokes it there.** This is the gap the benchmark found:
  `soc/compare/bench_vexriscv.v` gives its core **no data path to the ROM at all** — the instruction
  memory there is a read port for fetch and nothing else, and a load from a ROM address reads back
  zero. `soc/compare/bench.S` never noticed, because it has no constants and reads only what it
  stored itself. Dhrystone is string literals, so left in ROM they would be real bytes on one core
  and zeros on the other. Measured: with the strings in ROM, VexRiscv spins forever at `Func_2`'s
  `while` loop, which terminates only when two string bytes differ and reads both as zero. So
  `soc/compare/dhry.lds` puts `.rodata` in RAM with no load copy, `soc/compare/dhry_ram.py` builds
  the RAM image, and `soc/compare/dhry_start.S` is `test/crt0.S` with the copy loop removed. The
  alternative was a bus mux in that harness top, i.e. editing one of the two designs under
  comparison. All of it happens before the first marker.
- **Both RAMs are loaded and both register files zeroed** before the run, for the reason
  `test/testbench.v` zeroes its ROM banks: block RAM comes up holding what the bitstream put there.
  Left X, the two cores' prologues store different undefined registers. That is not cosmetic —
  with X in RAM this measurement reported a VexRiscv run of 418 cycles per Dhrystone that had
  branched on undefined data.
- **`dhry_1.c` and `dhry_2.c` are `make dhrystone`'s files, unedited**, and are compiled as three
  separate translation units with no LTO, which is how every published number is built. Only the
  port differs.
- **This core's figure here is not `make dhrystone`'s**, and must not be quoted as it: 0.679 against
  0.640, 6.1% apart, because the distortions above are real work removed. Strings in RAM cost no
  fetch window here, where in the shipping map a `.rodata` read is a text-range load that steals
  one.
- **The run count is not a factor.** The measured window is the benchmark's loop and nothing else:
  100 runs and 400 differ by 0.02% on this core and 0.26% on VexRiscv.
- **Neither side is a shipped design.** This core is RV32IMC + Zicsr with five traps and a machine
  timer; that one is RV32IC with a 1024-entry branch predictor, no CSR file, no traps and no
  interrupt. **This is not a like-for-like comparison and must not be quoted as one.**

## What makes the result able to fail

A benchmark harness is exactly where a comparison that cannot fail hides: a core faulting on its
first instruction reports a cycle count too.

- **The two data RAMs are compared word for word at the end of the run**, all 4096 of them, and must
  be identical. Same image, same memories, no interrupt on either side. This is what says the two
  cores did the same work — a self-check agreeing on both sides agrees on one bit, and the ROM gap
  above is exactly the kind of defect that leaves that bit intact for a while. Bus traffic is
  deliberately *not* what is compared: this core re-presents a store while it is stalled and
  VexRiscv replicates store data across all four lanes, so the two write streams differ in shape
  while committing the same bytes.
- **The benchmark's own self-check must pass on both cores**, reported through the control window as
  the riscv-tests verdict encoding.
- **Both cores must publish two markers.** One is a run that reached the start of the measured loop
  and never the end of it.
- **The simulated geometry is graded against itself**: `soc/compare/dhry.lds`'s two regions against
  `soc/compare/dhry_tb.v`'s two parameters, because a testbench simulating memories the image was
  not linked for is two machines reported as one. `ld` refuses a `.text` overflow; nothing but this
  refuses a `.bss` past the end of RAM.

All of it is graded in `soc/compare/dhry_fit.py` and `soc/compare/dhry_dmips.py` rather than in the
simulation, so `test/probe_gates.sh` can force every failure path with fixture text and no
simulator. Nineteen probes, including the RAM comparison going red on the 111 differing words the
ROM gap actually produced.

**The placed-versus-synthesised check applies to both cores and both pass**: 6751 placed
`ICESTORM_LC` against 6052 `SB_LUT4` synthesised alone (1.12×) here, 2356 against 1711 (1.38×)
there. ADR-0086's all-NOP case was 0.26×.

## Consequences

- **The cross-core claim no longer has a quoted number in it.** Both factors of both cores'
  throughput are measured in one place, on one image, with one toolchain, and the conclusion is
  unchanged in size: 1.21× on DMIPS where the quoted figures said 1.20×.
- **Their published DMIPS/MHz reproduces where their published clock did not.** Worth knowing which
  kind of number travels: this harness bounds our side of a clock comparison and not theirs
  (ADR-0086), but a cycles-per-work figure is mostly the RTL's and came out 7.1% *better* than
  published rather than worse.
- **`make compare-dhrystone` is a measurement, not a gate.** Nothing in it grades the shipping
  design, nothing in `rtl/` changed, and it is not on CI — the same standing as `make dhrystone` and
  `make compare-timing`. The graded things inside it are the harness's self-consistency and the
  cross-core RAM comparison.
- **The clock is the operator's to pass in.** `COMPARE_DHRY_MHZ` takes each core's worst-of-five from
  `soc/compare/sweep.sh`; with no clock the absolute column stays empty rather than being invented,
  and no placement figure is stored in the repository to go stale.
- **`soc/compare/bench_vexriscv.v` has no data path to its ROM, and now that is written down.** It
  did not matter for `soc/compare/bench.S` and it is not fixed here, because fixing it means editing
  one of the two designs being compared and re-taking every placement in ADR-0086. Anything else run
  in that harness has to keep its read-only data out of ROM until it is.
- **ADR-0086's table is amended** with the second factor and the absolute figures; its clock
  measurements are unchanged and were reproduced by this work at 30.73 ns and 20.75 ns worst of
  five.

## Amendment, 2026-08-12 — both factors re-taken on one tree, and the product is 1.05×

**Which factor went stale, and how.** The **cycle** factor above is superseded; the **clock** factor
is re-measured here and holds. Both were recorded on 2026-08-11, and the cycle one moved the same
day: launching the bus transaction from the execute slot took this core from 0.679 to
**0.784 DMIPS/MHz** in this harness (ADR-0099), which is 1.41× on cycles alone where the whole
product above was 1.21×. That left the headline as a current numerator over a stale denominator — a
cycle ratio from one tree beside a clock ratio from another, with a different accessor, one fewer
stall reason and several merges of decode logic in between. **A product whose two factors were not
measured on the same tree is not a measurement**, so both are re-taken here.

Measured on `be293ff`, with the toolchain ADR-0086 recorded: Yosys 0.68+post (`c12172fb`),
nextpnr-0.10-108-g68c1acd8, `riscv64-elf-gcc 16.2.0`, Icarus Verilog 13.0. VexRiscv's Verilog is
byte-identical to that run's — it comes out of the SHA-pinned clone and this repository never edits
it — so its whole column is a reproduction check on the flow rather than a new number.

### The clock, five placements a side

`COMPARE_SEEDS='default 1 2 3 4' soc/compare/sweep.sh`, the same seed set ADR-0086 used:

| | ns, sorted | worst | median | spread |
|---|---|---|---|---|
| **this core** | 29.00 · 29.68 · 29.79 · 30.07 · **30.67** | **30.67 ns — 32.61 MHz** | 29.79 ns | 5.8% |
| **VexRiscv** | 19.19 · 19.87 · 20.28 · 20.66 · **20.75** | **20.75 ns — 48.19 MHz** | 20.28 ns | 8.1% |

**The clock factor is unmoved: 1.48× worst on worst, 1.47× median on median.** Reading it off the
distributions rather than off one placement is what says so — the two ways of reading it differ by
less than a percent, which is inside either side's own spread. This core reproduces the 30.73 ns
recorded above at 30.67, and VexRiscv reproduces 20.75 exactly.

### The cycles, one image and one simulation

`make compare-dhrystone`, 400 runs, both self-checks passing and both data RAMs identical in all
4096 words:

| | cycles | cycles/Dhrystone | DMIPS/MHz |
|---|---|---|---|
| **this core** | 290,427 | 726.1 | **0.784** |
| **VexRiscv** | 408,758 | 1021.9 | **0.557** |

**1.41× on cycles**, this core needing **29.0% fewer** for the same work. Both figures reproduce
ADR-0099's to the cycle, so nothing merged since has moved this half either.

### The product

| | DMIPS/MHz | worst-of-five MHz | DMIPS |
|---|---|---|---|
| **this core** | 0.784 | 32.61 | **25.56** |
| **VexRiscv** | 0.557 | 48.19 | **26.84** |

**The throughput gap is 1.05×, from 1.48× on clock against 0.71× on cycles** — down from the 1.21×
recorded above, and the whole move is in the cycle factor. On each side's *median* placement instead
of its worst it is 1.04× (26.32 against 27.47), so the conclusion does not depend on which placement
is read.

**Quote it with what it is.** Neither side is a shipped design: this core is RV32IMC + Zicsr with
five traps and a machine timer, and has no timer in this harness and 4 KB of ROM; that one is RV32IC
with a 1024-entry branch predictor, no CSR file, no traps and no interrupt. The DMIPS column
multiplies a clock placed at 4 KB / 2 KB by cycles simulated at 8 KB / 16 KB, because no ice40 in
this flow has the block RAM to hold Dhrystone. All nine distortions are listed above and none of
them changed.

**Their published 92 MHz still does not reproduce.** Best of five here is **52.11 MHz** and worst is
**48.19**, against a published 92 — 1.77× to 1.91× short, the same picture ADR-0086 measured at 54.00
and 50.23. Their published 0.52 DMIPS/MHz still does, at 0.557. Two numbers from one project, one of
which travels to another flow and one of which does not.

### The graded harness check was exercised, not assumed

`soc/compare/placed_vs_synth.py` passes on both sides — 6644 placed `ICESTORM_LC` against 6003
`SB_LUT4` synthesised alone here (1.11×), 2356 against 1711 there (1.38×), against a 0.80× floor —
and `make compare-smoke` is green on six published values that both cores agree on and that are not
all the same value.

Passing says nothing on its own, so the founding defect was re-run live rather than left to the
fixture probes. **An all-NOP ROM still folds VexRiscv away and is still caught**: 1101 placed cells
against 1711, **0.64× and red** — less of a collapse than the 0.26× that founded the gate, but well
under the floor. **It no longer folds this core**: 6626 placed cells against the
real program's 6644, 1.10×, green, and 6345 `SB_LUT4` synthesised against 6364. The block-RAM census
says why — the NOP image makes every ROM word identical, and the VexRiscv harness's read-only array
collapses to a constant that takes 9 of its 30 `SB_RAM40_4K` and most of the core with it, while
this harness's instruction memory is written by the design and survives at 16 blocks whatever it
holds. So on this core's side the demonstrated red direction is `test/probe_gates.sh`'s fixture and
not a placement, and a future stimulus meant to test this gate has to fold something a write port
cannot protect.

## Amendment, 2026-08-30 — both factors on `main`, twelve seeds a side, and the product is 1.16×

**Why this is owed again.** The 2026-08-12 amendment's cycle factor named itself stale the day it
landed — a launch-from-execute-slot change moved it once already — and `CLAUDE.md` has carried a
standing note ever since that the region-answer work (ADR-0129, 13.79% of `make dhrystone`'s own
cycles) left the quoted 0.784 describing a tree that no longer exists. Between `be293ff` and this
amendment's tree, 61 commits landed and 17 of them touched `rtl/`: ADR-0129's region wait, the
eleven A instructions' decode and execute (ADR-0106, ADR-0108), atomic region faults (ADR-0109), the
Zkt claim and its proofs (ADR-0134, ADR-0137), the transmit-only UART, and more. **No single one of
those is credited for what moved below** — that would need the same tree rebuilt with each held out,
which nobody has done — the two numbers below are simply what one tree reads today, the way this
file's own rule requires.

Separately, this amendment also answers two questions nobody had asked of this specific pair before:
whether the two slaves in `soc/compare/dhry_tb.v`'s harness hold the same wait-state contract, and
whether that harness's write counters have the same asymmetry a CoreMark harness under review
elsewhere was found to have. Both come back negative, below.

Measured on `main` at `ab5af011b6a1d8a6b7e6e1027317c30121d2b6b6`. Toolchain: Yosys 0.68+post
(`c12172fb`), nextpnr 0.11.1, `riscv64-elf-gcc 16.2.0`, Icarus Verilog 13.0.

### The clock, twelve seeds a side

`COMPARE_SEEDS='default 1 2 3 4 5 6 7 8 9 10 11' soc/compare/sweep.sh`, run in six batches of two
seeds each rather than one call (each placement takes roughly 70 seconds and there are 24 of them):

| | ns, sorted | worst | median | best | spread |
|---|---|---|---|---|---|
| **littlecpu** | 30.25 · 30.59 · 30.59 · 30.62 · 30.89 · 31.24 · 31.26 · 31.58 · 31.95 · 32.08 · 32.17 · **32.36** | **32.36 ns — 30.90 MHz** | 31.25 ns — 32.00 MHz | 30.25 ns — 33.06 MHz | 7.0% |
| **VexRiscv** | 19.67 · 19.78 · 19.88 · 20.00 · 20.07 · 20.09 · 20.27 · 20.29 · 20.63 · 20.66 · 20.66 · **20.73** | **20.73 ns — 48.24 MHz** | 20.18 ns — 49.55 MHz | 19.67 ns — 50.84 MHz | 5.4% |

**The gap on clock is 1.56× worst on worst, 1.55× median on median** — up from the five-seed
amendment's 1.48×/1.47×, and it is the wider sample rather than the design that moved: littlecpu's
best-of-twelve (33.06 MHz) is *faster* than its five-seed worst-of-five (32.61 MHz) was, and its
worst-of-twelve is the same 30.90 MHz ADR-0139 already recorded independently for this design on
this harness. VexRiscv's whole distribution sits inside the five-seed one's span (19.67–20.73 against
19.19–20.75). Twelve seeds found a wider tail on this side, not a slower design; `soc/bands.py` still
declares no churn band or placement spread has been derived for hx8k, so neither distribution can yet
be called wide or narrow against a standard — only against each other and against the five-seed look
that preceded it.

### The cycles, one image and one simulation

`make compare-dhrystone`, 400 runs, both self-checks passing and both data RAMs identical in all
4096 words:

| | cycles | cycles/Dhrystone | DMIPS/MHz |
|---|---|---|---|
| **littlecpu** | 304,428 | 761.1 | **0.748** |
| **VexRiscv** | 408,758 | 1021.9 | **0.557** |

**VexRiscv reproduces exactly**, to the cycle, the same figure every prior amendment recorded — its
Verilog is byte-identical, read straight out of the pinned clone, so this is a reproduction check on
the flow and not a new measurement. **littlecpu's own figure moved, 0.784 → 0.748, −4.6%**, and per
the note above that is 61 commits' worth of `rtl/` change read as one number, not a single line
item's price.

### The product

| | DMIPS/MHz | worst-of-12 MHz | DMIPS | median-of-12 MHz | DMIPS |
|---|---|---|---|---|---|
| **littlecpu** | 0.748 | 30.90 | **23.11** | 32.00 | 23.94 |
| **VexRiscv** | 0.557 | 48.24 | **26.87** | 49.55 | 27.60 |

**The throughput gap is 1.16× on worst-of-twelve, 1.15× on median-of-twelve** — up from the
five-seed amendment's 1.05×/1.04×. Reading the move apart: the clock ratio widened from 1.48× to
1.56× (a wider sample on littlecpu's side, above) and the cycle ratio narrowed from 1.41× to 1.34×
(littlecpu's own cycle count grew 4.6% while VexRiscv's held). Both factors moved against littlecpu
this time, where the 2026-08-12 amendment's whole move was in one factor.

**Quote it with what it is**, the same caveat every prior amendment has carried: this is RV32IMC +
Zicsr with five traps and a machine timer (untimed in this harness, 4 KB of ROM) against RV32IC with
a 1024-entry branch predictor, no CSR file, no traps and no interrupt; the DMIPS column multiplies a
clock placed at 4 KB / 2 KB by cycles simulated at 8 KB / 16 KB, for the reason given above the first
table in this ADR. All nine distortions listed above still apply and none of them changed.

### The wait-state question, asked of this pair and answered negative

A security review of the CoreMark harness (not yet merged to `main`) found `soc/compare/bench_hazard3.v`
holding `hready` low for one cycle after every write's address phase — an artefact of AHB5 splitting a
write's address and data phases, which `rtl/memory.v`'s single-ported synchronous RAM cannot service
in the same cycle it services a new address. **That artefact does not exist in this pair.** Neither
`bench_littlecpu.v` (`.bus_wait(1'b0)`, tied off) nor `bench_vexriscv.v`
(`.dBus_cmd_ready(1'b1)`, tied off) ever holds the other side's command off the bus: both cores'
command interfaces carry address, write data and the byte strobe together on one cycle, matching
`rtl/memory.v`'s and `memory`'s expectation that a write's address and data arrive together, so
neither adapter needs — or has — an extra cycle to buffer one phase against the other.

Checked in the generated Verilog rather than assumed from the comment: VexRiscv's own
`when_DBusSimplePlugin_l482` (the memory stage's only stall on `dBus_rsp_ready`) is gated
`&& (!memory_MEMORY_STORE)` — it stalls waiting for a response on a **load**, never on a store, so a
store completes the cycle its command is accepted (`dBus_cmd_ready`, tied high here) and never waits
on `dbus_rsp_valid` at all. `bench_vexriscv.v`'s own `dbus_rsp_valid <= dbus_cmd_valid &&
!dbus_cmd_wr` — registered false the cycle after every write — is therefore not a withheld
acknowledgement; VexRiscv never asks for one. littlecpu's own store path is the same shape: `mem_wstrb`
is asserted and the write commits in the cycle `rtl/memory.v` sees it, with `bus_wait` tied to zero.
**A checked negative, not an assumed one**: this pair's cycle counts are not biased against either
side by an artificial wait state the harness invents, which is a different conclusion than the
CoreMark harness reached for its own Hazard3 pairing and is recorded here rather than left implicit.

### The write-counter question, asked of `dhry_tb.v` and answered negative

The same review found a CoreMark testbench counting littlecpu's writes on `|mem_wstrb` (any strobe
bit set) and its other core's writes on an exact `4'b1111` match — a comparison that would silently
undercount a core whose narrow stores never assert all four bits. **`soc/compare/dhry_tb.v` does not
have this asymmetry.** Both `ours_writes` and `vex_writes` are counted on `|dut_ours.mem_wstrb` and
`|dut_vex.mem_wstrb` respectively — the same predicate, the same width test, on both cores — so
neither side's cycle count nor its write tally is inflated or deflated relative to the other's by how
the counter is written. This counter is not what the DMIPS/MHz figures above are built from in any
case (the marker cycles between `CTL_MARK` writes are), so even an asymmetry here would not have
moved this ADR's numbers; it is checked anyway because a counter that is wrong once and unnoticed is
a counter nobody can trust the next time it is read for something that does matter.

### `make compare-smoke` re-run

All three benches — littlecpu, VexRiscv, Hazard3's iCE40 build — publish the same six values on the
one shared image: `first e784a639, last matched b812e4ba`. `soc/compare/placed_vs_synth.py` passed on
both sides of this sweep's own placements, at the same ratios ADR-0098's earlier amendment recorded
(1.11× littlecpu, 1.38× VexRiscv).

### Consequences

- **The 2026-08-12 amendment's headline (1.05×, 25.56 against 26.84 DMIPS) is superseded by this
  one's (1.16×, 23.11 against 26.87 worst-of-twelve).** Nothing about the direction changed — VexRiscv
  is still ahead on the product, littlecpu is still ahead on cycles by more than VexRiscv is ahead on
  clock is behind — but the size of the gap moved with the wider seed sample and with 61 merges' worth
  of `rtl/`, and both are stated above rather than left for the reader to reconcile against a number
  that was true on a different tree.
- **Five seeds was a look and twelve is the number this file's own convention has asked for since
  before this pair existed** (`SOC_MIN_MHZ`'s go/no-go convention, restated for `soc-timing` above).
  This is the first time that convention was applied to `soc/compare/`'s own hx8k harness rather than
  only argued for it.
- **The wait-state and write-counter findings are checked negatives, not assumed ones**, and are
  recorded here because a comparison this file exists to keep honest does not get to skip a check
  just because the check came back clean. The Hazard3-specific bias the review found is not this
  pair's to fix — Hazard3 is not part of the VexRiscv product — and remains open where the CoreMark
  work that found it will land.
- **A product whose two factors were taken on two trees, or with two different seed counts, is still
  not a measurement.** This amendment moved both factors and the seed count together for exactly the
  reason the 2026-08-12 amendment gives for doing so at all.

## Amendment, 2026-08-30 — a third core, one shared ISA, and the shared subset's own cost

**`make compare-dhrystone` now runs all three cores in one image and one simulation.** Hazard3's
iCE40 build joined `soc/compare/` with its clock factor measured and its Dhrystone cycle factor
explicitly deferred (ADR-0139); this closes that gap the way `soc/compare/dhry_port.c`'s marker
mechanism was always built to close it — `CSR_COUNTER=0` means Hazard3 cannot execute `csrr mcycle`
any more than the pinned VexRiscv can, and the marker protocol already times a core with no CSR file
at all, so nothing about Hazard3 needed a new mechanism, only wiring: a third `bench_hazard3`
instance in `soc/compare/dhry_tb.v`, its regfile zeroed the way the other two already are (Hazard3's
`RESET_REGFILE` defaults to 0, matching `fpga_icebreaker.v`, so nothing else clears it), and its
write bus read off `mem_wstrb_mux`/`mem_addr_mux`/`hwdata` — the same captured signals
`soc/compare/bench_tb.v`'s three-way smoke check already reads for the identical reason.

### The ISA had to be settled before any of this could compile

Each pinned build supports a different subset, verified against the actual sources rather than
trusted from a README or a prior ADR's summary:

- **littlecpu** — RV32IMAC + Zicsr + Zifencei + Zkt, as `CLAUDE.md` states.
- **VexRiscv** (the pinned riscv-formal clone) — RV32IC, no M. Confirmed by absence rather than a
  comment: `formal/riscv-formal/cores/VexRiscv/VexRiscv.v` contains no `mul` or `div` identifier
  anywhere in the generated Verilog, which is what "no M extension" means for a design with no
  plugin architecture left to read.
- **Hazard3's iCE40 build** — RV32IMA, no C. `soc/compare/hazard3/example_soc/fpga/fpga_icebreaker.v`
  states `EXTENSION_C(0)`, `EXTENSION_M(1)`, `EXTENSION_A(1)` verbatim, and
  `soc/compare/bench_hazard3.v` already copied that line before this ticket, for the clock-only
  comparison ADR-0139 built.

**The intersection of all three is plain RV32I** — VexRiscv is the one with no M, Hazard3's iCE40
build is the one with no C, and nothing narrower than their union of gaps is left. `COMPARE_DHRY_CFLAGS`
moves from `rv32ic` (the two-core pair's shared subset) to `rv32i`, and `test/march_test.sh`'s
allow-list entry for it is folded into the one that already named `soc/compare/bench.S`'s reason for
the same string. Multiply and divide are libgcc calls for every core in the three-way row now, even
the two whose netlists carry a hardware multiplier — this core's parallel one and Hazard3's
`MULDIV_UNROLL=1` sequential one are both idle for the whole run.

### The block-RAM arithmetic that could have closed the ticket, and did not

`soc/compare/dhry_fit.py` already refused to guess whether a third core's own census would push the
harness's block-RAM arithmetic past what an hx8k has, so it was checked before writing the rest of
this: Hazard3's iCE40 build synthesised alone is **4 `SB_RAM40_4K`**, not 18 like VexRiscv's
branch-predictor-carrying configuration. Against Dhrystone's own 28 blocks at this geometry (8 KB ROM
/ 16 KB RAM, unchanged from the two-core harness), that is:

```
littlecpu   4 of its own + 28 for the image =  32 blocks: fits
vexriscv   18 of its own + 28 for the image =  46 blocks: DOES NOT FIT
hazard3     4 of its own + 28 for the image =  32 blocks: fits
```

Hazard3 fits at exactly the part's ceiling, the same shape littlecpu already did — `make
compare-timing` never places two of the three cores in this harness at once, so "does this core fit"
was always a per-core question against the same 32-block budget, and a third core answering it the
same way littlecpu does is not a new kind of finding, only a repeated one. VexRiscv is still the one
that does not fit, for the same reason ADR-0086 named: its 1024-entry branch predictor.

### The three-way row: one image, one simulation, all three cores

400 runs, `riscv64-elf-gcc 16.2.0`, all three self-checks passing and all three data RAMs identical
to littlecpu's (the reference `soc/compare/dhry_tb.v`'s ramdiff lines compare against — agreement is
transitive, so this is the same claim a full round-robin would be, for one fewer full-RAM scan):

```
-march=rv32i -mabi=ilp32 -O2 -std=c11 -ffreestanding
-fno-tree-loop-distribute-patterns -Wall -Wextra -Werror
```

| | cycles | cycles/Dhrystone | DMIPS/MHz |
|---|---|---|---|
| **littlecpu** | 300,427 | 751.1 | **0.758** |
| **VexRiscv** | 385,647 | 964.1 | **0.590** |
| **Hazard3 (iCE40)** | 319,632 | 799.1 | **0.712** |

VexRiscv takes 1.284× littlecpu's cycles for the same work; Hazard3 takes 1.064×; Hazard3 takes
0.829× VexRiscv's. Every ratio the ISA permits comes out of this one table, because RV32I is the only
subset all three cores share — there is no fourth pairing left over to ask about.

**Hazard3's row carries a disclosed bias the other two do not pay.** `soc/compare/bench_hazard3.v`'s
AHB5 adapter holds `hready` low for one cycle after every write's address phase, because `hwdata`
does not arrive until the following cycle and a single-ported memory cannot serve that write's data
phase and a new transfer's address phase at once (ADR-0139's own bus section). `soc/compare/dhry_tb.v`
counts those cycles directly — `wr_pending_q` sampled every cycle inside the measured window — rather
than estimating them: **28,805 of Hazard3's 319,632 measured cycles, 9.01%, are this wait state.**
littlecpu drives `.bus_wait(1'b0)` in this harness and VexRiscv's bus here is always-ready, so neither
of the other two rows carries an equivalent charge. This is disclosed rather than corrected, the same
choice PR #245 made for the two-core VexRiscv pair after finding the equivalent question clean on
both counts there: the wait state is a real, protocol-mandated cost of Hazard3's single-AHB5-port
adapter, not a bug in the harness, and removing it from the count would make Hazard3's row describe a
bus this harness does not build. `soc/compare/dhry_dmips.py` prints the percentage next to the row it
belongs to on every run, so a reader does not have to go looking for it.

**Also confirmed clean**: `soc/compare/dhry_tb.v` counts a write on all three cores the same way,
`|mem_wstrb`/`|mem_wstrb_mux` — any write, not `== 4'b1111` for one of them the way an
already-caught defect in the CoreMark testbench did. There is one write-counting convention in this
file, not three.

### The ISA's cost, isolated from the geometry

`soc/compare/dhry_solo_tb.v` runs littlecpu alone at `soc/compare/dhry_tb.v`'s own geometry — same
8 KB ROM / 16 KB RAM, same linker script, same markers — built at `DHRY_CFLAGS`, `make dhrystone`'s
own native `-march=rv32imac_zicsr_zifencei_zkt`, unchanged. The only variable between this row and
littlecpu's row in the three-way table above is which instructions the compiler was allowed to emit:

| | cycles | cycles/Dhrystone | DMIPS/MHz |
|---|---|---|---|
| **littlecpu, RV32I (shared)** | 300,427 | 751.1 | **0.758** |
| **littlecpu, native rv32imac_zicsr_zifencei_zkt** | 301,227 | 753.1 | **0.756** |

**The shared subset costs this core nothing on Dhrystone, and if anything runs 0.27% faster than its
native ISA on this benchmark** — 300,427 against 301,227 cycles, a reproducible difference in what
the compiler emitted at the two `-march` values (both runs are deterministic RTL simulation of a
fixed program, not a placement with seed variance), not a hardware-capability cost. That is not the
surprising result it first looks like: Dhrystone is string- and pointer-dominated, its few
multiplications are a small fraction of the measured loop, and this core's fetch machinery already
decodes a compressed successor from the fetch window it holds (ADR-0093), so C's code-density win and
M's hardware-multiply win both land on a benchmark that spends almost nothing on either. **This is the
opposite of a hidden asterisk**: the three-way row's 0.758 DMIPS/MHz for littlecpu is not a discount
against some larger native number — on this benchmark, in this harness, it already is the native
number, to within a fifth of a percent.

This does not generalise to every workload — `make dhrystone`'s own shipping-SoC figure (0.664
DMIPS/MHz, ADR-0129) differs from both rows above by more than 0.27%, because that figure is taken at
a different memory geometry (8 KB ROM / 64 KB RAM) with the region-wait fault this harness's map does
not have room to reach. The isolated pair above holds the geometry fixed and varies only the ISA;
`make dhrystone`'s figure holds neither fixed against this table, so it is not a third row of this
same comparison and is not read as one.

### The clock, twelve seeds a side, and the product

`COMPARE_SEEDS='default 1 2 3 4 5 6 7 8 9 10 11' COMPARE_CORES='littlecpu vexriscv hazard3'
soc/compare/sweep.sh`, on `b068533` — the same tree the cycle table above was measured on, so this is
a product rather than the two-trees mistake this ADR's own first amendment already named and reversed
once. Reading it off the distribution rather than one placement is what says the tree has not moved
much since ADR-0139's five-seed table: littlecpu's worst reproduces ADR-0139's 30.90 MHz exactly, and
the two other cores land within a few tenths of a percent of it.

| | worst | median | best |
|---|---|---|---|
| **littlecpu** | 30.90 MHz | 32.00 MHz | 33.06 MHz |
| **VexRiscv** | 48.24 MHz | 49.55 MHz | 50.84 MHz |
| **Hazard3 (iCE40)** | 32.07 MHz | 33.20 MHz | 33.78 MHz |

### Every product the ISA permits, all three at RV32I, worst-of-twelve × the cycle table above

| | DMIPS/MHz | worst-of-twelve MHz | DMIPS |
|---|---|---|---|
| **littlecpu** | 0.758 | 30.90 | **23.42** |
| **VexRiscv** | 0.590 | 48.24 | **28.46** |
| **Hazard3 (iCE40)** | 0.712 | 32.07 | **22.83** |

**VexRiscv leads littlecpu 1.22× on the worst placement of each (1.21× on medians)** — 28.46 DMIPS
against 23.42, from 1.56× on clock against 0.78× on cycles (littlecpu needs 22.1% fewer cycles for
the same work). This is the RV32I three-way row's own number, not the two-core RV32IC pair's 1.05×
prior amendments recorded — different ISA, different image, not the same measurement re-stated.

**Hazard3 and littlecpu are close, in Hazard3's favour on clock and littlecpu's on cycles, netting to
a near-tie**: littlecpu leads 1.03× on the worst placement of each (23.42 against 22.83), 1.03× on
medians too (24.26 against 23.64) — Hazard3's 1.04× clock advantage (32.07 against 30.90) does not
quite cover littlecpu's 6.0% fewer cycles.

**VexRiscv leads Hazard3 1.25× worst-of-twelve (1.24× on medians)** — 28.46 DMIPS against 22.83, from
VexRiscv's 1.50× clock advantage (48.24 against 32.07 MHz) outrunning Hazard3's 17.1% fewer cycles.

None of these are shipped designs, and none of the three implements the ISA the image is built at:
this core is RV32IMAC + Zicsr with traps, VexRiscv here is RV32IC with a branch predictor, no CSR
file and no traps, Hazard3's iCE40 build is RV32IMA with no CSR counters and no interrupt. The DMIPS
column multiplies a clock placed at the harness's 4 KB/2 KB geometry by cycles simulated at 8 KB/16 KB
— no ice40 in this flow has the block RAM to hold Dhrystone at any core's placed size, littlecpu and
Hazard3's own 4 blocks each included. Read on medians instead of worst placements the ranking does
not change, which is what says it is not an artefact of which seed happened to place worst.

### Consequences

- **`make compare-dhrystone` reports four rows, not two**: littlecpu, VexRiscv and Hazard3 in one
  RV32I image and one simulation, plus littlecpu alone at its native ISA in the same geometry. The
  two-core RV32IC pair this ADR previously reported stays true of the tree it was measured on and is
  superseded by the three-way row as the harness's default output; nothing about VexRiscv's own
  number changed, only which comparison `make compare-dhrystone` runs by default.
- **The write-counting and bus-wait-state questions PR #245 checked for the two-core pair are checked
  here for the third**: `soc/compare/dhry_tb.v` counts a write the same way on all three cores
  (`|mem_wstrb`/`|mem_wstrb_mux`, never `== 4'b1111`), and Hazard3's own disclosed bias — the AHB5
  adapter's mandatory write-data wait state — is measured directly from the simulation
  (`wr_pending_q` sampled every cycle of the measured window) and printed as a percentage beside its
  row, not folded into the cycle count silently.
- **The per-core marker/verdict monitor is one module, `soc/compare/dhry_monitor.v`, instantiated on
  each DUT's own bus signal names** rather than copied inline per core — three copies in
  `soc/compare/dhry_tb.v` before this amendment, a fourth near-copy in
  `soc/compare/dhry_solo_tb.v`'s first draft, one module now.
- **The block-RAM question a third core raises was checked, not assumed**: Hazard3's iCE40 build is 4
  `SB_RAM40_4K` standalone, the same as littlecpu's, so it fits the same 32-block budget VexRiscv's
  branch predictor does not. Three cores did not make the harness's arithmetic impossible.
- **The clock is twelve seeds a side, on the same tree the cycle table was taken on, and reproduces
  ADR-0139's five-seed one rather than moving away from it**: littlecpu's worst-of-twelve is
  ADR-0139's own 30.90 MHz exactly. VexRiscv leads littlecpu 1.22× on DMIPS at their RV32I row's
  worst placements (1.21× on medians); littlecpu and Hazard3 are within 3% of each other, littlecpu
  ahead on both readings; VexRiscv leads Hazard3 1.25× (1.24× on medians).
- No `rtl/` file changed. This is entirely `soc/compare/`, the same boundary ADR-0086, ADR-0098 and
  ADR-0139 already draw around this kind of measurement.


## Amendment, 2026-09-04 — re-taken on `e332776`, and the product did not move

The amendment above was measured before executor-only forwarding landed. Re-taking both factors on
`e332776` — twelve seeds a side, one tree, one toolchain, both cores on the conventional
`soc/compare/dhry.lds` layout as always — gives the same product from visibly different halves.

| | worst MHz | median MHz | DMIPS/MHz | DMIPS at worst | DMIPS at median |
|---|---|---|---|---|---|
| **littlecpu** | 30.13 | 30.80 | **0.768** | **23.14** | 23.65 |
| **vexriscv** | 48.24 | 49.55 | 0.557 | 26.87 | 27.60 |

**The product is 1.16× in VexRiscv's favour on worst-of-twelve and 1.17× on median-of-twelve** —
the same 1.16× the previous amendment recorded, reached from halves that both moved:

| factor | previous amendment | here | direction |
|---|---|---|---|
| littlecpu DMIPS/MHz | 0.748 | **0.768** | ours, +2.7% |
| littlecpu worst MHz | 30.90 | **30.13** | theirs, −2.5% |
| cycles, vexriscv/littlecpu | 1.343× | **1.379×** | ours |
| **product** | **1.16×** | **1.16×** | unmoved |

VexRiscv's own columns reproduce to the digit — 48.24 and 49.55 MHz, 1021.9 cycles per Dhrystone,
0.557 DMIPS/MHz — which is the control this pair has always needed: it is a vendored core at a
pinned SHA and it must not move when ours does. `DHRY ramdiff=0 of 4096 words` again, so both cores
still leave identical memory.

**This is the amendment worth reading for method rather than for its number.** Forwarding
(ADR-0154) bought cycles and cost period, the two effects very nearly cancelled in the product, and
a reader quoting either half alone would have concluded the gap had closed or widened depending on
which half they picked. That is the whole reason this ADR insists both factors come off one tree.
It also means **a stale product is not automatically a wrong one** — this one was stale for four
merges and still correct — so staleness is a reason to re-take, never on its own a reason to
disbelieve.

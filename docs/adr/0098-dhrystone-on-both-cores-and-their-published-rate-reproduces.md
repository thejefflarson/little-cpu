# 0098 — Dhrystone runs on both cores in one harness, and their published rate reproduces

Status: Accepted

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
which is most of what its slower clock costs it back.

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

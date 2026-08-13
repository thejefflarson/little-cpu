# 0086 — Both cores in one harness, and the gap is the fetch loop

Status: Accepted · *The 1.6× below is this tree's, 2026-08-08. Re-swept over the same seed set on
2026-08-12 it is 1.48×, and the throughput product it feeds is in
[ADR-0098](0098-dhrystone-on-both-cores-and-their-published-rate-reproduces.md)'s amendment. The
all-NOP demonstration under "the first attempt produced no number" no longer folds this core, and
the amendment at the end says what does.*

## Context

[ADR-0080](0080-twenty-four-megahertz-is-not-reachable-on-the-up5k.md) placed this RTL on an hx8k at
31.08 MHz over 22 logic levels, set it beside VexRiscv's published 63–92 MHz on the same fabric, and
called the result "a real 3× gap on the same silicon". It also wrote down what that comparison was
missing: *"Ours is now on their fabric, but theirs has never been placed on ours."* Their 92 MHz is
their SoC, their memories, their floorplan, their seeds and their synthesis flags; ours is
`make soc-timing`. Two experiments quoted as one.

Nothing in the repository explained the gap either. ADR-0076 deleted the decode head whole and
measured 3.3%, inside the churn band. ADR-0078 ceilinged the whole family of changes that take the
pc off this cycle's decode at 16.32 MHz on up5k — real, and nowhere near 3×. Two cores with
near-identical DMIPS/MHz (0.529 measured here against their published 0.52), both five-stage, both
stall-only with no datapath bypass, and a factor of three nobody could point at.

ADR-0080 recorded generating VexRiscv's Verilog as an open route needing a Scala toolchain. That
turned out to be false: the generated Verilog is already in the tree. `formal/riscv-formal` is
SHA-pinned and ships `cores/VexRiscv/VexRiscv.v`, 3810 lines, for that project's `FormalSimple`
configuration. No sbt, no Scala, no vendoring.

## Decision

**Both cores are placed and timed in one harness, `soc/compare/`, and measured that way the gap is
1.6×, not 3×.**

Five placements a side, default seed plus 1–4, read on the worst of each:

| ns, sorted | worst | spread |
|---|---|---|
| **this core** — 30.56 · 30.62 · 30.71 · 30.90 · **31.70** | **31.70 ns — 31.55 MHz** | 3.7% |
| **VexRiscv** — 18.52 · 18.66 · 19.53 · 19.77 · **19.91** | **19.91 ns — 50.23 MHz** | 7.5% |

**31.70 / 19.91 = 1.59×.** Best against best is 1.65×, so the answer does not turn on which end of
the distribution is read; the two distributions do not come near overlapping either.

It decomposes cleanly, and both terms are real:

| worst placement of each | LUT levels | carry hops | ns per LUT level | logic / routing |
|---|---|---|---|---|
| this core | 22 | 2 | 1.43 | 8.86 ns / 22.87 ns (72.1% routing) |
| VexRiscv | 17 | 0 | 1.17 | 6.51 ns / 13.42 ns (67.4% routing) |

22/17 = 1.29 levels, 1.43/1.17 = 1.22 nanoseconds each, and 1.29 × 1.22 = 1.58 against the 1.59
measured. **The path is a third longer and each level costs a fifth more.** VexRiscv's 17 levels are
the same at all five placements; ours moves between 20 and 23.

This also reproduces ADR-0080's hx8k figure — 31.08 MHz over 22 logic levels there, 31.55 MHz over
22 here — on a smaller SoC, which is the check that this harness is measuring the thing that ADR
measured.

## The harness

`make compare-timing COMPARE_CORE=littlecpu|vexriscv`, and `soc/compare/sweep.sh` for the
distributions. One geometry, held by `soc/compare/geometry_test.sh` and checked on every
`make test`:

- **ice40 hx8k, ct256, three pads** — the clock and two LEDs, the iCE40-HX8K Breakout Board's own
  assignments. hx8k because that is where both sides already had a number, and because it is the
  only ice40 with enough logic to hold this core at all.
- **4 KB of ROM in block RAM and 2 KB of data RAM**, both cores, `rtl/memory.v` instantiated by both
  harnesses at the same base and the same depth.
- **One program image**, `soc/compare/bench.S`, RV32I, built once and split into the two ROM shapes
  each core's fetch wants.
- **Same toolchain, same seeds**: Yosys 0.68+post (`c12172fb`), nextpnr-0.10-108-g68c1acd8, and
  `icetime` read by `soc/timing_split.py` — the same instrument `make soc-timing` uses.

VexRiscv is read out of the pinned clone and never copied into this repository. Its `rvfi_*` outputs
are `delete -port`ed for the standalone reference synthesis, which is `formal/check-nonperturbation.py`'s
technique; left connected they present 556 `SB_IO` and no ice40 package can place them.

## The first attempt produced no number, and that is why there is now a gate

The first version of this harness filled the ROM with NOPs. With no store ever executing every
output of the design is provably constant, so yosys deleted the datapath: **449 placed logic cells
against the 1711 the same core synthesises to on its own**, and a 33.69 ns critical path through the
fragment that was left. `(* keep *)` on the instance held the cell and did not stop the folding
inside it. Nothing in the flow said a word.

So the flow now has `soc/compare/placed_vs_synth.py`: the placed `ICESTORM_LC` count against what the
core synthesises to alone, with no harness to fold against, graded at 0.8×. Measured, both cores sit
above 1.1×. The all-NOP case is 0.26×. It has five probes in `test/probe_gates.sh`, including that
exact 449-against-1711, and `soc/compare/geometry_test.sh` has ten more.

`soc/compare/bench_tb.v` is the second half. The gate says the core is in the netlist; it does not
say the netlist runs, and the piece most likely not to is VexRiscv's bus adapter. So both harnesses
run the one image in one iverilog simulation and are required to publish the **same sequence of
values** to the same address. Two things this caught, both of which had produced a green run:

- the program's mix was an `or`, which saturates the accumulator to all-ones, so every publication
  was the same value and two stuck cores would have agreed;
- every access was word-aligned, so deleting the shift from VexRiscv's byte strobe changed nothing.
  The byte and halfword stores are at offsets 1 and 2 of their words now, and deleting that shift
  fails the bench.

A third defect was in the flow itself: `COMPARE_DEPS` ends in an order-only `|` for VexRiscv, and
everything after a `|` is order-only, so the phony ROM rule stopped forcing a rebuild. `--seed`
reached nextpnr on a netlist make never regenerated, and four "placements" reported one number to
the millisecond. That is what a placement sweep looks like when it is not sweeping.

## The two critical paths, side by side

Both are the fetch loop. That is the finding, and it is not what a 3× gap suggests. The stations
below are the nets `icetime` names along each path, with yosys's mangling stripped — the endpoints
are real cell pins, the ones in between are derived names and read as ancestry rather than as
signals.

**This core, 31.70 ns, 22 LUT levels + 2 carry hops:**

```
ram_8_3 (SB_RAM40_4K) [clk] -> RDATA[14]   2.246 ns before the first LUT
  -> riscv.accessor_out_valid, riscv.decoder.out    decode and the emptiness check
  -> imem.fetch_stall, riscv.accessor.in_is_lw      two of the six stall reasons
  -> imem.odd_first                                 which half of the fetch window
  -> riscv.decoder.pc                               the pc adder, both carry hops
  -> imem.rom_even.0.0_RADDR ...                    the ROM's address decode
  -> imem.rom_even.0.1_RDATA[1]                     the ROM's read-data select register
```

**VexRiscv, 19.91 ns, 17 LUT levels, no carry hop:**

```
IBusSimplePlugin_injector_decodeInput_payload_rsp_inst   a REGISTER holding the instruction
  -> _zz_decode_SRC2, execute_to_memory_MEMORY_STORE     decode and the hazard check
  -> IBusSimplePlugin_decompressor_output_ready          the compressed decoder's handshake
  -> IBusSimplePlugin_iBusRsp_stages_0_output_ready
  -> IBusSimplePlugin_fetchPc_output_ready
  -> IBusSimplePlugin_fetchPc_correctionReg              an ordinary flip-flop
```

Other placements of this core route the middle of that loop differently — one goes through
`csrs.mcause` and `imem.in_range2` instead — but the two ends do not move, and neither does the
count of levels between them.

## What the difference actually is

**Their fetch loop does not contain the instruction memory; ours contains it at both ends.** Ours
starts at a block RAM's clock-to-output and spends **2.246 ns — 7% of the whole period — before the
first LUT**, and ends inside the ROM's own address decode, because the fetch address is published a
cycle early and a stalled cycle re-presents the same words. That is the no-wrong-path-state
commitment, which is what keeps the BMC depths small and `pcloop`'s induction free of speculative
state. Theirs starts and ends at ordinary flip-flops: the instruction is already registered when the
loop begins, and the bus command leaves the cycle after the pc is written.

**Their loop is a ready chain; ours is a decode-and-commit path.** Theirs is
`decompressor_output_ready → iBusRsp_stages_0_output_ready → fetchPc_output_ready` — stage
handshakes, one bit wide. Ours is `decoder.out`, the four-slot emptiness check
(`accessor_out_valid`), two of the six stall reasons and then the pc adder. This core detects and
commits every trap in decode, so `csrs.mcause` is in that neighbourhood too and turns up on the
critical path of another placement; VexRiscv's configuration here has no CSR file and no traps at
all, so it has nothing to put there.

**They have a branch predictor and we forbid one.** 14 of their 18 block RAMs are
`IBusSimplePlugin_predictor_history`, 1024 entries of 55 bits. Wrong-path state is exactly what this
design commits not to have.

**The rest is nanoseconds per logic level, and this harness cannot separate it from utilisation.**
Ours places at 96% of the part, theirs at 31%. Congestion is pressure on the longer path and not on
the shorter one. Controlling for it would need a bigger ice40 and there is not one.

## Area, and the block RAM claim corrected

| | placed `ICESTORM_LC` | core alone, `SB_LUT4` | placed / synthesised | block RAM in the harness |
|---|---|---|---|---|
| this core | 7400 / 7680 (96%) | 6611 | 1.12× | 16 — 4 register file, 8 ROM, 4 data RAM |
| VexRiscv | 2379 / 7680 (31%) | 1711 | 1.39× | 30 — 4 register file, **14 branch predictor**, 8 ROM, 4 data RAM |

**Their register file is the same 4 block RAMs ours is.** The 18 blocks VexRiscv synthesises to on
its own are 4 for `RegFilePlugin_regFile` and 14 for the branch predictor's history table. That
matters for anyone sizing a part: the ROM in this harness is 4 KB rather than the shipping 8 KB
because a branch predictor had to fit beside it, not because of a register file.

## What is different between the two designs, stated so nothing here is quoted as like-for-like

**This is not a comparison of two shipped designs and must not be presented as one.**

- **The ISAs differ, and not the way ADR-0080 recorded.** The generated core is **RV32IC** — the
  compressed decoder is there, `IBusSimplePlugin_decompressor` and all — with **no M extension, no
  CSR file, no traps and no interrupt**. Ours is RV32IMC_Zicsr_Zifencei with the full mandated M-mode
  CSR set, five traps and a machine timer. ADR-0080's "every VexRiscv iCE40 number is RV32I" is
  corrected for this artefact: C is on both sides, M and the privileged architecture are on ours
  alone.
- **This is not the 92 MHz configuration either.** That row is 1130 LC of "small, no datapath bypass,
  no interrupt". This one is 1711 `SB_LUT4` with a compressed decoder and a 1024-entry branch
  predictor. What it does share with that row is the stall-only interlock: `HazardSimplePlugin`'s
  bypass conditions are constant-true here, so it stalls on every hazard, as this core does.
- **The 92 MHz is not reproduced.** In this harness, on this toolchain, that core places at
  50.23 MHz at its worst of five and 54.00 at its best. So a large part of the published 3× is not in the silicon at
  all — it is the difference between their flow and this one, and this harness cannot say how much of
  it is their floorplan, their memories, a PLL, `--freq`/`--opt-timing`, or a different
  configuration. **It bounds our side, not theirs.**
- **The harness is neither design's own SoC.** The shipping SoC is 8 KB of ROM and 64 KB of SPRAM on
  an up5k; this is 4 KB and 2 KB of block RAM on an hx8k, because hx8k has no SPRAM and 32 block RAMs
  to divide between two cores. `make soc-timing`'s number and this one are not comparable.
- **`rtl/timer.v` is not in the harness.** With it this side is 7829 logic cells against the part's
  7680 and does not place. `irq_timer` is driven from a free-running counter rather than tied off, so
  the core's interrupt path is still real logic and the 324 cells left out are the peripheral's.
- **No `-dsp`.** hx8k has no `SB_MAC16`, so this core's multiplier is soft logic here — 6611
  `SB_LUT4` against the 3921 + 4 `SB_MAC16` it maps to on up5k. VexRiscv has no multiplier to map.
- **The image is RV32I.** It has to be, to be one image. So this core's multiplier, divider,
  compressed decoder and CSR instructions are in the placed netlist and never issued, which does not
  move a static timing number.

## Consequences

- **The 3× is retired as a single number.** What is measured, one harness, five placements a side, is
  1.6×. Anything past that is a flow difference this harness does not resolve, and a proposal that
  starts from "they are three times faster" is starting from two experiments added together.
- **The answer to "where is it" is the fetch loop, on both sides.** That agrees with ADR-0076, which
  found no single input to `next_pc` worth more than 5% and all of them together worth 21%, and with
  ADR-0078, which priced collecting that 21% at the no-wrong-path-state commitment. **This ADR
  reopens neither.** It says the comparison points at the same place those two already measured, so
  their ceilings are the numbers to beat, not this one.
- **Two of ADR-0080's statements are corrected**, both about the artefact rather than about its
  conclusions: the core is RV32IC, not RV32I, and its 18 block RAMs are mostly a branch predictor
  rather than a register file. ADR-0080's own measurements are unchanged, and this harness reproduces
  its 31.08 MHz over 22 levels to within the placement spread.
- **`soc/compare/` is a measurement, not a gate.** Nothing in it grades the shipping design, nothing
  in `rtl/` changed, and `SOC_MIN_MHZ` is untouched. The one graded thing is the placed-versus-
  synthesised check, which grades the harness rather than the core, plus the geometry check that
  hangs off `make test` because nothing else on a developer's machine would notice the harness
  rotting.
- **A harness that measures an optimised-away design is now a checked failure rather than a habit.**
  Three separate defects in this one flow produced a green run and a number: the folded core, the
  saturating program, and the sweep that never re-placed. The first two are checks now; the third is a
  fixed prerequisite order with the reason written beside it.

## Amendment, 2026-08-12 — the all-NOP stimulus no longer folds this core, and what does

The all-NOP ROM above is quoted as the demonstration that this gate discriminates. **On this core's
side it no longer does.** Re-run live rather than trusted, on `d8ecfa2` with the toolchain this ADR
records — Yosys 0.68+post (`c12172fb`), nextpnr-0.10-108-g68c1acd8 — one placement at the default
seed each, the placed `ICESTORM_LC` against the same standalone `SB_LUT4` the gate reads:

| stimulus | this core, against 6006 | verdict | VexRiscv, against 1711 | verdict |
|---|---|---|---|---|
| the real program, `bench.S` | 6635 — **1.10×** | green, 30.21 ns | 2356 — **1.38×** | green, 20.66 ns |
| all-NOP ROM image | 6628 — **1.10×** | **green — no fold at all** | 1101 — **0.64×** | red |
| both pads tied off | 1 — 0.00× | red, 0.24 ns | 1 — 0.00× | red |
| reset held asserted | 1 — 0.00× | red | 1 — 0.00× | red |
| a constant instruction in place of the memory's answer | 125 — **0.02×** | red, 10.59 ns | 1 — 0.00× | red |
| all-NOP image, text write port disconnected | 129 — **0.02×** | red, 10.63 ns | no write port to remove | — |
| a core input left unconnected, as found | 536 — **0.09×** | red, 19.70 ns | not applicable | — |

The ratios differ by factors of fifty, so nothing here turns on which placement is read and no sweep
was taken; the whole-design rows reproduce the amended clock figures to within their spread.

**The reason is the write port, and yosys's own memory map says so in one line.** Under the NOP
image `bench_littlecpu.imem.rom_even` and `imem.rom_odd` are still `mapping memory ... via
$__ICE40_RAM4K_`; disconnect `mem_wstrb` and both disappear from that list entirely. A memory the
design writes cannot be a constant whatever it was initialised with, so the instruction word is not
a constant, so the decoder that reads it is live and the datapath behind it with it. VexRiscv's
`bench_vexriscv.rom` is read-only, and it is exactly the memory that leaves the map under the same
image — 30 block RAMs to 21 — while its register file, its data RAM and its branch predictor's
history stay. **Text became writable for the `fence.i` reason and took a red direction with it as a
side effect nobody chose.**

**So the shape of a stimulus that works here is not "make the contents uniform" but "make the
datapath dead".** Three do. Tying the harness's two pads off is the most direct statement of the
property under test and deletes everything, which makes it a poor diagnostic — one placed cell is
obviously wrong, where the defect this gate exists to catch looks like a measurement. The other two
leave a plausible fragment and a plausible number: a core handed a constant instruction instead of
the memory's answer places 125 cells and icetime times them at 94.41 MHz, and the all-NOP image with
the write port taken away places 129 at 94.10 MHz. **The NOP image and that last row differ in one
connection**, which is what makes it the successor to the founding experiment rather than a new one.

**The founding 0.26× is a weaker red than it was.** The same image against the same SHA-pinned
VexRiscv now leaves 1101 of 1711 rather than 449 — 0.64× against a 0.80× floor, 0.16 of margin where
there was 0.54 — because the harness it was first measured in is not this one, and in this one the
predictor's write-driven history table holds most of that core up on its own. The verdict travelled
and the ratio did not. A red direction with 0.16 of margin is one flow change from expiring on that
side too, so the number to re-take on a pin bump or a harness edit is this one, not the 449.

**While this was being measured the gate went red for real, on this core's side, for a defect
already merged.** Adding an instruction access fault gave `littlecpu` an `imem_fault` input and
`imemory` an output to drive it, and `soc/compare/bench_littlecpu.v` was not in that change:
the input was left unconnected. Yosys said `Warning: Wire ... is used but has no driver` and
carried on; the harness synthesised to 473 `SB_LUT4`, placed 536 cells, and icetime reported
**19.70 ns — 50.77 MHz, better than the real design's 30.21** for a design that was one sixth of the
core. `make compare-timing` is not on CI, so nothing had failed. That is the founding defect
recurring from a different cause, caught by the check written for it, and the fix is in this
change. **`make compare-smoke` could not have caught it**: it publishes the same six values before
and after the fix, to the byte, because `bench.S` never fetches outside the ROM window and so never
asserts the signal that was floating.

**A red direction is a property of the design at the moment it was taken.** This is the third this
week to expire because the design improved rather than because it broke — `test/asm/selfmod.S` when
the bus transaction moved to the execute slot, `fence.i`'s half of serialization structurally, and
now this one when text became writable. A fixture probe survives that, because it drives the
comparison against numbers on disk and asks only whether the comparison can fail. A live stimulus
does not, because it asks whether *today's* design still fails, and the answer is allowed to change
without anyone touching the check. `test/probe_gates.sh`'s five probes for this gate are unchanged
here and still pass, and they are what keeps it honest in CI; this amendment is what keeps the
live demonstration true.

Nothing above moves the floor. 0.02× and 0.09× clear 0.80× by more than an order of magnitude, and
1.10× and 1.38× sit above it by a quarter and a half, so no measurement here argues about where it
sits.

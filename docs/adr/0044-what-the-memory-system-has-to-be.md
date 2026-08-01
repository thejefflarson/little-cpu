# ADR-0044: What the memory system has to be, and why today's placeholders cannot be it

**Status:** Accepted · 2026-08-01 · *Records constraints, not a design. Supplements ADR-0003 and
ADR-0008; the design it enables is post-M4 and needs its own ADR.*

## Context

`CLAUDE.md`'s opening says the eventual home is an ice40 up5k. Two things have since been measured
that the sentence does not account for.

The first is area, recorded in ADR-0038 and
[`docs/ideas/fit-the-core-on-the-up5k.md`](../ideas/fit-the-core-on-the-up5k.md): the core is at
132% of the part, and the regfile change takes it to 80% (4236 cells, PR #62).

The second is this ADR. **`rtl/imemory.v` and `rtl/memory.v` are placeholders that cannot be built
on this part, in four independent ways**, and nothing in the repo says so. They elaborate, they
simulate, they are instantiated by `rtl/littlesoc.v`, and they are unbuildable. That gap has been
invisible because no gate looks at it: `make fit` synthesizes `littlecpu` with memories *external*,
by design (ADR-0038 decision 1), and the SoC has never been placed.

This ADR is the memory-system analogue of the area brief. It records what is true about the part so
the eventual design starts from constraints rather than from a fifteen-line placeholder.

## What is true about the hardware

| resource | size | read | initialisable from bitstream |
|---|---|---|---|
| `SB_RAM40_4K` (BRAM) | 30 × 4 kbit = **15 KB** | **synchronous** | yes |
| `SB_SPRAM256KA` (SPRAM) | 4 × 256 kbit = **128 KB** | **synchronous** | **no** |

**There is no combinational-read memory primitive on this part**, and the only large one cannot be
initialised at all.

## What the placeholders ask for

```
imemory   62 KB  × 2 (rtl/littlesoc.v instantiates it twice for the second read port) = 124 KB
memory    62 KB
                                                              TOTAL DEMAND  186 KB
                                                  AVAILABLE    128 + 15    = 143 KB
                                                                  OVER BY    43 KB
```

## The four constraints any design must satisfy

**1. Instruction fetch reads combinationally, and no primitive does.** `rtl/imemory.v` is
`always_comb`. Invariant 1 requires the decoder to see the instruction at the current PC in the
same cycle it decides the next one. Every ice40 memory is synchronous.

**2. Fetch needs two words per cycle, and `littlesoc.v` gets them by duplicating the ROM.**
ADR-0003's straddle window reads `imem_addr` and `imem_addr + 4` every cycle so a 32-bit
instruction crossing a 4-byte boundary costs nothing. `rtl/imemory.v` has one port, so
`rtl/littlesoc.v` instantiates it twice. **That doubles the storage rather than adding a port**,
and it is most of why the demand above exceeds the part.

**3. 62 KB does not fit in BRAM, so the ROM must be SPRAM — which cannot be initialised.**
`initial $readmemh("./rom.mem", rom)` is not merely unsynthesizable; `SB_SPRAM256KA` has no INIT
capability at all. **A ROM in SPRAM requires a runtime boot path** that copies from external SPI
flash. `rtl/littlesoc.v`'s flash-pin scaffolding and its `// TODO SPI mem` were gesturing at exactly
this before they were removed as dead.

**4. The data memory is closer, but its byte strobes do not map cleanly.** `rtl/memory.v` is
already `always_ff` — synchronous, which matches the hardware. But it takes a 4-bit *byte* strobe,
and SPRAM takes nibble masks over a 16-bit word. That mapping is a real detail, not a rename.

## Decision

**Record these as constraints on the eventual design. Do not treat the placeholders as a starting
point — they are a specification of the interface, not a sketch of the implementation.**

Specifically:

- **`make fit`'s `littlecpu`-with-memories-external top stays** (ADR-0038 decision 1). It measures
  the core, which is the thing the core's own work changes. A separate SoC-level measurement is
  what the memory design needs, and it is that design's job to produce it.
- **`CLAUDE.md`'s up5k claim is an intent, not a statement of fact**, and is annotated as such
  alongside the area pointer. Between here and hardware sit an area gap, a memory system that does
  not exist, and no timing measurement of any kind.
- **The techniques below are recorded because they are known and constrain the search, not because
  they are chosen.** Choosing among them is the post-M4 ADR's job, taken with measurements this
  repo does not yet have.

## Techniques that are known to work — and the one that is ruled out

**Whatever this design chooses must be modellable by the formal ladder.** That is the constraint
that eliminates the otherwise-obvious answer below, and it is not negotiable on a core whose whole
plan is to get back to verified.

**For the combinational-read problem — clock the memory address from the combinational *next* PC at
posedge**, rather than from the registered PC. The decoder then sees the instruction available for
the whole of the following cycle, and **invariant 1 survives exactly**: no extra cycle, no
speculation, no flush. It also *shortens* the critical path, because the memory access leaves the
combinational chain that currently runs `pc → imem_addr → memory → window mux → decode → next pc`.

**A negedge read is NOT a candidate, and this is the place that says so.** It is the obvious second
answer, it is newly unblocked now that the regfile no longer needs the falling edge, and it must
still be rejected — for the same reason the regfile rejected it (ADR-0042, and the measurement in
ADR-0040).

**The formal ladder cannot model a negedge read at all.** sby's own `prep` step runs
`formalff -setundef -clk2ff`, which fails closed with `ERROR: CLK clock ... also used with opposite
polarity`, rc=16, before any check runs. That is not a tuning problem: `clk2fflogic` is the flag
that makes it run, and ADR-0040 measured that the flag *produces false greens* — under it `reg_ch0`
passes on designs the stock ladder kills. `checks.cfg` structurally cannot express a correct
version, because genchecks derives `skip`, `depth` and `RISCV_FORMAL_CHECK_CYCLE` from one column
while `clk2fflogic` makes a cycle two BMC steps.

Choosing a negedge for the regfile would have meant `reg_ch0` never running against
`rtl/regfile.v` again. Choosing one for instruction fetch is worse: **every `insn_*` check on the
ladder reads through `imem_data`**, so it would take the entire per-instruction half of the ladder
out of contact with the shipping design. An unverifiable memory on an unverified core is not a
trade this project can make.

**For the dual-word problem — interleaved 16-bit banks**, as the deferred list already names. Even
halfwords in one bank, odd in the other; any 32-bit window at any 2-byte alignment then reads one
halfword from each. That replaces duplication with banking and removes the 62 KB of waste that
duplication costs.

**For the initialisation problem — a boot path from SPI flash**, or a ROM small enough to live in
BRAM. These are different products: the second bounds program size to under 15 KB and keeps the
system self-contained; the first costs a bootloader and a flash controller.

## Consequences

- The memory system is **post-M4 work with its own ADR and its own brief**, and it is larger than
  "resolve invariant 1" — it is a bootloader, a banking scheme, and an address-timing decision.
- **Nothing here blocks M2 or M3.** It blocks running on hardware. The sequencing is fine; what was
  wrong was that the gap was undocumented.
- **No timing number exists or can exist until this lands.** `icetime` needs a placed design, which
  needs a real pinout, which is this work. Every Fmax statement in this repo is therefore reasoning,
  and ADR-0038's declared 12 MHz stands as an intent rather than a measurement.
- `rtl/imemory.v` and `rtl/memory.v` keep their current shape until then. They are correct as a
  *simulation* model and as a statement of the interface the core expects, and replacing them
  before the design exists would only move the placeholder.
- **Invariant 1 is the last combinational-read requirement in the design** once the regfile lands.
  Whatever this work chooses is therefore the final word on whether that invariant survives contact
  with the hardware — which is why it deserves its own decision rather than being settled as a side
  effect of an area sprint.
- **The design space is narrower than it looks, and deliberately so.** Ruling out the negedge on
  verifiability leaves the next-PC-clocked address as the only known technique that keeps invariant
  1 intact *and* stays modellable. If that turns out not to work, the honest options are to change
  invariant 1 — with its own ADR, per ADR-0038's precedent for invariant 6 — or to change the
  target part. Reaching for the negedge because the remaining options are uncomfortable would be
  buying hardware that works with a core nobody can check, which is the trade this repo has spent
  the whole rewrite refusing.

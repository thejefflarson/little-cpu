# ADR-0038: Area is measured in logic cells, Fmax is declared at 12 MHz, and two area levers are rejected permanently

**Status:** Accepted · 2026-08-01 · *Settles the measurement standard and the rejections from
[`docs/ideas/fit-the-core-on-the-up5k.md`](../ideas/fit-the-core-on-the-up5k.md). The regfile
decision that brief recommends is deliberately **not** made here — it needs the spike data that
brief gates it on.*

## Context

`CLAUDE.md` names an ice40 up5k as the eventual home. The first place-and-route this project has
ever run says the core does not fit:

```
ICESTORM_LC:    6659 / 5280   126%   ← FAILS TO PLACE
ICESTORM_RAM:      0 /   30     0%
ICESTORM_DSP:      4 /    8    50%
ERROR: Unable to place cell 'regfile.regs[6]_SB_DFFE_Q_21_DFFLC', no BELs remaining
```

Planning that number produced two wrong estimates in a row, in opposite directions, and both had the
same cause. This ADR fixes the cause and records the two levers that were examined and rejected, so
they stop resurfacing.

## Decision 1 — area is measured in nextpnr logic cells, never yosys cell counts

```
4855 LCs used as LUT4 only
 524 LCs used as LUT4 and DFF     ← only these share a cell
1228 LCs used as DFF only
  37 LCs used as CARRY only
```

Yosys reported 5379 `SB_LUT4` for the same design that nextpnr places as 6659 LCs. A logic cell
holds one LUT4 **and** one flip-flop, but a DFF whose D input is not the output of its co-located
LUT consumes a whole cell on its own. 1228 of this design's cells are exactly that. **A LUT4 count
is structurally blind to it.**

That blindness cost two estimates:

- an estimate of **102%** utilization where the truth was **126%** — LUT4 count, missing the
  unpaired flip-flops;
- an estimate that moving the regfile to block RAM recovers **~1024** cells, when it also deletes
  the read-port mux trees and address decode and is worth **more than twice** that — the same
  blindness in reverse, undercounting the *saving* because the saving is mostly LUT4s attached to
  the flip-flops.

**So: the fit metric is `nextpnr-ice40 --up5k --package sg48` on `littlecpu` with memories
external, reported by a `make fit` target. Yosys cell counts are not evidence about area and must
not appear in a planning document.** Synthesis top for the metric is `littlecpu`, not `littlesoc`,
because the SoC's memories are placeholders whose real implementation will be SPRAM and will not
consume logic cells.

## Decision 2 — Fmax is declared at 12 MHz, and raising it requires an ADR

Invariant 1 (fetch is combinational, the decoder owns the PC) and invariant 6 (the regfile is
combinational-read) together put

```
pc → imem_addr → imem_data → window mux → decode → regfile read → branch compare → next pc
```

in a single cycle **by design**. A low Fmax is a consequence of a stated project value, not a
defect, and this repo optimizes for readability over throughput on purpose.

The first timing data available — `rtl/regfile.v` placed alone — measures its combinational read
path at **2.04 ns logic + 12.78 ns routing = 14.82 ns**, on hx8k, which is *faster* silicon than
up5k. 86% routing, because a distributed 32:1 mux is a routing problem.

**Declare 12 MHz** (the iCEBreaker oscillator) and record `icetime` output in `make fit` once the
design places. At 12 MHz a half-period is 41 ns — enormous relative to any path measured so far.

> **Measured at [ADR-0054](0054-the-memory-system-and-the-first-real-timing-number.md), and the
> design does NOT close 12 MHz**: the SoC places at **88.51 ns = 11.30 MHz** (`icetime`; 11.70 by
> nextpnr's own analysis), **34.20 ns logic (38.7%) + 54.29 ns routing (61.3%)**, on the
> `imem.in_range → decode → next PC → imem.in_range2` loop. Two things this paragraph got wrong.
> The half-period is not the budget — the whole 83.33 ns period is, and the design is 6% over it.
> And the report lands in **`make soc-timing`**, not in `make fit`: the two targets measure
> different designs and their numbers must not be merged. **The 12 MHz declaration stands
> unchanged** — reconciling it with a measurement is a decision, not a consequence, and this
> paragraph's own test still applies to anyone proposing to close the gap.

**Anyone proposing to raise Fmax is proposing to break invariant 1 or invariant 6, and must bring an
ADR that says which.** This is recorded so that a future performance complaint is answered with the
design's own reasoning rather than a reflexive optimization.

## Decision 3 — two area levers are rejected permanently

**Sharing one adder across add / sub / branch-compare / address-generation.** The classic area win,
and wrong here twice over. Cross-stage sharing is impossible — decode and execute operate on
different instructions in the same cycle. Within the executor, folding the operations onto one adder
saves an estimated **60–120 cells**: 32-bit carry chains are cheap on ice40 and ABC already shares
some. That is roughly **2% of the gap**, bought by destroying the one-operation-per-line
`case (1'b1)` structure — in the module whose own comments call it "the module whose legibility
matters most," and for which this repo already accepts 20 iverilog `sorry:` notes rather than obscure
it.

**Radix-4 division.** The deferred list already records that it "roughly doubles comparator and mux
logic." It is a CPI lever that *increases* area. It does not belong in an area pass.

**Deferred with an explicit trigger, not rejected:** narrowing the divider datapath from 64-bit to
the textbook 33-bit shift-remainder form (~150–250 cells, readability neutral). Only if margin falls
below 15% after the regfile work, and only after M2 — the divider is the one datapath in this core
with no formal oracle in any form (ADR-0010), so it is the worst possible place to make an
unforced change.

## Decision 1a — the metric cannot reach "it places", and the ratchet is on utilisation

Added after this ADR's first draft, from measurement. The draft said `make fit` would be
"report-only until the design places and a ratchet after." **That trigger can never fire.**

`littlecpu` with memories external presents **231 `SB_IO` against sg48's 39**. Even the 76%
configuration below fails placement — on `imem_data2[24]$sb_io`, not on logic cells. A top with
realistic IO means a real pinout, which means the SoC memory system, which this work puts out of
scope.

**So the ratchet is on the logic-cell utilisation nextpnr prints *before* it attempts placement, and
`make fit` deliberately tolerates the IO placement error.** `icetime` and any "the design places"
claim wait for the SoC memory work. Fmax stays declared at 12 MHz and unmeasured, per decision 2.

> **The SoC memory work landed (ADR-0054) and this decision is unchanged.** `rtl/littlesoc.v` places
> with **4 `SB_IO`** — both memories are internal, so there is no external bus — and
> `make soc-timing` is where it is measured. `make fit`'s top stays `littlecpu` with memories
> external, still tolerating its IO error, because it measures the thing the core's own work
> changes. **Two numbers, two targets, and they are not comparable**: 3875 LC for the core against
> 4041 for the SoC, the difference being the ROM's depth mux, the RAM's range decode and two LED
> taps.

A `make fit` that required successful placement would never run at all — which is worse than no
metric, because it would look like one.

## Measured, after the estimates

All figures `nextpnr-ice40 --up5k --package sg48`, `littlecpu`, memories external:

| Configuration | LC | % | EBR |
|---|---|---|---|
| Baseline, post-trap-entry | **6971** | **132%** | 0/30 |
| + merged shifter | 6835 | 129% | 0/30 |
| **+ negedge-EBR regfile** | **4017** | **76%** | **4/30** |
| + both | 3998 | 75% | 4/30 |

Two things this settles, both against earlier estimates in this repo:

- **The regfile is sufficient on its own.** Not "necessary but not sufficient" (the ticket that
  preceded this ADR), and not "close to sufficient" (the design brief). 132% → 76%, one lever.
- **The shifter merge is not an area lever.** 136 logic cells standalone and **19** on top of the
  EBR regfile — 0.4% of the part in the configuration that would ship, against a 300–450 estimate.
  The same "ABC already shares some" argument decision 3 uses to reject adder sharing applies here
  too, just less strongly. It must justify itself on readability alone or not at all.

## Consequences

- A `make fit` target becomes the tracked measurement — an **LC-utilisation** ratchet per decision
  1a, not a placement gate.
- The Makefile's existing `riscv.json` / `riscv.asc` / `timing` targets **measure nothing and must
  go**: they synthesize `littlesoc`, whose only outputs are the flash pins, so yosys deletes the
  entire core and P&R reports **4 LCs, 0%**. The repo cannot carry two fit numbers that differ by
  6967 cells.
- `CLAUDE.md`'s up5k claim stays false until the fit work lands, and should be read as an intent
  rather than a statement of fact until then.
- The two rejections are recorded here specifically so a future reader proposing them finds the
  measurement and the reasoning rather than re-deriving both.
- **The negedge-BRAM regfile is deliberately not decided here.** It trades against invariant 6 and
  needs its own ADR, written with the spike data the design brief gates it on — formal wall-time
  under `clk2fflogic`, EBR inference behaviour, and confirmation that the Sail co-simulation's
  `debug_items` probe still reads the real register array. Deciding it in advance of that data is
  exactly the failure this ADR's first decision exists to prevent.

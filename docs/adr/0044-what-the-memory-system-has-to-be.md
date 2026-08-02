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

## Consequences for the formal ladder — eleven declined checks reopen, and invariant 2 comes under pressure

Everything above is about hardware. This section is about the ladder, and it is the part that will
be hardest to reconstruct later, because the thing it depends on is an *absence*: **this core's bus
cannot refuse a transaction.** `imem_data`/`imem_data2`/`mem_rdata` are free every cycle, with no
fault line and no handshake — invariant 1, ADR-0015, and `formal/wrapper.v`, which speaks that bus
directly. A memory system that can say *no* is a different bus, and eleven of the checks
`formal/checks.cfg` declines are declined against the bus we have.

**The count is derived, not remembered.** `formal/genchecks-audit.py` reports, on this tree,
`85 generated, 14 declined for want of a [depth] line`, and set-equalities that 14 against
`checks.cfg`'s `#omit` lines in both directions. Of those fourteen:

**Eleven are conditional on a memory system that does not exist. Named, so the burn-down is
mechanical:**

| check | what it needs that today's bus cannot give |
|---|---|
| `fault_ch0` | `rvfi_mem_fault{,_rmask,_wmask}` — a transaction the memory refused |
| `bus_imem_ch0` | `RISCV_FORMAL_BUS` |
| `bus_imem_fault_ch0` | `RISCV_FORMAL_BUS` + a faulting bus |
| `bus_dmem_ch0` | `RISCV_FORMAL_BUS` |
| `bus_dmem_fault_ch0` | `RISCV_FORMAL_BUS` + a faulting bus |
| `bus_dmem_io_read_ch0` | `RISCV_FORMAL_BUS` + a distinguished MMIO region (`rvformal_addr_io`) |
| `bus_dmem_io_read_fault_ch0` | the same, plus a faulting bus |
| `bus_dmem_io_write_ch0` | `RISCV_FORMAL_BUS` + `rvformal_addr_io` |
| `bus_dmem_io_write_fault_ch0` | the same, plus a faulting bus |
| `bus_dmem_io_order_ch0` | `RISCV_FORMAL_BUS` + `rvformal_addr_io` + more than one outstanding access |
| `causal_io_ch0` | `rvformal_addr_io` |

**Two of those eleven carry a standing second argument that no memory design touches**:
`bus_imem_ch0` and `bus_dmem_ch0` would re-derive a property `formal/imemcheck.sv` and
`formal/dmemcheck.sv` already hold against the real split `imem_addr`/`imem_addr2` interface, and
adopting them means driving nine more RVFI outputs — which ADR-0047's non-perturbation check has an
interest in. Expect those two to be re-tagged as declined-on-the-merits rather than adopted. They
are listed here anyway, because a ruling made once and written down beats a check that quietly
never gets reconsidered.

**Three are not conditional at all, and no memory design reopens them**: `csrc_inc_mcycle_ch0` and
`csrc_inc_minstret_ch0` (red on a *correct* core — a defect in `rvfi_csrc_inc_check.sv`'s shadow
model, measured and swept in `checks.cfg`'s `[csrs]` block), and `cover` (relocated to
`formal/cover.sby` with this repo's own five goals). That eleven/three split is why `checks.cfg`'s
`#omit` lines now carry a `[BLOCKED]`/`[DESIGN]` tag: both classes read identically as prose, and a
future reader cannot otherwise tell a burn-down item from a settled decision.

### `fault_ch0` is where the ladder and invariant 2 meet

`checks.cfg`'s omit line for it said the check *"models a post-decode trap invariant 2 forbids"* —
true, and stated as though the matter were settled. It is not settled; it is *unreached*. The two
propositions only coexist because nothing on this core can raise an access fault:

- **Invariant 2** says every trap is detected **and committed** in decode, and that this is what
  makes CSR commit precise with no reorder buffer.
- **An access fault is raised by the memory**, which answers after decode has already committed.

So the memory system does not merely unblock eleven checks. **It forces a ruling on one of the nine
invariants** — the ones `CLAUDE.md` opens by saying are a bug to violate even if the tests pass.

Read from `checks/rvfi_fault_check.sv` at the pin (`c992aa61`) rather than from memory, because the
check's shape narrows the question usefully. It splits into three cases by fault mask, and they are
not equally hard:

- `wfault` → asserts `mcause == 7` (store access fault) — **after decode**
- `rfault` → asserts `mcause == 5` (load access fault) — **after decode**
- `ifault` (neither mask set, `insn == 0`) → asserts `mcause == 1`, instruction access fault —
  which arrives **with the fetched instruction**, in the cycle decode is looking at it, and is
  therefore the one case that could be committed in decode with invariant 2 untouched.

One more thing that makes the burn-down concrete: the whole `mcause` half of that check sits under
`` `ifdef RISCV_FORMAL_CSR_MCAUSE ``, and `genchecks` emits that define only for CSRs named in
`[csrs]` — where `mcause` is deliberately absent for the WARL reason written out there. **Reopening
`fault_ch0` is therefore three things, not one**: a bus that can refuse, `rvfi_mem_fault*` driven
from `rtl/`, and `mcause` on the RVFI CSR set (which is blocked on its own, unrelated grounds).

### The options — recorded, not chosen

Three, and they are visible now in a way they will not be once a design is half-built:

1. **No faulting bus at all.** An address-decoded system where every access in range is answered
   and nothing out of range is reachable. Invariant 2 survives untouched, and every `*_fault` check
   plus `fault_ch0` is declined permanently — those five `#omit` lines become `[DESIGN]`. The
   remaining five `bus_*` and `causal_io_ch0` are decided separately: by whether the map ends up
   with a distinguished IO region, and by the `imemcheck`/`dmemcheck` argument above.
2. **Faults resolved in decode by construction.** The memory map is a static address-range decode
   that decode itself can evaluate from the address it already computes — the same
   `$signed(immediate) + $signed(reg_rs1)` the misalignment check reads. There is no *dynamic*
   refusal, so a fault is a decode-time property of the address and invariant 2 holds as written.
   `fault_ch0` becomes reachable at least for `ifault`; `rfault`/`wfault` follow only if the range
   decode is exact.
3. **Invariant 2 changes**, with its own ADR, its own precision argument, and a rewrite of the
   ladder's post-decode trap story.

**This ADR picks none of them**, deliberately and for the same reason it declines to pre-decide the
memory design at all: the choice belongs to the eventual ADR, taken with measurements this repo
does not have. ADR-0038 set that precedent by refusing to decide the regfile before the spike data
existed, and ADR-0042 then decided it *with* the data — including a cycle cost nobody would have
guessed at (+18.0%) and a verifiability argument that reversed the area-only recommendation.
Guessing here would produce a decision of the quality of the one ADR-0038 refused to make.

**What the eventual memory ADR owes this one**: a ruling on each of the eleven checks *by name*,
with each `#omit` line either deleted (the check joins the ladder), re-tagged `[DESIGN]` (it is
declined permanently, on the merits), or left `[BLOCKED]` with what it is still blocked on. That is
the same burn-down shape as `formal/EXPECTED_FAIL`, and it is why the eleven are tabulated above
instead of described.

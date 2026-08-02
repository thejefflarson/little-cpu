# ADR-0054: The memory system, and the first real timing number

**Status:** Accepted · 2026-08-01 · *The design ADR-0044 said the memory system would need. Adopts
ADR-0044's constraints, decides the two it left open, and answers ADR-0038 decision 2's standing
question with a measurement instead of an argument.*

## Context

Every Fmax statement in this repo has been reasoning. The one real figure anyone had taken was
`rtl/regfile.v`'s combinational read placed alone on an hx8k — **2.04 ns logic + 12.78 ns routing =
14.82 ns, 86% routing** (ADR-0038 decision 2) — and it was useful precisely because of the
breakdown, which said the cost was wires rather than gates.

A whole-design number was blocked behind a chain: `icetime` needs a placed design, placement needs a
real pinout, a pinout needs a memory system that can exist on the part, and ADR-0044 recorded
`rtl/imemory.v` and `rtl/memory.v` as unbuildable in four independent ways. This ADR builds the
memory system and takes the number.

**ADR-0044 scopes this work post-M4 and M2 is not closed.** Pulling it forward is deliberate. The
guard is that no M2 term regresses, and the runs below are that guard discharged, not asserted.

## Decision 1 — the ROM is in block RAM, and that bounds programs to 8 KB

`SB_SPRAM256KA` has **no INIT capability at all**, so a ROM there needs a runtime boot path copying
from external SPI flash. That is a different product and it is out of scope. `SB_RAM40_4K` is
initialisable from the bitstream, and the part has 30 of them.

`rtl/littlesoc.v` ships **2048 words = 8 KB**, which is 16 EBRs; `rtl/regfile.v` takes 4, leaving 10
spare. It is a parameter, and `test/testbench.v` overrides it to 16 KB because simulation has no
EBRs to run out of.

**The 8 KB ceiling is not "orders of magnitude" clear of the suite, which is worth knowing.**
Measured over all 56 programs: 55 of them are **≤ 1,176 bytes** of `.text`. `test/asm/rvc.S` is
**12,232**, almost all of it `.skip` padding for its page-boundary-straddle case. A 12 KB ROM would
hold it, at 24 EBRs — 28 of 30 with the regfile, and 203 extra LUT4s for the deeper read mux
(measured). The trade is recorded rather than taken: 8 KB with headroom, and one program named as
not fitting.

**What this decision does NOT solve, stated here rather than found on hardware: the data RAM cannot
be initialised either.** Every `.S` program has a `.data` section; a bitstream built from one would
run against zeroed SPRAM. Closing that means a `.text` copy stub (a crt0 copying an image linked
into ROM) or ADR-0044's flash boot path. Neither is built here. **The SoC as it stands places, meets
a timing figure, and would not run a program that reads its own `.data`.**

## Decision 2 — banking replaces duplication, at WORD granularity, not halfword

`rtl/littlesoc.v` used to instantiate `imemory` **twice** to get ADR-0003's second fetch port, which
doubles storage to add a port and was most of the 43 KB overage ADR-0044 measured. `rtl/imemory.v`
is two interleaved banks now: word W lives in one bank at index W/2 and word W+1 in the other, so
the pair is always one read from each and storage is the ROM size, once.

**ADR-0044 names this technique at 16-bit granularity** — even halfwords in one bank, odd in the
other. That is right for a memory that returns the 32-bit window itself. This core's fetch interface
asks for two adjacent **words** and does its own windowing (`rtl/fetcher.v`), so the parity that
matters is word parity; 16-bit banks would need two reads from each bank rather than one, which is
duplication again one level down. Word-granularity interleaving is the same technique against the
interface this core actually has — and it is also the one that leaves `imem_data`/`imem_data2`
meaning exactly what `formal/imemcheck.sv` checks them to mean.

## Decision 3 — the fetch address is published one cycle early, and invariant 1 survives exactly

ADR-0044's recommendation, adopted. There is no combinational-read memory on this part, and
invariant 1 requires decode to see the instruction at `pc` in the same cycle it decides the next one.
So the memory's address register is loaded with the **next** fetch address at every posedge, in
lockstep with the PC itself.

Mechanically: `rtl/decoder.v`'s scattered `pc <= ...` writes become one combinational priority chain
named `next_pc` — the same order Verilog's last-write-wins gave them — `rtl/fetcher.v` turns it into
`imem_addr_next` (`{next_pc[31:2], 2'b00}`, which is what `imem_addr` holds one edge later), and
`rtl/littlecpu.v` exports it.

**No extra cycle, no speculation, no stall reason, and NO FLUSH.** A stalled cycle sets
`next_pc = pc`, so the memory re-latches the same address and re-presents the same words. Nothing was
added to ADR-0009's stall protocol; invariant 8 still names five reasons over two mechanisms.

**A negedge read stays ruled out**, for ADR-0044's reason and not a weaker one: sby's `prep` fails
closed on mixed clock polarity, `clk2fflogic` produces false greens (ADR-0040 measured `reg_ch0`
passing on a design the stock ladder kills), and every `insn_*` check reads through `imem_data`.

### What holds the lockstep, since nothing else would notice it breaking

Breaking it has exactly one symptom — the SoC fetches the wrong instruction — with no elaboration
error, no lint finding, and **nothing on the riscv-formal ladder in contact with it**
(`formal/wrapper.v` answers `imem_data` combinationally against `imem_addr` and never reads the new
port). So it is asserted in three places, and each was probed:

| where | what it asserts | probe |
|---|---|---|
| `formal/pcloop.sv` | `imem_addr == $past(imem_addr_next)` on the composed fetcher+decoder loop | driving `imem_addr_next` from `pc` instead of `next_pc` fails at basecase step 1 |
| `rtl/decoder.v`'s `FORMAL` block | `pc == $past(next_pc)` — i.e. `pc` has no second driver | k-induction, `components_decoder` |
| `test/decoder_tb.v` | the same, on every edge the bench takes | a second `pc` driver gated on `instr_addi` is caught; one gated on `instr_wfi` is not, because the bench never issues a wfi on a non-stalled cycle |

## Decision 4 — the data RAM is shaped for SPRAM inference, and the shape is load-bearing

`rtl/memory.v` stays a plain behavioural array; `synth_ice40 -spram` does the 16-bit width split and
the nibble-mask mapping ADR-0044 flagged. No vendor primitive is instantiated, so both simulators run
the same description the synthesiser reads.

**The one non-obvious line: the read port is NO-CHANGE on a write cycle.** yosys's SPRAM rule
(`ice40/spram.txt`) declares `rdwr no_change`; the obvious read-first spelling maps the same array to
**148 `SB_RAM40_4K`** — five times the part's entire block RAM — and yosys reports that as a normal
run, failing later in nextpnr with a message about BELs. Measured both ways. Nothing in the pipeline
observes the difference: `rtl/accessor.v` reads `mem_rdata` only on a load's response cycle
(ADR-0015), when decode is bubbled and no store is in flight. `test/mem_tb.v` pins it.

**The census is checked against a declaration, not grepped for by name.** `make soc-timing` requires
exactly 2 `SB_SPRAM256KA` and 20 `SB_RAM40_4K`. The first version of that check was
`grep -q SB_SPRAM256KA soc.synth.log` and **it passed on the mutated build** — yosys logs the cell's
library definition whether or not it instantiates one.

## Decision 5 — the memories are on the simulator's dependency graph

`rtl/littlesoc.v`, `rtl/memory.v` and `rtl/imemory.v` were in neither sim leg's build. They are now:
`test/testbench.v` instantiates the real modules instead of carrying its own inline arrays, so the
**56-program suite and the Sail co-simulation run against the memory system `rtl/littlesoc.v`
synthesises**.

That is not bookkeeping. `rtl/imemory.v` is synchronous and addressed a cycle early; a broken
lockstep would have had every program executing the wrong instructions, and before this change
nothing in the tree would have noticed. The runners de-interleave the `--rom` image across the two
banks (`load_rom_banks` in `test/cxxrtl.cc` and `test/cosim.cc`) and read RAM through the instance.
`test/imem_tb.v` joins `make test-units` — seven benches now — and walks every word index at both
parities against a **flat** reference, plus the range decode; the reference is deliberately not
derived from the bank split, because a reference derived from the thing under test agrees with any
split at all.

## Decision 6 — the SoC has four pins, and that is the honest answer

`clk`, `btn_n`, `ledr_n`, `ledg_n` (`soc/littlesoc.pcf`, iCEBreaker assignments). Both memories are
internal, so the design has **no external bus at all** — which is exactly why it places where
`make fit`'s top cannot: 4 `SB_IO` against sg48's 39, versus 231 (ADR-0038 decision 1a).

The two LEDs are passive taps — one bit of the last store's data, and a sticky trap flag. They exist
because **a design with no observable output is one yosys is entitled to delete**, which is what
happened to the previous version of this module: it reported 4 logic cells and 0% utilisation while
doing it. Neither tap adds an MMIO region, so ADR-0044's `causal_io_ch0` / `bus_dmem_io_*` questions
are exactly where they were.

## The measurement

`make soc-timing`, deliberately named so it cannot be confused with `make fit`. **Homebrew yosys
0.67+post, nextpnr-ice40, icetime; machine load ~3 on 10 cores; ~33 s wall from clean.**

```
ICESTORM_LC:    4041 / 5280   76%
ICESTORM_RAM:     20 /   30   66%     16 ROM + 4 regfile
ICESTORM_SPRAM:    2 /    4   50%
SB_IO:             4 /   39   10%
SB_GB:             8 /    8  100%
ICESTORM_DSP:      4 /    8   50%
```

```
critical path : 88.51 ns  (11.30 MHz)     icetime
  logic       :  34.20 ns  38.7%
  routing     :  54.29 ns  61.3%
  logic levels: 41
  start       : imem.in_range
  end         : imem.in_range2
nextpnr's own analysis: 11.70 MHz
```

**The path, named.** `imem.in_range` (the ROM's range flag, which masks `imem_data`) → the fetch
window → decode → the `next_pc` adder's carry chain → `imem.in_range2` (the range comparator on the
new address). That is exactly the loop invariant 1 puts in one cycle — fetch, decode, next PC, fetch
address — and it is now measured rather than argued.

One caution about reading the report: the intermediate net names are yosys's, so a net named
`riscv.csrs.mcause_..._D` on this path is **shared decode logic that also feeds `mcause`**, not a
detour through the CSR file.

### Two findings came out of the first run, and both are fixed here

1. **The critical path started at the `btn_n` PAD.** 1.105 ns of pad delay plus four routing hops,
   on the longest path in the design, because `reset` was combinational off an asynchronous input.
   It is a two-flop synchroniser now — which it needed to be for metastability anyway.
2. **It ended in a 32-bit incrementer feeding the ROM's range check.** `imem_addr_next + 4 <
   ROM_BYTES` put a second carry chain in series with the one that had just produced the address.
   Both range tests compare the word index against a constant now (`< ROM_WORDS` and
   `< ROM_WORDS - 1`), which is the same predicate with no adder — and it removes a corner besides:
   the old form wrapped at the top of the address space and answered the second word of a fetch at
   `0xfffffffc` out of real ROM.

**9.60 → 11.30 MHz for those two.** The whole value of taking a measurement is in that line.

### The churn axis is measured, and it is bigger than `make fit`'s

**Two logically identical spellings of `rtl/memory.v`'s write/read arms give 88.51 ns and 91.67 ns —
41 and 53 logic levels — on 11 logic cells' difference in the netlist.** `rtl/memory.v` is the data
RAM; **it is not on the critical path at all**. 11 cells anywhere is enough for nextpnr to
redistribute placement, and the fetch loop's routing moves with it. Both figures are reproducible run
to run (nextpnr is seeded); it is the **edit** the number is unstable under, exactly as ADR-0038
found for logic cells — but at **3.6%** against that axis's ~1.2%.

Two consequences, and they are the same two ADR-0038 drew:

- **A `make soc-timing` delta smaller than a few percent is not evidence of anything.** The 9.60 →
  11.30 MHz improvement earlier in this ADR is safely outside that band; a 2% one would not be.
- **`SOC_MIN_MHZ` needs headroom wider than the band**, or it goes red on changes that synthesise to
  the same hardware. It is 10.0 MHz — 11.5% under the measurement, roughly four times the band and
  the same ratio `FIT_MAX_LC` keeps.

The discovery has a cost recorded with it. The nested spelling of `rtl/memory.v`'s arms says the
no-change property more directly and was written first; it ships in the **flat** form, and the file
says why at the site. ADR-0038 rejected the shifter merge at 19 cells *saved* on legibility grounds
and `CLAUDE.md` declines a hygiene change costing 37 — spending 11 cells and 3.6% of the only timing
number this project has, on a design already 6% short of its declared 12 MHz, cuts the same way.

### What the number does not describe

- It is `icetime`'s **static topological estimate** for **one placement** of **one build**, at the
  default worst-case corner. It is toolchain-dependent exactly as `make fit` is (ADR-0052 measured
  21 cells between two yosys builds on identical RTL) and edit-dependent as measured just above;
  quote it with the toolchain.
- It says nothing about whether the design *works* on hardware. The RAM cannot be initialised
  (decision 1), so it would not run any program in `test/asm` as linked.
- `SB_GB` is at **8/8**. A change that needs a ninth global buffer is a placement failure no
  logic-cell or frequency ratchet would predict.

### The design does not meet ADR-0038's declared 12 MHz, and that stands

11.30 MHz by `icetime`, 11.70 by nextpnr, against a declared 12. **ADR-0038 declares 12 MHz as an
intent, and this ADR does not move it in either direction** — reconciling a measurement with an
intent is a decision, not a consequence, and closing a 6% gap means shortening the loop invariant 1
puts in one cycle, which is ADR-0038 decision 2's own test for "you are proposing to break invariant
1 or invariant 6."

`make soc-timing` therefore ratchets against **`SOC_MIN_MHZ = 10.0`** (this line said 10.5 and the
Makefile never did; corrected in [ADR-0056](0056-what-writable-text-costs-in-ladder-depth-and-in-nanoseconds.md),
which measured a build at 9.98 MHz and watched the gate exit 2), a regression floor set *below*
the measurement rather than at the intent. A gate pinned at 12 would be red on arrival, which is a
gate nobody keeps; one pinned at 11.30 would be red on any resynthesis. Both directions are probed:
`make soc-timing SOC_MIN_MHZ=99` exits nonzero naming the loop, and a nextpnr that produces no
bitstream or no utilisation table fails rather than reporting a fast empty design.

**nextpnr's exit status is deliberately not the signal.** It defaults to a 12 MHz target on ice40 and
exits nonzero when a design misses it — after writing the `.asc`. Throwing that away would mean no
`icetime` report at all, i.e. no measurement, *because* the design is 6% slow.

## The core got smaller, and that number is separate

`make fit`'s top is untouched (ADR-0038 decision 1): `littlecpu` with memories external, no
`littlesoc` in `FIT_SRCS`. **The number moved anyway, because the decoder changed: 4187 → 3875 logic
cells, both measured on this branch under the same local Homebrew yosys.** 312 cells, six times the
±50 churn floor, and measured as a delta rather than quoted across toolchains — which is the trap
ADR-0052 records. Attributable: six independent `cond ? pc+imm : pc+inc` writes to `pc` became one
`branch_taken` and one priority mux, and `out.rvfi.pc_wdata` stopped being a second hand-maintained
copy of the whole redirect chain. `FIT_MAX_LC` drops 4400 → 4100.

**The two area numbers must not be merged.** 3875 is the core; 4041 is the SoC, and the difference is
the ROM's depth mux, the RAM's range decode and the LED taps.

## The M2 terms did not regress — run, not asserted

| gate | result |
|---|---|
| `make -C formal check` | **85 checks, 85 pass**, both set equalities in both directions |
| `components_decoder` / `_executor` / `_pcloop` | PASS by k-induction (pcloop carries the new lockstep assertion) |
| `imemcheck` / `dmemcheck` / `cover` | PASS / PASS / PASS, all five cover goals |
| `complete` / `complete_cover` | PASS / PASS |
| `nonperturbation` | PASS |
| `make test` | **56/56**, `test/EXPECTED_FAIL` empty |
| `make test-units` | 7 benches green |
| `make cosim-suite` | **56/56**, `test/COSIM_EXPECTED_FAIL` empty |
| `make fit` | 3875 / 4100 — see above |
| `make lint` | clean in both passes |

**The ladder needed no weakening, and that was the criterion most likely to bite.** `imem_addr` is
still `{pc[31:2], 2'b00}` combinational off the registered PC and `imem_data` is still free every
cycle, so `formal/wrapper.v` and `formal/imemcheck.sv` describe the same bus they always did. The
new port is one extra output the ladder does not read.

**Which is also the honest limitation**: the ladder is in contact with the core's *interface*, not
with the memory system behind it. Nothing in `formal/` checks that `imem_data` is what the ROM
holds — `formal/imemcheck.sv` says so itself, in the standing `DISCHARGED NOWHERE` note it carries
for exactly this fact. What checks the memory system is `test/imem_tb.v`, `test/mem_tb.v`, and the
56-program suite now running through it.

## Consequences

- **`make soc-timing` is a hand-run target and is not on CI**, because it needs the RISC-V cross
  compiler to build a real ROM image and CI's required set must not grow a toolchain dependency for
  a number that is a design constraint rather than a correctness one — the same reasoning ADR-0052
  used to keep `fit` non-required.
- **`soc/` is new**: the pinout, the ROM bank splitter, and the `icetime` report summariser. The
  splitter refuses to truncate a program that does not fit; a breached ROM ceiling is a finding about
  the part.
- **ADR-0044's eleven declined ladder checks are unchanged, and this ADR owes it a ruling on each.**
  Nothing here adds a faulting bus or a distinguished IO region, so all eleven `#omit` lines stay
  `[BLOCKED]` exactly as they are: this memory system answers every access in range and drops
  everything else silently, which is ADR-0044's **option 1** in shape but not yet in commitment.
  **Invariant 2 is untouched and unpressured** — there is no refusal to raise a fault from. Making
  option 1 a permanent ruling (re-tagging those five `#omit` lines `[DESIGN]`) is a decision this ADR
  deliberately does not take, because the SoC does not yet have the address map a real product would
  and the ruling should be made against one.
- **The `make waves` program moved into an `ifdef ICARUS` block** writing the ROM banks directly. A
  hierarchical reference, which yosys does not resolve — it implicitly declares the dotted name and
  carries on — hence the guard.
- ADR-0044's line that "no timing number exists or can exist until this lands" is discharged.
  ADR-0038 decision 2's "record `icetime` output once the design places" is discharged, in
  `make soc-timing` rather than in `make fit`, because they measure different designs.

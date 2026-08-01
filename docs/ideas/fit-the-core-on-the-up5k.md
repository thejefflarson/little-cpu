# Fit the core on the up5k without spending readability

**Status:** Design brief. Measured at `ed2bab5`; RTL read at `6309b3e`. The numbers below are
**pre-trap-entry** — step 0 of the plan is to re-measure, and every estimate here is falsifiable
within one commit.

## The problem

`CLAUDE.md`'s opening says the eventual home is an ice40 up5k. That claim is currently false:

```
ICESTORM_LC:    6659 / 5280   126%   ← FAILS TO PLACE
ICESTORM_RAM:      0 /   30     0%
ICESTORM_DSP:      4 /    8    50%
ICESTORM_SPRAM:    0 /    4     0%

ERROR: Unable to place cell 'regfile.regs[6]_SB_DFFE_Q_21_DFFLC', no BELs remaining
```

Every feature since `rtl/csrs.v` has made it more false — the CSR file cost +639 LUT4 on its own,
and trap entry (`6309b3e`) landed after the measurement above.

One framing rule dominates everything that follows: **an area plan that wins cells by making the
core harder to read has failed on this project's own terms.** Every lever below is ranked on
readability first, and two were killed for costing it.

## Only nextpnr numbers are real

```
4855 LCs used as LUT4 only
 524 LCs used as LUT4 and DFF     ← only these share a cell
1228 LCs used as DFF only         ← ~regfile (32×32=1024) + CSR state
  37 LCs used as CARRY only
```

Yosys reports 5379 `SB_LUT4`; nextpnr reports 6659 LCs. The difference is flip-flops that failed to
pair with a LUT — a DFF whose D input is not the output of a co-located LUT consumes a whole logic
cell, and a LUT4 count cannot see that.

**This error has now been made twice, in both directions.** An estimate built on LUT4 counts said
102% when the truth was 126%. A later estimate said the regfile was worth ~1024 cells when the truth
is more than twice that. Any number in this document not traceable to a nextpnr report is marked as
an estimate.

## Where the cells actually are

The regfile and executor account for roughly 2000 LUT4-only cells. The rest, from reading the RTL
against standalone-synthesis proportions — **estimates, medium confidence:**

| Block | Est. in-core LUT4 | What it is |
|---|---|---|
| Regfile read path | 1200–1400 | two 32:1 × 32-bit mux trees (~11 LUT4/bit/port), bypass compares, x0 gating |
| Executor | 2500–2900 | divider 64-bit datapath 700–900; three barrel shifters 500–600; ~15-way result mux 400–600; comparators and operand plumbing 300–400 |
| Decoder | 550–750 | 18-arm × 32-bit immediate mux, six branch-compare arms, `mem_addr_calc`, jalr target, ~60 one-hot terms |
| CSR file | 450–550 | 17-way × 32-bit read mux, WARL mux, two 64-bit increment chains |
| Accessor | ~200 | load align/sign-extend, store byte replication |
| Fetcher | ~60 | the 16-bit window shift |
| Writeback | ~20 | |

That closes against the measured 4855 + 524.

## Approaches considered

**A — Verification-first: hygiene only, all structural work after M2.** Rejected. It treats
"verified" as a binary this repo does not itself believe in (ADR-0023 and ADR-0033 are careful
expositions of what green does and does not mean), it leaves the target claim false for no gained
assurance, and the two levers that matter are covered by oracles that exist *today*. M2's residue —
ALTOPS, BMC bounds, non-converging `equiv.sh` — is partly structural to the pinned harness and may
never fully close.

**B — Two levers, sequenced by oracle strength.** **Recommended.** Detailed below.

**C — Broad restructure: B plus adder sharing, divider narrowing, result-mux refactor, and the SoC
memory system.** Rejected. B alone lands at ~74–84% with headroom; every additional lever in C buys
margin nobody needs at a price the charter forbids, and it rewrites the divider — the one datapath
with no formal oracle in any form — mid-M3.

**Considered and rejected as a variant:** retarget to hx8k, where 6659 LCs fits at 87% today. It
abandons the stated product home and forfeits the SPRAM story the memory parameters were literally
sized for (15872 words is half the up5k's 128 KB), to avoid writing one ADR.

## Recommended: two levers

### Lever 1 — merge the three barrel shifters

`rtl/executor.v:129,133,134` spell SLL/SRL/SRA as three separate operators. One right-shifter plus
input/output bit reversal for left shifts (reversal is wiring, free in LUTs), SRA differing only in
the fill bit.

- **Recovery:** ~300–450 LCs (medium confidence)
- **Readability:** *improves* it. Three operators whose sharing is invisible become one named
  construction with the cost made legible — which is this project's idiom.
- **Invariant traded:** none.
- **Oracle:** `RISCV_FORMAL_ALTOPS` replaces only the **M-extension** ops
  (`formal/checks.cfg:288`). The shift checks — `insn_sll/srl/sra/slli/srli/srai` plus three
  compressed forms — run at **real 32-bit width and pass**. This is one of the best-covered changes
  available in this repo, not one of the riskiest. Plus six shift `.S` tests, Sail co-sim, and new
  randomized shift vectors in `test/exec_tb.v` (which currently has none).

### Lever 2 — the regfile moves to EBR

Posedge write, negedge read, bypass stays in fabric.

- **Recovery:** ~2100–2400 LCs (medium-high confidence: 1024 DFF measured, read-mux estimated).
  Taking ~128% to **~84–89% by itself** — close to sufficient alone, not merely necessary.
- **Readability:** neutral if the ADR states the contract clearly.
- **Invariant traded:** 6, re-worded to its observable content — see below.
- **Also a timing fix:** the measured read path is 2.04 ns logic + **12.78 ns routing** (86%
  routing) on hx8k, which is *faster* silicon than up5k. A distributed 32:1 mux is a routing
  problem, which is exactly why EBR wins.
- **Oracle:** `test/regfile_tb.v` (asserts the contract, not the implementation), `reg_ch0` under
  btormc, and Sail co-simulation reading the **real** `regs` array. ADR-0032's mutation experiment
  is the argument: an injected extra architectural write was missed by the `.S` suite, the
  per-retire monitor, *and* the full ladder when gated past the BMC bound — and caught by the
  co-sim. For this change the co-sim is not a nicety, it is the oracle.

### What invariant 6 actually means

Its load-bearing content, which ADR-0004's stall-only scoreboard depends on, is: **decode observes,
in cycle N, the architectural value of rs1/rs2 including a cycle-N writeback.** The FF array plus
combinational mux is one implementation. A negedge-strobed EBR read (address settles in the first
half-cycle, data returns in the second) plus the existing bypass mux is another, with identical
observable semantics.

One implementation, no `ifdef` fork — two regfiles is drift by construction. On a Tiny Tapeout/ASIC
flow the same RTL synthesizes back to a DFF array plus a negedge mux: it works there, it just isn't
smaller there, which is a different problem for a different ADR.

## The gate: not M2, something concrete

**Do not block this on M2.** M2's residue does not cover the regfile any better next quarter than
today; the instruments that do cover it all exist now. The one genuinely missing control is
suite-wide Sail co-simulation, which ADR-0032 already scoped as future work.

The regfile change is gated on:

1. Sail co-sim wired across the full `.S` suite and green at the pre-change SHA
2. `reg_ch0` PASS immediately before and after
3. Full ladder + `formal/EXPECTED_CHECKS` set equality unchanged
4. `make test`, `test-units`, lint green in both sim legs

That converts "wait for verification" from a vibe into a checklist item.

## Killed, with reasons — recorded so they stop resurfacing

- **Executor adder sharing.** Cross-stage sharing is impossible (decode and execute work on
  different instructions in the same cycle). Within the executor it saves ~60–120 LCs — 32-bit carry
  chains are cheap on ice40 and ABC already shares some — for a first-order cost to the
  one-op-per-line `case (1'b1)` legibility that CLAUDE.md protects. ~2% of the gap at the highest
  price on the list.
- **Radix-4 divider.** The deferred list already records that it "roughly doubles comparator and mux
  logic." It is a CPI lever that *costs* area. Wrong list.
- **"Delete `littlesoc.v`."** It is the synthesis top and instantiates the memories; it cannot be
  deleted. What *is* dead: the flash-pin scaffolding, the free-running `flash_clk` divider, the
  declared-but-unused `mem_valid`/`mem_ready` wires, the `// TODO SPI mem`. Worth ~2 LCs and a clean
  svlint run — hygiene, not an area lever.

**Deferred with a trigger:** divider datapath narrowing, 64-bit → the textbook 33-bit
shift-remainder form (~150–250 LCs, readability neutral). Only if post-regfile margin falls below
15%, and only after M2 — the divider's only oracles are `exec_tb`, the `.S` suite and Sail.

## Fmax: declare 12 MHz and stop

Invariant 1 (combinational fetch) and invariant 6 put `pc → imem_addr → imem_data → window mux →
decode → regfile read → branch compare → next pc` in one cycle **by design**. A low Fmax is a
consequence of a stated value, not a defect.

The regfile read alone measured 14.82 ns on faster silicon; the full loop plausibly lands at
50–70 ns on up5k, ~15–20 MHz. Declare **12 MHz** (the iCEBreaker oscillator) and record `icetime`
in `make fit` once the design places. At 12 MHz a half-period is 41 ns, which is enormous relative
to the negedge discipline's budget. Anyone proposing to raise Fmax later is proposing to break
invariant 1 or 6, and must bring an ADR.

## Risks

- **Formal runtime is the load-bearing unknown, not correctness.** A negedge read register forces
  `clk2fflogic`, roughly doubling modeled state per cycle. `reg_ch0` — the exact check tying RVFI to
  the real regfile — was inconclusive for months under yices and passes in ~8–12 s under btormc
  (ADR-0024). It is the check most likely to regress to inconclusive. **Settled by a half-day spike
  branch running the full ladder against the negedge regfile and comparing wall times, before any
  RTL is declared landed.** Fallback if it degrades: prove regfile equivalence standalone (EBR model
  vs FF model, a small self-contained `sby` task) and keep the ladder's wrapper on a posedge model —
  but decide that with the spike's data, not in advance.
- **Yosys EBR inference** of a posedge-write/negedge-read memory may need the `SB_RAM40_4KNR`
  primitive family or an explicit attribute rather than falling out of `memory_bram`. Same spike
  settles it — inspect the netlist for `SB_RAM40_4K*` and the nextpnr RAM count.
- **Write-to-read visibility across a half cycle.** The posedge-N write must be readable at
  negedge N. Comfortable at 12 MHz, but it is an implementation assumption the FF array did not
  have: assert it in `regfile_tb` with a directed back-to-back case, and treat it as the one
  timing-closure obligation `icetime` must confirm.
- **`test/cosim.cc` reads the real `regs` array via cxxrtl `debug_items`.** An EBR-inferred memory
  still surfaces, but the item name or shape may shift. Since the co-sim is the gate, **verify the
  probe still reads the true array before trusting a green run** — a co-sim silently comparing
  nothing is exactly `docs/THREAT_MODEL.md`'s category-1 failure.

## Sequence

0. **Measure.** `make fit` — synth `littlecpu` (memories external), nextpnr sg48, print the
   LC/RAM/DSP table and `icetime` once placeable. Record the post-trap-entry number.
1. **Hygiene.** `littlesoc.v` dead scaffolding; `mul_div_counter` `[6:0]`→`[5:0]`. Re-measure.
2. **Shifter merge** + shift vectors in `exec_tb`. Full ladder, suite, Sail. Re-measure (~−300–450).
3. **Sail co-sim suite-wide.** Test infrastructure, no RTL. This is the regfile's gate.
4. **Spike** the negedge regfile on a branch: ladder wall times, EBR inference, cosim probe.
   Go/no-go with data.
5. **ADR + negedge-EBR regfile**, extended `regfile_tb`, gates above. Re-measure
   (~−2100–2400 → roughly **3900–4400 LCs, 74–84%, EBR 4/30, DSP 4/8**).
6. **Declare fit.** Utilization and `icetime` in `make fit`'s output; CLAUDE.md's up5k sentence
   becomes true.

## Out of scope

The SoC memory system — SPRAM data RAM, ROM banking or a next-pc-clocked BRAM, byte-strobe handling
on SPRAM nibble masks. Post-M4, its own ADR, reusing the read-edge convention this brief settles.

Noted for that future ADR so it does not start false: **ROM and regfile cannot both do a negedge
read in the same cycle**, because the regfile's read address depends on the ROM's read data. The ROM
will want the other standard trick — clocking the BRAM address with the combinational *next* PC at
posedge.

# Fit the core on the up5k without spending readability

**Status:** Design brief. Originally measured at `ed2bab5` (pre-trap-entry). **The estimates below
were subsequently checked by building and measuring each variant — see "Measured" immediately below,
which supersedes them where they disagree.** ADR-0038 carries the same table.

> **Two of this brief's formal claims were measured again and are FALSE.** Both concerned
> `reg_ch0`, both are corrected in place below, and both had the same cause: the stale-run-directory
> defect in `formal/Makefile` that ADR-0040 found and fixed. Before that fix, re-running a check in
> an existing `checks/<name>/` aborted inside sby and left the *previous* run's `status` file for
> `check-baseline.sh` to re-grade — so a mutated or re-synthesised design could be reported PASS
> having never been evaluated. Any other formal result in this brief taken from a repeated run in
> one tree is suspect for the same reason and should be re-measured before it is relied on.
> **[ADR-0040](../adr/0040-the-ladder-refuses-a-negedge-regfile-and-make-check-was-re-grading.md)
> supersedes this brief on everything to do with the ladder and a negedge regfile.**

## Measured — this supersedes the estimates in the body

All `nextpnr-ice40 --up5k --package sg48`, `littlecpu`, memories external, post-trap-entry:

| Configuration | LC | % | EBR |
|---|---|---|---|
| Baseline | **6971** | **132%** | 0/30 |
| + merged shifter | 6835 | 129% | 0/30 |
| **+ negedge-EBR regfile** | ~~**4017**~~ ~4100 | ~78% | **4/30** |
| + both | ~~3998~~ ~4100 | ~78% | 4/30 |

The negedge variant was built and run, not projected: **52/52 `.S` tests pass under cxxrtl with the
per-retire monitor live**, ~~`reg_ch0` passes~~, and yosys infers **`4 × SB_RAM40_4KNR`** from a
plain two-array negedge-read model — no attribute needed. **EBR inference is a non-risk; delete it
from the spike's scope.**

> **Two corrections to the four lines above, ratified at integration (ADR-0042's wave).**
>
> **1. The primitive is `SB_RAM40_4KNR`, not `SB_RAM40_4K`.** This paragraph asserted the opposite,
> by name. Re-measured directly — `synth_ice40` on a two-array negedge-read model reports
> `4 × SB_RAM40_4KNR`, the negative-edge-read-clock variant, which is exactly what a negedge read
> should infer. The *conclusion* the sentence was written to support is unchanged and if anything
> stronger: inference works, needs no attribute, and is not a risk. But the cell name was wrong, and
> it is the kind of wrong that sends the next reader looking for a bug that is not there.
>
> **2. Do not quote either figure to four digits.** ADR-0042's wave re-measured the same lever on the
> same baseline and got **4137**, against the **4017** here — 120 cells, 2.3% of the part. The
> baseline (6971) reproduces exactly, so this is not a different tree; the earlier figure came from a
> scratch variant that is not in the repo and cannot be diffed, so the two are not comparable at that
> resolution. **Neither number is trustworthy in its last two digits.** The conclusion is unaffected:
> the regfile alone closes the area problem, and all three ways of doing it land within 86 cells of
> each other. See also this repo's own ±~50-cell churn floor — a `make fit` delta smaller than that
> is not evidence of anything.

**`reg_ch0` cannot pass on a negedge regfile, and did not** (ADR-0040 finding 1). sby runs its own
`prep` model step after the `.sby`'s `[script]`, and `formalff -clk2ff` there fails closed on mixed
clock polarity in the full `rvfi_testbench`: *"CLK clock ... also used with opposite polarity, run
clk2fflogic instead"*, `DONE (ERROR, rc=16)`, in about a second. `check-baseline.sh` counts that as
red. The PASS recorded here was the previous run's status file being re-graded, not a verdict about
the negedge design. The area and `.S`-suite numbers in this table were measured by other means and
stand.

Four corrections to the body:

1. **The regfile is sufficient alone** (132% → 76%), not "close to sufficient". Every other lever is
   optional.
2. **The shifter merge is not an area lever** — 136 LCs standalone, **19** on top of the regfile,
   against the body's 300–450. It survives on readability or not at all.
3. **The named risk below is the wrong one.** See "The real formal risk".
4. `make fit` **can never place** (231 `SB_IO` vs 39), so "declare fit" cannot mean what step 6 says.
   ADR-0038 decision 1a resolves this.

## Measured — three ways to reconcile invariant 6 with a synchronous BRAM read

> **Decided: option B.** [ADR-0042](../adr/0042-the-regfile-read-is-synchronous-and-costs-a-cycle.md)
> is the decision and supersedes this section's "contingent on" wording. The `imem_data` modelling
> change was accepted, `hang` and `liveness_ch0` went green with it, and the shipping numbers are
> **4236 logic cells / 80%** at **+18.0%** suite cycles — better than the +27.8% below, because the
> stall was gated on `uses_rs1`/`uses_rs2` after this was written.
>
> **The cell count is a measurement of ADR-0042's tree, taken locally, and not of `main`.** It is
> **4208 / 79%** as of ADR-0052, measured by the `fit` CI job at the pinned OSS CAD Suite — which is
> also where it emerged that the number depends on the yosys build: the same commit gives **4187**
> under a local Homebrew yosys. Take the current number from the `fit` job, not from a document and
> not from a local run, and say which toolchain produced it.


The root constraint is that an **ice40 EBR read is synchronous** while **invariant 6 requires a
combinational read**. This brief only ever costed one way of reconciling those (a negedge strobe).
All three were built and measured against the same tree, so the choice is now a comparison rather
than an assumption. **No regfile RTL is landed by this record**; the variants were scratch
instruments, and none is in the tree.

- **A — negedge read.** Posedge write, negedge array read, bypass in fabric, two arrays.
- **B — synchronous read, pay a cycle.** Posedge write-first read, and decode holds the address for
  a cycle and bubbles: a fifth stall *reason* on the existing bubble mechanism, no flush.
- **C — serialise the two read ports.** One array, posedge, rs1 then rs2, two bubble cycles.

All figures `nextpnr-ice40 --up5k --package sg48`, `littlecpu`, memories external; `.S` suite under
cxxrtl with the per-retire monitor live; ladder is `make -C formal check` at 82 checks.

| | baseline | **A** negedge | **B** sync + 1 stall | **C** serialised |
|---|---|---|---|---|
| Logic cells | 6971 (132%) | **4137 (78%)** | 4223 (79%) | 4175 (79%) |
| EBR | 0/30 | 4/30 | 4/30 | 2/30 |
| DSP | 4/8 | 4/8 | 4/8 | 4/8 |
| `.S` suite | 52/52 | 52/52 | 52/52 | **44/52** |
| Suite cycles (52 programs) | 22512 | **22512 (+0.0%)** | 28760 (**+27.8%**) | n/a (broken) |
| Formal ladder | 82 pass | **cannot run** | 80 pass, **2 red** | 3 red |

A's 4137 does not reproduce ADR-0038's 4017 for the same lever on the same baseline (6971 does
reproduce exactly). 120 cells, 2.3% of the part — the earlier figure came from a scratch variant
that is not in the tree and cannot be diffed, so the two are not comparable at that resolution and
neither should be quoted to four digits. **The conclusion is unaffected**: the regfile is sufficient
on its own, and the three ways of doing it are within 86 cells of each other.

Four things this settles.

**1. The area difference between the three is noise.** 4137 / 4223 / 4175 logic cells — a spread of
86 cells, 1.6% of the part. **Every option clears 132% → ~79%, and area does not choose between
them.** In particular C's one-array saving is 2 EBRs out of 30, in a resource that is 87% idle,
bought for +48 logic cells over A and a materially harder control problem.

**2. The ladder refuses A outright, and the refusal is total.** Reproduced exactly as ADR-0040
finding 1 describes, on the same cell:

```
prep: ERROR: CLK clock on $flatten\wrapper.\dut.\regfile.$procdff$2308 ($dff) from module
             rvfi_testbench also used with opposite polarity, run clk2fflogic instead.
DONE (ERROR, rc=16)      # reg_ch0, 1.5s
```

That is not one red check — it is every check on the ladder, because the model step fails before
any of them. Running the ladder against A at all requires substituting a posedge model at the
`formal/wrapper.v` seam ADR-0040 decision 2 specifies, plus an equivalence proof to carry the
ladder's results back to the shipping RTL. That proof was attempted here: a two-model miter under
`multiclock on` with the read-timing assumption stated per ADR-0017. Its **base case passes at depth
20; k-induction does not close**, because the inductive invariant is equality of the two storage
arrays and yosys's Verilog frontend rejects the hierarchical references needed to state it
(`ERROR: Identifier '\gold.regs' is implicitly declared`). So A's ladder story is a bounded proof
carrying a bounded ladder — one more indirection on the one check (`reg_ch0`) that ties RVFI back to
the real register file.

**3. B and C are modelled normally, and the ladder immediately caught something.** B's `reg_ch0`
PASSes in 10s, and the non-vacuity probe works exactly as ADR-0040 specifies it — deleting the rs2
write-through bypass gives `bad state property 1 reachable at bound k = 20 SATISFIABLE` in 6.5s. So
the ladder is a live oracle for B in a way it can never be for A. And on the full run it went red:

```
82 checks: 80 pass, 2 fail
> hang
> liveness_ch0        # both: bad state property 0 reachable at bound k = 30 SATISFIABLE
```

Both are counterexamples, not timeouts. **The cause is structural and is the most useful thing this
comparison found.** A synchronous read means decode must decide, in cycle N+1, whether the operand
fetched at the posedge belongs to the instruction it is now issuing — and the only signal available
for that is the read address, which is combinational out of `imem_data`. `formal/wrapper.v` leaves
`imem_data` free every cycle (invariant 1: nothing in the core waits on its value), so the
environment may change the instruction word under a held PC forever and the core never issues. The
`ifdef RISCV_FAIRNESS` block in `formal/wrapper.v` is deliberately empty on the stated grounds that
"littlecpu has no port an adversarial environment could hold to defeat forward progress."
**Any synchronous-read regfile makes `imem_data` exactly such a port**, and that comment stops being
true.

The remedy is not a fairness fudge: it is that the harness should model instruction memory as a
*function of the address* rather than as a free value, which is what memory is. That is a real
change to `formal/wrapper.v`, and it should be costed before B is chosen — but it makes the harness
more faithful rather than less, and it is reviewable in a way A's substitution-plus-equivalence-proof
is not.

**4. C's control cost is real, not imagined.** Two independent attempts; the second is functionally
wrong in a way the per-retire monitor catches (8 programs, `MONITOR-ERROR 131`/`132`), and `reg_ch0`,
`hang` and `liveness_ch0` are all red. The reason is visible in the RTL: with one array, rs1's word
is read a cycle before it is used, so it has to be *parked* — and any write landing while it is
parked must be forwarded onto the parked copy. That is a **second bypass level**, which is the first
step toward the forwarding network invariant 4 forbids, and it is bought for two idle EBRs.

### What each one costs a reader

- **A** reads best in the regfile and worst everywhere else. `rtl/regfile.v` grows one `always_ff`
  and stays legible; `rtl/decoder.v` is untouched; CPI is unchanged. The cost is entirely non-local:
  a new rule that `reg_rs1`/`reg_rs2` are valid only in the second half of the cycle, which **no
  tool in this repo checks** — plus a formal harness that must be told to prove a different circuit
  than the one that ships.
- **B** reads best overall, and the sentence it adds is the plain one: *a register read takes a
  cycle.* It lands entirely inside ADR-0004's stall-only interlock and ADR-0009's stall protocol —
  one extra term in `stall`, one extra term in the existing bubble arm, **no flush** (invariant 1
  holds by construction, because the PC simply holds). It costs 27.8% of the suite's cycles on a
  core whose charter says CPI is deliberately sacrificed for readability, and it re-times decode's
  external contract, so `test/decoder_tb.v` needs rewriting (5 mismatches, all timing).
- **C** reads worst. It is the only one that adds a state machine, a parked operand and a second
  bypass level, for the smallest benefit.

### Recommendation

**C is out.** It costs the most complexity for a saving in the one resource the design has to spare.

**Prefer B, contingent on the harness change in finding 3.** It is the only option under which the
formal ladder is an oracle for the regfile that actually ships, and its non-vacuity is demonstrated
rather than argued. Its new obligation — the instruction word is stable while the PC is held — is
one the ladder *checks*, which is why it went red; A's new obligation is one nothing checks, which
is why A looks free. On a core that is not yet verified, an enforceable rule that costs cycles is a
better trade than an unenforceable one that costs nothing, and the project's own charter already
says CPI is what it is willing to spend.

**A stays viable and is the fallback** if the `imem_data` modelling change turns out to be
unacceptable: it is the only option with zero CPI cost, and it is the cheapest in cells by 86.

### Also measured

`formal/equiv.sh` is **not** exposed to ADR-0040's polarity loss, and that is worth recording
because the ADR's standing warning names it. The isolated reproduction still holds — `prep -flatten
-nordff` then `write_btor` on a negedge regfile emits `2 input 1 clk` and never uses node 2 again,
with **zero warnings**. But `equiv.sh` has no backend: its `prep -flatten -top littlecpu` retains
both negedge `$dff` cells (`CLK_POLARITY 1'0`) alongside 32 posedge ones, and `equiv_make` /
`equiv_simple` / `equiv_induct` reason on those cells directly. Under A it neither warns nor
mis-models; it also did not return within 10 minutes, which is the non-convergence ADR-0020 already
records. Under B and C the RTL is all-posedge and nothing about `equiv.sh` changes at all.

## The real formal risk — soundness, not runtime

> **Superseded by [ADR-0040](../adr/0040-the-ladder-refuses-a-negedge-regfile-and-make-check-was-re-grading.md).**
> The polarity loss described below is real *in isolation*, and the warning it draws for bespoke
> yosys scripts — `formal/equiv.sh` is one — is correct and worth keeping. But the conclusion is
> not: **the generated ladder never takes that path**, because sby inserts a `prep` model step that
> a hand-written script does not, and that step fails closed. The ladder refuses to run rather than
> going green. Two further numbers here are also wrong: `multiclock on` cannot be set from
> `checks.cfg` at all (genchecks' `[options]` parser ends in `assert 0`, so it is an
> `AssertionError`), and the measured cost of a correctly-configured `clk2fflogic` run is **13–14×**
> per check, not the ~3.6× below. Read the section for the isolated-`write_btor` finding; take
> nothing from it about the ladder.

The body says a negedge read forces `clk2fflogic` and risks a slow or inconclusive `reg_ch0`.
Neither happens, and what does happen is worse.

The generated checks run `prep -flatten -nordff` then `write_btor`, and **`write_btor` accepts a
negedge `$dff` with no warning.** In the emitted btor2, `clk` is declared an input and never used,
and `rd_a` and `regs_a` advance on the same step:

```
10 state 7 rd_a
28 state 27 regs_a
29 read 7 28 4      ; regs_a[rs1]
30 next 7 10 29     ; rd_a' = regs_a[rs1]
```

The half-cycle relationship is silently discarded. The ladder would model a circuit that is not the
RTL, **go green, and take the same wall time** (22.6 s vs 23.5 s baseline). That is
`docs/THREAT_MODEL.md` category 1 — false assurance — and strictly worse than the inconclusive
result the body worried about. **A spike that compared wall times alone would have produced a
confident, wrong green.**

Making it sound is also more expensive than "doubling": with `multiclock on` + `clk2fflogic`,
`reg_ch0` at its generated depth 21 finishes in 3.8 s — implausibly fast, consistent with vacuity,
because riscv-formal's depths are in **clock cycles** and `clk2fflogic` makes a cycle at least two
steps. At doubled depth it takes 84 s, **~3.6× baseline**. So the real cost is a re-derivation of
every depth in `checks.cfg` (ADR-0025 territory) plus ~4× ladder runtime.

## ~~`reg_ch0` does not cover invariant 6 today~~ — FALSE. It covers it, in both directions.

**This section's claim was measured again and does not reproduce** (ADR-0040). Against
`rtl/regfile.v` in stock ladder configuration, deleting the rs2 write-through bypass produces a
counterexample on **bad state property 1** — which is `rvfi_reg_check.sv`'s rs2 assertion
specifically, not a generic failure — at k=20, in about 14 seconds. Deleting the rs1 bypass, both
bypasses, and `regs[waddr] <= wdata + 1` all fail too, on property 0. The only mutation that passes
is removing the `waddr != 0` write-suppression guard, and that is benign by construction: the read
mux returns 0 for `rs1`/`rs2 == 0` unconditionally, so the written value is unreachable and there is
no architectural difference for any check to find.

**Why this brief said otherwise**, confirmed by reproduction at integration rather than inferred: the
stale-run-directory defect described at the top of this file. Running `reg_ch0` clean gives PASS in
22s; deleting the rs2 bypass and re-running *without* removing `checks/reg_ch0/` aborts inside sby in
0s with `ERROR: Directory 'reg_ch0' already exists` and leaves a byte-and-mtime-identical
`PASS 0 21` behind, which `check-baseline.sh` then re-grades as a pass. Removing the directory first
and re-running the identical mutation gives `bad state property 1 reachable at bound k = 20
SATISFIABLE`. The mutation was never evaluated; the previous run's verdict was reported in its place.

**So the formal ladder IS an oracle for the write-through bypass.** Deleting the rs2 bypass is now
this repo's liveness probe for `reg_ch0` — one line, a direct invariant-6 violation, ~14 seconds —
and is the thing to reach for before believing any `reg_ch0` result taken under a changed
configuration. The decision to gate the regfile change on co-simulation rather than on M2 still
stands, but it no longer rests on this argument: it rests on ADR-0032's mutation experiment, where
an injected architectural write outside the retiring instruction's `rd` was missed by the whole
ladder and caught by the co-sim.

## Three things the body missed

- **`test/regfile_tb.v:102` reads `dut.regs[0]`**, so it is not the purely contract-level bench the
  Makefile comment claims. Three of its checks fail against a negedge model *solely* because they
  sample `#1` after a posedge. Extending it is a **rewrite of its timing discipline**, not an
  addition.
- **`test/cosim.cc:183` hard-codes `"uut regfile regs"` and `:191` asserts `depth == 32`.** A
  two-array regfile has neither. It fails closed, which is right — but it *definitely* breaks, and
  duplication adds an obligation: **`regs_a ≡ regs_b` needs its own assertion**, since the co-sim
  reads only one.
- **A new non-local rule.** The change is not only "decode observes the cycle-N architectural
  value" — it is that **`reg_rs1`/`reg_rs2` become valid only in the second half of the cycle.**
  Nothing samples them earlier today (every consumer is posedge-registered), but that is now a rule
  a future change can break silently with no tool to catch it. It is the same shape as invariant 8's
  stall rules and belongs in **CLAUDE.md's invariant list**, not only in the ADR.

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

> **The "negedge read" in this section is one of three options, and it is no longer the recommended
> one.** All three were built and measured; see "Measured — three ways to reconcile invariant 6 with
> a synchronous BRAM read" above, which supersedes this subsection on the choice of read discipline.
> The area case below stands for all three.

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

1. Sail co-sim wired across the full `.S` suite and green at the pre-change SHA — **available now**
   as `make cosim-suite`, graded against `test/COSIM_EXPECTED_FAIL` (ADR-0039)
2. ~~`reg_ch0` PASS immediately before and after~~ — **not achievable as written, and ADR-0040
   decision 2 replaces it.** A negedge regfile makes sby's `prep` step fail closed, so there is no
   "after" reading to take. The gate becomes instead: the ladder wrapper instantiates the posedge
   regfile explicitly and by name at a seam visible from `formal/wrapper.v`, `reg_ch0` PASSes against
   *that*, and a standalone equivalence proof — not written as a bespoke yosys script ending in
   `write_btor`, per ADR-0040 finding 1 — discharges the difference while stating its read-timing
   assumption explicitly (ADR-0017)
3. Full ladder + `formal/EXPECTED_CHECKS` set equality unchanged, from a **fresh** run — `make -C
   formal check` re-runs from scratch as of ADR-0040 and no longer needs to be argued about
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

~~The regfile read alone measured 14.82 ns on faster silicon; the full loop plausibly lands at
50–70 ns on up5k, ~15–20 MHz.~~ **Both halves of that guess are struck.** The loop measured
88.5 ns — worse than the guess — and then reached **76.9 ns, 12.7–13.5 MHz** across four
placements. 12 MHz is a requirement now, `soc-timing` is a required check, and `make soc-timing`
is where it is measured, not `make fit`.

**The last sentence is the one that was most wrong.** Raising Fmax did not mean breaking invariant 1
or 6. It meant noticing that the write-through bypass compared against a live combinational `rs1`
when the regfile's own contract already said the answer belongs to the previous cycle's address
pair — six LUT levels for four lines, no invariant touched. The lesson worth keeping: a low Fmax
was assumed to be the price of a stated value, and most of it turned out to be an accident nobody
had looked for.

## Risks

- ~~**Formal runtime is the load-bearing unknown, not correctness.**~~ **Settled, and the other way
  round** (ADR-0040): correctness of the *model* was the unknown, and runtime is not the issue
  because the ladder declines to run at all rather than running slowly. The fallback this bullet
  names — a standalone equivalence proof with the ladder's wrapper on a posedge model — is the
  option that was chosen. A negedge read register forces
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

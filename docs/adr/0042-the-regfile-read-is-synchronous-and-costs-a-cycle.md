# ADR-0042: The regfile read is synchronous and costs a cycle, and the ladder is told that instruction memory is memory

**Status:** Accepted · 2026-08-01 · *Amends invariant 6 and adds invariant 9. Settles the regfile
question ADR-0038 deliberately left open and ADR-0040 collected data for. Supersedes
[`docs/ideas/fit-the-core-on-the-up5k.md`](../ideas/fit-the-core-on-the-up5k.md) on the choice of
read discipline.*

## Context

An ice40 EBR read is **synchronous**. CLAUDE.md invariant 6 said the regfile read is
**combinational**. Those cannot both hold, and the flip-flop array that made invariant 6 true was
the whole area problem: the core measured **6971 logic cells, 132% of an up5k**, and did not fit.

ADR-0038 measured that and declined to decide the regfile, on the explicit grounds that it needed
spike data. ADR-0040 produced some of that data and declined again. This ADR decides it, and it
decides it against a comparison rather than against one option: **all three ways of reconciling a
synchronous BRAM with invariant 6 were built and measured on this tree.**

| | baseline | **A** negedge read | **B** synchronous + 1 stall | **C** serialised, one array |
|---|---|---|---|---|
| Logic cells | 6971 (132%) | 4137 (78%) | 4223 (79%) | 4175 (79%) |
| EBR | 0/30 | 4/30 | 4/30 | 2/30 |
| `.S` suite | 52/52 | 52/52 | 52/52 | **44/52** |
| Suite cycles, 52 programs | 22512 | 22512 (+0.0%) | 28760 (+27.8%) | n/a |
| Formal ladder | 82 pass | **cannot run at all** | 80 pass, **2 red** | 3 red |

**Area does not choose between them.** 86 cells separate the three, 1.6% of the part. Every one
clears 132% → ~79%. So the decision is about what each costs in verification and in reading, and the
numbers above are what make that sayable rather than arguable.

## Decision 1 — B: the read is synchronous, and decode spends a cycle on it

`rtl/regfile.v` is two arrays (an EBR has one read port, so a second read port is a second copy),
posedge write, **posedge registered read**, with two forwarding points in fabric:

- **write-first** into the read register, covering a write committed at the very posedge that
  captures the read;
- the existing **write-through bypass**, covering a write presented in the cycle the operand is used.

`rtl/decoder.v` gains `operand_stall`: an instruction presents its address pair for one cycle,
bubbles, and issues on the next. **Measured: 4236 logic cells, 80%, EBR 4/30, DSP 4/8** — against
6971 and 132%, a reduction of **2735 cells**. yosys infers `4 × SB_RAM40_4K` from the plain arrays,
with no attribute and no explicit primitive.

### Invariant 6, reworded to what is now true

> **The regfile read is synchronous and takes one cycle. Decode observes, in the cycle it issues,
> the architectural value of rs1/rs2 including a writeback committed in that same cycle.**

The second sentence is unchanged in content — it is the whole thing ADR-0004's stall-only scoreboard
depends on, and it is why decode still needs no forwarding path for the writeback slot. What changed
is that obtaining it takes two cycles instead of none. The flip-flop array was one implementation of
that contract, not the contract itself.

### Invariant 9, the new non-local rule

> **The regfile's answer belongs to the address pair presented in the *previous* cycle.** Decode must
> hold `pc` across the pair, and nothing may consume `reg_rs1`/`reg_rs2` in a cycle whose address
> pair differs from the one presented last cycle.

This is the same shape as invariant 8's stall rules: true today because `operand_stall` enforces it,
breakable silently by a future change, and not derivable from reading `rtl/regfile.v` alone.
`test/regfile_tb.v` pins it directly — it presents x5, then points `rs1` at x6 in the use cycle and
asserts that **x5's** value comes back.

### It lands on the existing stall mechanism, and adds no flush

`operand_stall` is a fifth stall **reason** on the bubble mechanism ADR-0009 already defines, not a
third shape. It shares the `hazard` arm verbatim: `pc <= pc; out <= '0`. **No flush logic exists or
is needed** (invariant 1) — the stalled instruction is not speculative and is never killed, because
holding `pc` makes `rtl/fetcher.v` re-present the same instruction next cycle. That is the reason
this is a stall rather than a fetch-ahead pipeline register, and it is worth stating because a
fetch-ahead design *would* have needed a kill signal.

### The stall is conditional, and the condition was measured

`operand_stall` is neither "every instruction waits a cycle" nor a cycle counter. It fires only when
an operand the instruction **actually reads** was not the one presented last cycle:

```systemverilog
assign operand_stall = !read_taken || (uses_rs1 && prev_rs1 != rs1) ||
                                      (uses_rs2 && prev_rs2 != rs2);
```

Both refinements were measured over the 52-program suite rather than assumed:

| predicate | suite cycles | vs baseline |
|---|---|---|
| baseline (combinational read) | 22512 | — |
| address pair unchanged | 28760 | **+27.8%** |
| ...and only the ports it reads | **26562** | **+18.0%** |

The `uses_rs*` gate recovers **2198 cycles, 35% of the penalty**, for one extra term on a line that
already existed — and it is `hazard_rs1`'s own predicate reused verbatim, so it reads as the same
kind of thing rather than as a new special case. That is why it was taken; a refinement buying half a
percent would not have been, because CLAUDE.md protects this mechanism for its legibility.

It is safe because `lui`, `jal`, `auipc` and the zimm CSR forms — the four instructions with
`uses_rs1 == 0` — each **override** `out.rs1`/`out.rs2` in the publish block rather than passing
`reg_rs1`/`reg_rs2` through, so a skipped fetch cannot leak a stale operand into the executor. And
`rvfi_rs1_valid` is byte-identical to `uses_rs1` while `rvfi_rs2_valid` *is* `uses_rs2`, so RVFI
reports zero for exactly the operands whose fetch is skipped. Note also which direction that
dependency runs: the stall predicate reads `uses_rs*`, which is not `ifdef`'d, so nothing here
weakens ADR-0020's argument that RVFI instrumentation is write-only with respect to the core.

### The CPI cost is the trade this project says it makes

**+18.0% cycles across the suite, measured.** CLAUDE.md's fourth line says CPI is deliberately
sacrificed for a design that reads well; this is that sentence being cashed. It is recorded here as a
number so a future reader asking "why is this core slower than it needs to be?" finds the answer and
the alternative rather than re-deriving both. The alternative was A, at +0.0% — see decision 4.

## Decision 2 — the ladder is told that instruction memory is a function of its address

`formal/wrapper.v` gains an assumption: **if `imem_addr` is unchanged from last cycle, `imem_data` is
unchanged too** (and the same for the second fetch port).

Without it, `hang` and `liveness_ch0` fail — real counterexamples at k = 30, not timeouts, on an
otherwise 82-pass ladder. The cause is exactly this change: decode decides whether the fetched
operand belongs to the instruction it is issuing by comparing `rs1`/`rs2`, which are combinational
out of `imem_data`. Left fully free, the environment hands the held PC a different instruction word
every cycle and decode never issues.

**This is not a fairness fudge, and it is not a weakening dressed as a fix.** The structural fact is
that a ROM asked twice for the same address answers the same both times, and there is no write port
anywhere in this design that could reach instruction memory. `formal/imemcheck.sv` already relies on
it, in the *stronger* `rand_const_reg` form — a fixed address whose data is fixed for the entire
trace. Per ADR-0017 the assumption names that fact at the site where it is made, not only here.

Three things about it are deliberate:

1. **It is the weakest form sufficient for the property.** Consecutive-cycle stability, not a full
   functional model. A full model needs an unbounded array; this is implied by one, is all the
   operand-fetch cycle needs, and leaves the environment free to answer differently for an address
   revisited later.
2. **The cost is named.** An assumption can only make checks easier. A defect that manifests only
   when the same address yields different data in consecutive cycles is now invisible to the ladder.
   No such defect is possible against real memory — that is the point — but it is a real narrowing of
   the environment.
3. **`formal/wrapper.v`'s `RISCV_FAIRNESS` comment is now wrong and says so.** It asserted that
   "littlecpu has no port an adversarial environment could hold to defeat forward progress." A
   synchronous regfile read makes `imem_data` exactly such a port. The comment is corrected in place
   rather than left to rot.

### The rejected alternative: an imem-independent ready predicate

`operand_stall` could have been driven off "the PC changed at the last posedge" instead of off the
address pair. `pc` is decode-owned state, so `hang` and `liveness_ch0` would have gone green with no
harness change at all.

**Rejected.** It does not remove the dependency on `imem_data` being stable — it *hides* it. The
operands fetched at the posedge are still keyed to `imem_data`'s value in the previous cycle, so
under a varying `imem_data` the core would issue with a mismatched operand instead of refusing to
issue. That converts a liveness failure, which the ladder reports loudly, into a correctness failure
that only shows up if some `insn_*` check happens to construct the right trace. **Given a choice of
which check a bad assumption breaks, break the one that fails closed.**

## Decision 3 — `reg_ch0` covers invariant 6, and here is the probe that shows it

ADR-0040 established this against the flip-flop regfile and named the probe. It was re-run against
**this** regfile, before any green run from the ladder was believed:

| `rtl/regfile.v` | `reg_ch0` | wall |
|---|---|---|
| as shipped | `PASS 0 31` | 32s |
| **rs2 write-through bypass deleted** | **`bad state property 1 reachable at bound k = 20 SATISFIABLE`** | 32s |

Bad state property **1** is `rvfi_reg_check.sv`'s rs2 assertion specifically, not a generic failure —
the same signature ADR-0040 recorded. **Deleting the rs2 write-through bypass remains this repo's
liveness probe for `reg_ch0`, and it still works after the regfile was rewritten underneath it.**

Note the wall times are equal, which is exactly why ADR-0040 states the probe as a property rather
than as a duration. Reach for the `reachable at bound` line, never the clock.

## Decision 4 — why A (negedge) was rejected, and it was not about area

A is **cheaper in cells (4137 vs 4236, by 99) and free in cycles (+0.0% vs +18.0%)**. It was still
rejected, on one ground:

**The generated ladder cannot run against it at all.** sby's own `prep` step fails closed on mixed
clock polarity, on the first check it reaches:

```
prep: ERROR: CLK clock on $flatten\wrapper.\dut.\regfile.$procdff$2308 ($dff) from module
             rvfi_testbench also used with opposite polarity, run clk2fflogic instead.
DONE (ERROR, rc=16)
```

That is not one red check. It is all 82, because the model step dies before any of them. `clk2fflogic`
— the remedy the message names — is rejected by ADR-0040 findings 2 and 3 and stays rejected: at
genchecks' own depths it makes `reg_ch0` PASS on a regfile with the rs2 bypass deleted, and
`checks.cfg` structurally cannot express the corrected horizon.

So shipping A means substituting a posedge model at a seam in `formal/wrapper.v` (ADR-0040 decision
2) and discharging the difference with a standalone equivalence proof. That proof was built during
this work. **Its base case passes at depth 20 and k-induction does not close**: the inductive
invariant is equality of the two storage arrays, and yosys's Verilog frontend rejects the
hierarchical references needed to state it (`ERROR: Identifier '\gold.regs' is implicitly declared`).
The result would be a bounded proof carrying a bounded ladder, with one extra indirection on
`reg_ch0` — the single check that ties RVFI's self-report back to the real register file, and
M2 term 3.

**A also adds an obligation nothing checks**: that `reg_rs1`/`reg_rs2` are sampled only in the second
half of the cycle. B's new obligation is checked — by `hang` and `liveness_ch0`, which is how it was
found. On a core that is not verified (ADR-0037's six-term conjunction; **M2 is not reached**), an
enforceable rule that costs 18% of the cycles beats an unenforceable one that costs nothing.

## Decision 5 — why C (one array, serialised ports) was rejected

C saves **2 EBRs out of 30, in a resource that is 87% idle**, and costs 48 more logic cells than A.
It reached **44/52** on the `.S` suite after two implementation attempts — eight programs failing
with `MONITOR-ERROR 131`/`132` — with `reg_ch0`, `hang` and `liveness_ch0` all red.

The reason is structural and visible in the RTL: with one array, rs1's word is read a cycle before it
is used, so it must be **parked**, and any write landing while it is parked has to be forwarded onto
the parked copy. That is a **second bypass level** — the first step toward the forwarding network
invariant 4 forbids — bought for two block RAMs nothing wants.

## Decision 6 — `make fit` becomes a ratchet

It was report-only because the core did not fit. It does now, and a number nothing defends drifts
back. `FIT_MAX_LC` is **4400** against a measurement of **4236**.

The budget is deliberately not the measurement. Edits that synthesise to identical hardware move the
count by tens of cells, because ABC's result depends on the order it sees the netlist in. A ratchet
pinned to 4236 would go red on changes that alter nothing, and the only way to clear it would be to
raise the number — which is how a ratchet becomes a rubber stamp. 164 cells is more than three times
the observed noise band. It is also **not** set at the part's 5280: fitting is the floor, not the
goal.

`make fit` also learned nextpnr's second spelling of an IO placement failure. The 132% configuration
died with `Unable to place cell ... no BELs remaining`; the 80% one dies with
`Unable to find a placement location for cell` on an `mem_rdata` pad. Only the first was matched, so
a normal run printed a warning telling the reader not to trust the number.

## The ASIC note

On a Tiny Tapeout or any flow with no BRAM primitives, this same RTL synthesises back to a
flip-flop array plus a read register and the two forwarding muxes. **It works there — it is simply
not smaller there**, and the `operand_stall` cycle is then paid for nothing. That is a different
problem for a different ADR; nothing here should be read as claiming the design is portable-optimal,
only that it is portable-correct.

## Three corrections to merged documents

All three came out of measurement during this work.

1. **ADR-0040's standing warning names `formal/equiv.sh` as exposed to the polarity loss. It is
   not.** The isolated reproduction holds — `prep -flatten -nordff` then `write_btor` on a negedge
   regfile emits `2 input 1 clk` at line 3 and never uses node 2 again, with zero warnings — but
   `equiv.sh` has no backend. Its `prep -flatten -top littlecpu` retains both negedge `$dff` cells
   (`CLK_POLARITY 1'0`) alongside 32 posedge ones, and `equiv_make`/`equiv_simple`/`equiv_induct`
   reason on those cells directly. The warning is right about `write_btor`/`write_smt2` scripts and
   wrong about this one. (Moot for the shipping RTL, which is all-posedge, but it would have been
   load-bearing had A been chosen.)
2. **The area brief says yosys infers `SB_RAM40_4K` and specifically "no `SB_RAM40_4KNR`". Both
   halves are backwards.** A negedge read infers `4 × SB_RAM40_4KNR`; the posedge read that ships
   infers `4 × SB_RAM40_4K`. Immaterial to the decision — EBR inference needs no attribute either
   way — but the claim as written is wrong for the design it was written about.
3. **A measures 4137 logic cells here against ADR-0038's 4017 for the same lever**, while the 6971
   baseline reproduces exactly. The earlier variant is not in the tree and cannot be diffed, so
   **neither figure should be quoted to four digits.**

## Consequences

- **Invariant 6 changes meaning and invariant 9 is new.** CLAUDE.md carries both. A future change
  that samples `reg_rs1`/`reg_rs2` in a cycle where the address pair moved is a bug that no gate in
  this repo would catch — invariant 9 exists because that is true.
- **The stall list grows to five reasons** (ADR-0026 said four). The mechanism count does not change:
  `operand_stall` bubbles, exactly like a RAW hazard.
- **`test/regfile_tb.v` and `test/decoder_tb.v` were rewritten, not extended.** Both were re-timings:
  the regfile bench had no notion of a fetch cycle, and five of the decoder bench's checks failed on
  timing alone with nothing wrong in the RTL. `decoder_tb`'s serialization vectors additionally had
  to stop leaning on the previous vector sitting in `decoder_out` — the operand-fetch bubble clears
  that slot one cycle ahead of every instruction now — and now drive `executor_out.valid` to say what
  they mean. Both benches were mutation-tested; the mutations and what caught them are in the pull
  request.
- **`test/cosim.cc`'s probe moved from `"uut regfile regs"` to `"uut regfile regs_a"`, and was proved
  to read the live array rather than assumed to.** ADR-0032's mutation — an extra architectural write
  outside the retiring instruction's `rd`, gated past cycle 40 — is **missed by `make test` at 52/52
  and caught by the co-simulation in 49 of 52 programs** (the three that still agree are too short to
  reach cycle 40). That is the same result ADR-0039 measured against the one-array regfile, which is
  what makes it evidence about the probe rather than about the mutation.
- **The co-simulation reads one of two arrays, so it now asserts they are equal.** `regs_b` has
  exactly one consumer (`reg_rs2`) and the co-sim never looks at it, so an rs2-side write defect
  would produce a co-simulation that agrees perfectly while the core computes wrong answers.
  `test/cosim.cc` checks `regs_a == regs_b` every cycle of all 52 programs and `test/regfile_tb.v`
  asserts it over directed vectors. Confirmed by mutation: an `regs_b`-only corruption is invisible
  to the architectural comparison and caught by both of these.
- **`docs/ideas/fit-the-core-on-the-up5k.md` is superseded on the read discipline.** Its area case
  stands for all three options; its recommendation of a negedge read does not.
- **The negedge-BRAM regfile comes off CLAUDE.md's deferred list** — not built, and now decided
  against with measurement rather than pending.

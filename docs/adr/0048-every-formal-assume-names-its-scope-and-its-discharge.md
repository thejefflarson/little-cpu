# ADR-0048: Every formal `assume` names its scope as well as its discharge

**Status:** Accepted · 2026-08-01 · *Supplements ADR-0017; scopes ADR-0010, ADR-0025*

## Context

ADR-0017 requires that every `assume` added to a component proof "states the structural fact it
models and where that fact is itself checked." The rule has never been audited against the tree.
This ADR is that audit, and it found the rule to be missing a third clause.

`rtl/executor.v`'s

```verilog
always_comb assume(in.rs1 <= 32'h0000000f);
always_comb assume(in.rs2 <= 32'h0000000f);
```

is an honest, well-documented assumption. It names its fact (ADR-0010's "restrict it… record the
restriction"), it says why the restriction exists (keeping the divide invariant's intermediate
arithmetic inside 64 bits), and it sits directly above the invariant it was written for. It is also
**proof-global**: an `always_comb assume` with no guard constrains every assertion in the module,
not the ones near it. The four multiply assertions forty lines above it are therefore asserting
about operands in 0..15, where every high half is zero and every sign bit is zero — so they hold
whatever the multiplier's sign logic does.

Measured, not argued. Each row is one mutation of `rtl/executor.v` in a scratch copy, run as
`sby -f components.sby executor` (the tree is untouched; `components_executor` is PASS at
`18d17a2` in 6.6s):

| mutant | what it breaks | verdict |
|---|---|---|
| `multiply & 64'h0000_0000_ffff_ffff` | MULH/MULHU/MULHSU all return 0 | **PASS** |
| `mul_sign_y = rs2[31] & (is_mulh \| is_mulhsu)` | MULHSU treats rs2 as signed | **PASS** |
| `mul_sign_x = 1'b0` | MULH/MULHSU stop sign-extending rs1 | **PASS** |
| `mul_sign_y = 1'b0` | MULH stops sign-extending rs2 | **PASS** |
| `multiply + 1` (control) | the low half | FAIL, 0.5s |
| `mul_sign_x = rs1[0] & …` | sign taken from bit 0 | FAIL, 0.5s |

Three of the four defects ADR-0010 names by hand — "the sign enables are also swapped (MULHSU is
signed × unsigned, the code has it backwards), and the sign bits are taken from bit 0 rather than
bit 31" — survive the proof that was written to catch them. The two that are caught are what makes
this insidious: the multiply assertions are not dead, they are *narrowed*, and a green
`components_executor` reads exactly the same either way.

This is not a bug in the assumption. The assumption is correct and necessary for the property it
was written for. It is a bug in what the *reader* is told: nothing at the site says which other
assertions it also constrains.

## Decision

**ADR-0017's rule gains a third clause. An `assume` must state:**

1. **the structural fact it models** (ADR-0017, unchanged);
2. **where that fact is discharged** — a named check, a named test, or the word **nowhere**
   (ADR-0017, unchanged, with "nowhere" made an explicitly acceptable and expected answer);
3. **its scope** — which assertions it is in force over, and whether that set is larger than the
   one it was written for.

Clause 3 is the new one, and it is the one this audit exists to add. An unguarded `always_comb
assume` inside a shared `` `ifdef FORMAL `` block is in force over *every* assertion in that
module. That is a fact about SystemVerilog, not about the design, and it is invisible from the
lines around the assume.

**Two ways to satisfy clause 3**, and the choice is the author's:

- **Narrow the assume** to the assertions it was written for — guard it, or move it into the
  `if (…)` of the property that needs it. Preferred where it costs nothing.
- **Write the scope down** at the site. Required where narrowing would change the proof.

`rtl/executor.v`'s operand cap takes the second route in this change (the first would alter what
`components_executor` proves, and that file's divide invariants are owned by in-flight work — see
Consequences). The rest of the tree's assumes are annotated in place.

**No assume in this repo may be deleted or weakened to make a check pass.** ADR-0010 already says
the remedy for an intractable proof is to restrict it *and record the restriction*; this ADR adds
that the record must include who else the restriction binds.

## The census

Every `assume` in `formal/*.sv`, `formal/wrapper.v`, and the `` `ifdef FORMAL `` blocks of `rtl/`,
at `18d17a2`. There are 24 sites in 6 files. "Discharged" is where the modelled fact is *checked*,
not where it is *believed*.

### `formal/wrapper.v` — the full-core ladder harness

| site | fact modelled | discharged | scope |
|---|---|---|---|
| `wrapper.v:83` | instruction memory is a function of its address: same `imem_addr` two cycles running ⇒ same `imem_data` | **nowhere.** Structural: `rtl/imemory.v` is a `$readmemh` ROM with no write port, and ADR-0044 keeps it that way. No check asserts it | **all 82 generated checks.** Written for `hang` and `liveness_ch0` (ADR-0042 measured both red without it); also in force over all 70 `insn_*`, `reg`, `pc_fwd`/`pc_bwd`, `causal*`, `unique` and `cover` |
| `wrapper.v:84` | the same, for the fetch window's second port `imem_addr2`/`imem_data2` (ADR-0003) | as above | as above |

`RISCV_FAIRNESS` is an explicitly empty block with its reasoning written out — the correct shape
for "there is nothing to assume here", and it already reads the way clause 3 asks for.

### `formal/imemcheck.sv` — fetch against a pinned halfword

| site | fact modelled | discharged | scope |
|---|---|---|---|
| `imemcheck.sv:60` | the fetch bus answers from memory: the low half of the word at `uut_imem_addr` is the checker's pinned halfword | **nowhere.** Structural, same ROM as above. `rvfi_imem_check` asserts the *core's* consistency with it, never the environment's | the task's one `rvfi_imem_check` assertion |
| `imemcheck.sv:62` | same, upper half of the first fetch port | as above | as above |
| `imemcheck.sv:64` | same, low half of the second port (ADR-0003) | as above | as above |
| `imemcheck.sv:66` | same, upper half of the second port | as above | as above |

Sound because all four constrain **DUT inputs** (`uut_imem_data`/`uut_imem_data2`) against a
`rand_const_reg` pair, i.e. they narrow the environment and never the core. The scope is narrow by
construction: the task has exactly one assertion, inside riscv-formal's unmodified checker.

### `formal/dmemcheck.sv` — loads against a one-address shadow

| site | fact modelled | discharged | scope |
|---|---|---|---|
| `dmemcheck.sv:86` | the data bus returns, one cycle after the request, whatever was last stored to that address | **nowhere**, and unlike the imem cases there is no clean structural argument either: `rtl/memory.v` is in no test path at all and ADR-0010 records a read-override bug in it. The modelled memory is not the memory this repo has | the task's one `rvfi_dmem_check` assertion |

### `formal/complete.sv` — the whole-ISA walk *(out of scope for edit; owned by the `complete` exclusion-set work)*

| site | fact modelled | discharged | scope |
|---|---|---|---|
| `complete.sv:59` | store-then-reload forwarding for the most recent store only | **nowhere**, same as `dmemcheck.sv:86` and with the same missing structural backing | `complete`'s single `assert(spec_valid && !spec_trap)` |

### `formal/pcloop.sv` — the composed fetcher+decoder proof *(not edited here; `bb6e228` owns this file)*

| site | fact modelled | discharged | scope |
|---|---|---|---|
| `pcloop.sv:126` | reset is asserted before the first clock edge | **nowhere.** Structural: `test/testbench.v` and `formal/*.sv` all drive `reset = 1` initially | the whole `pcloop` task |
| `pcloop.sv:127` | the same, restated for the pre-`clocked` window | as above | as above |
| `pcloop.sv:128` | reset is a once-at-the-start pulse and never returns | **nowhere.** True of every harness in the tree; asserted by none of them | the whole `pcloop` task. Larger than written for: it was written for the two pc-history assertions and also binds the echo, the hold and both redirect assertions |

### `rtl/decoder.v` — `components_decoder`

| site | fact modelled | discharged | scope |
|---|---|---|---|
| `decoder.v:1005` | reset holds before the first clock edge | **nowhere** (structural, as above) | the whole task |
| `decoder.v:1006` | the same, restated for the pre-`clocked` window | **nowhere** | the whole task |
| `decoder.v:1013` | reset is a once-at-the-start pulse | **nowhere** | the whole task — written for the two pc-increment assertions, also in force over the ADR-0028/ADR-0030 trap-arm assertions added later |
| `decoder.v:1023` | `in.pc == pc`: the fetcher echoes the decoder's own pc combinationally (`rtl/fetcher.v`'s `out.pc = pc`, wired pc→pc in `rtl/littlecpu.v`) | **`formal/pcloop.sv`** asserts exactly this, and as of `bb6e228` it is a real, running discharge — `make -C formal components_pcloop`, in CI, PASS in 2.7s. It was red and unrun for the whole period this audit ran; see F2 | the whole task. ADR-0017 analysed its effect on the pc-increment pair; it is also in force over the bubble, hazard, `one_of`, trap-cause and trap-arm assertions |

### `rtl/executor.v` — `components_executor`

| site | fact modelled | discharged | scope |
|---|---|---|---|
| `executor.v:266` | reset holds before the first clock edge | **nowhere** (structural) | the whole task |
| `executor.v:267` | the same, restated for the pre-`clocked` window | **nowhere** | the whole task |
| `executor.v:279` | reset is a once-at-the-start pulse and cannot be re-asserted mid-divide | **nowhere** | the whole task — written for the divide assertions, also in force over the four multiply assertions and the ADR-0015 freeze block |
| `executor.v:330` | the decoder emits at most one op-select flag (`$onehot0` over all 29 `is_*` fields of `decoder_output`) | **`rtl/decoder.v`'s own `one_of` assertion** (`components_decoder`, `assert($onehot(…))` under `instr_valid`) — the one assume in the tree with a real, running discharge | the whole task. Written for the arithmetic assertions; also in force over the freeze block and every divide invariant. Verified complete against `rtl/structs.v`: all 29 flags listed, `is_valid_instr` correctly excluded |
| `executor.v:392` | ADR-0010's recorded restriction: operands in 0..15 keep the divide invariant's intermediate arithmetic inside 64 bits | **nowhere.** It is a restriction, not a fact about the design — nothing discharges it and nothing should | **the whole task, including the four multiply assertions it was not written for.** See F1 |
| `executor.v:393` | the same, for `in.rs2` | as above | as above |
| `executor.v:408-413` (6 sites) | ADR-0009's stall protocol: decode holds `in` steady for the duration of a multi-cycle divide | **nowhere.** `rtl/decoder.v`'s own task asserts the pc holds on a stalled cycle, not that `decoder_out` holds | guarded (`state == divide \|\| $past(state) == divide`), so genuinely scoped to the divide assertions. The one guarded `assume` in `rtl/` |

## Findings

**F1 — `rtl/executor.v`'s operand cap makes three of ADR-0010's four named multiplier defects
invisible.** The table at the top of this ADR is the demonstration. Recorded, not fixed: the
divide-invariant assumes are owned by in-flight work, and fixing this correctly means narrowing the
cap to the divide assertions, which changes what `components_executor` proves and belongs with that
work. The site carries a scope statement in the meantime — clause 3's second route.

**F2 — `formal/pcloop.sv` was red and unrun. FOUND INDEPENDENTLY HERE; FIXED CONCURRENTLY BY
`bb6e228` (ADR-0046).** This audit measured it at `18d17a2` as `sby -f components.sby pcloop` →
`DONE (FAIL, rc=2)` in 0.67s at `pcloop.sv:273 step 3`, and attributed it from the counterexample
VCD: the failing cycle's predecessor is the first post-reset cycle, where `operand_stall = 1`,
`stall = 1` and the pc legitimately holds, while `prev_may_stall` reads 0 because `f_may_stall`
lists **four** of `rtl/decoder.v`'s **five** stall terms. ADR-0042 added `operand_stall`; the
over-approximation was never widened, so it had stopped being one. Same class as F1 one level up: a
guard whose excused set is smaller than the RTL's.

The ladder-depths work landed the fix while this was in flight, reaching the same attribution
independently, and it went further than this finding did in two ways worth recording:

- **The fix needed no new port, and this ADR's first draft said it did.** That claim was wrong.
  `rs1`/`rs2` are decoder *output ports*, so `pcloop.sv` transcribes `rtl/decoder.v`'s
  `prev_rs1`/`prev_rs2`/`read_taken` register into the harness and over-approximates at the
  comparison, exactly as every other `f_may_stall` term does. Recorded because "this cannot be done
  from the module's own nets" is the kind of claim that closes off a fix, and it was written without
  being checked.
- **The task is wired up now**: `formal/Makefile` has a `components_pcloop` target and
  `.github/workflows/ci.yml`'s `components` job runs it. Re-measured on the merged tree for this
  ADR: `DONE (PASS, rc=0)` by k-induction in 2.7s.

What survives, and is why this stays in the census rather than being deleted: **the defect was
invisible for as long as it was because nothing ran the task.** `rtl/decoder.v:1023`'s
`assume(in.pc == pc)` was, for that whole period, discharged by a check nobody invoked — and a
discharge like that reads identically in the source to a real one. That is the durable lesson, and
it is why clause 2 asks *where* rather than merely *whether*.

One methodological correction, because it was nearly published as a finding: a sandbox diagnostic
asking whether widening `f_may_stall` closes the red was run twice and returned no verdict, and the
draft inferred from that that widening "makes the query dramatically harder." **That inference was
wrong and the measurement was an artefact.** `formal/components.sby` now pins `pcloop: smtbmc
boolector` precisely because the default solver sits on this task for over fourteen minutes without
returning, which is what both attempts hit. Solver choice, not problem hardness — a non-answer is
not evidence about the problem.

**F3 — the two divide-completion assertions are unreachable in the base case.** `components.sby`
sets no `depth`, so `mode prove`'s basecase runs to 20 steps. The real divider needs 33 cycles from
issue, so `$past(state) == divide && state == init` cannot occur in any concrete trace the basecase
explores. Demonstrated: replacing `assert(out.rd_data == divu_ref)` (and the `remu_ref` twin) with
`assert(1'b0)` gives **basecase = pass**, induction = FAIL — where every other assertion guard in
the module gives basecase = FAIL in 0.5s. Those two assertions are the *only* place the divide
invariants are tied to `out.rd_data`, so the end-to-end divide result rests entirely on the
induction step and on the invariant chain feeding it; no concrete trace ever computes a division.
Per ADR-0025 this is recorded as a fact about the depth **and** the guard, and neither is raised
here — raising the basecase past 33 is a depth change requiring its own empirical evidence, and the
ladder-depths work is in flight.

**F4 — the two writable-memory assumes are true of a memory this repo does not have.**
`imemcheck.sv` and `wrapper.v` model a ROM, and `rtl/imemory.v` really is one at every address, so
"nowhere" there costs nothing. `dmemcheck.sv:86` and `complete.sv:59` model a *writable* memory.
The only one in the tree is `rtl/memory.v`, and it answers any address at or past `4*RAM` with
`mem_rdata <= mem_wdata` rather than with stored data. `dmem_addr` and `mem_addr` are free 32-bit
values in both harnesses, so the assumed model and the real module disagree over the overwhelming
majority of the address space. (ADR-0010's "`rtl/memory.v` has the same shape of problem: it is in
**no** current test path" is stale as of `test/mem_tb.v`, which is on `make test-units` — but that
bench only asserts an out-of-range read does not *alias* `ram[0]`; it never says what one returns,
so it does not discharge this either.) ADR-0044 rules the placeholder out as a starting point and
does not replace it, so when the real memory system is built there is no check anywhere that will
hold it to what these two proofs assumed of it.

**F5 — the same operand cap blanks the signed divide path, and that one needs new assertions
rather than a narrower assume.** With operands in 0..15, `op_sign_x` and `op_sign_y` are constant
zero, so `assert(op_sign_x == in.rs1[31])` and its twin read `assert(0 == 0)`, and the only
completion assertions in the task are the *unsigned* pair (`divu_ref` / `remu_ref`). Nothing
asserts that the signed sign-restore happens at all. Measured: deleting
`op_sign_x != op_sign_y ? -mul_div_store[31:0] :` from the `op_is_div` capture **PASSES**, and
deleting `op_sign_x ? -mul_div_x[31:0] :` from the `op_is_rem` capture **PASSES**. ADR-0012's sign
wrapper — the reason this divider is allowed to be unsigned at all — has no formal coverage here.
`test/exec_tb.v` covers it; the component proof does not, and did not appear not to.

## Vacuity: every compound `assert` guard, demonstrated

Method: replace the assertion body with `assert(1'b0)` under its existing guard and rerun the task.
**basecase = FAIL is a reachability witness** — the solver produced a concrete trace in which the
guard holds. basecase = pass means the guard is not reachable within the basecase depth, which is
finding F3's shape. No assume was touched to produce any of these.

`components_executor` (`sby -f components.sby executor`), all twelve guards:

| guard | verdict |
|---|---|
| `clocked && !reset && !$past(reset) && $past(accessor_stall)` (ADR-0015 freeze) | FAIL 0.6s — reachable |
| `… && !$past(accessor_stall) && $past(state)==init && $past(in.is_mul)` | FAIL 0.5s — reachable |
| the same for `is_mulh` / `is_mulhu` / `is_mulhsu` | FAIL 0.5s each — reachable |
| `state == divide` (op-latch tie-back) | FAIL 0.5s — reachable |
| `state == divide` (counter bound) | FAIL 0.5s — reachable |
| `state == divide && mul_div_counter > 0` (`mul_div_y` scaling) | FAIL 0.5s — reachable |
| `state == divide` (division identity) | FAIL 0.5s — reachable |
| `state == divide` (quotient bound) | FAIL 0.5s — reachable |
| `$past(state)==divide && state==init && $past(in.is_divu)` | **basecase pass** — F3 |
| the same for `is_remu` | **basecase pass** — F3 |

`components_decoder` (`sby -f components.sby decoder`), all seventeen guards. Ten of them are
`always_comb` with no `clocked` term, so those were run twice: once with `assert(1'b0)` and again
with `assert(!(clocked && !reset))`, which forces the witness to be a real post-reset cycle rather
than the pre-reset step 0. Both rounds FAIL for every one:

| guard | verdict |
|---|---|
| `clocked && !branch_jump && !prev_stall && !prev_reset && prev_uncompressed` | FAIL @ step 4 |
| the same with `!prev_uncompressed` | FAIL @ step 4 |
| `clocked && prev_stall && !prev_reset` | FAIL @ step 3 |
| `clocked && !out.valid` | FAIL @ step 1 |
| `rs1 == 0` / `rs2 == 0` | FAIL @ step 1 (post-reset witness) |
| `instr_valid` | FAIL @ step 1 (post-reset witness) |
| `instr_illegal` / `instr_ebreak` / `instr_ecall` / `load_misaligned` / `store_misaligned` | FAIL @ step 1 each (post-reset witness) |
| `!trap_pending` / `trap_pending` | FAIL @ step 1 each (post-reset witness) |
| `clocked && !prev_reset && prev_trap_entry` (→ `mtvec`, and → `out.rd == 0`) | FAIL @ step 3 |
| `clocked && !prev_reset && prev_mret_entry` | FAIL @ step 3 |

`components_pcloop`, re-measured on the merged tree where the task is green (the first pass was
taken while it was red, which needed the sequential-advance assertion neutralised first; that
scaffolding is gone and the numbers below are the clean ones): echo guard FAIL @ step 1,
`prev_hard_stall` @ step 3, both redirect guards @ step 4. The sequential-advance guard is reachable
**in both width branches** — `assert(!prev_uncompressed)` and `assert(prev_uncompressed)` under it
both FAIL @ step 4 — so neither the 4-byte nor the 2-byte arm is vacuous. The witnesses moved out by
one step relative to the red-tree run, which is the operand-fetch cycle `f_may_stall` now accounts
for.

`formal/cover.sv` is the full-core harness's own reachability demonstration and needs no mutation.
Re-measured for this audit rather than quoted: `make -C formal cover` → `DONE (PASS, rc=0)` in 12s,
all five goals reached, including the compound one (`mem_read >= 2 && mem_write >= 2 &&
long_insns >= 2 && comp_insns >= 2`) at step 11.

`formal/complete.sv`'s single guard (`!reset && rvfi_valid && !rvfi_trap`) needs no mutation either:
the check is red today (M2 term 5), and a counterexample is a reachability witness by construction.

## Rationale

The alternative to clause 3 was a lint: forbid unguarded `always_comb assume` in a shared FORMAL
block outright. Rejected — `executor.v:330`'s `$onehot0` is exactly such an assume, is correct, and
*should* be proof-global, because the decoder really does emit at most one flag on every cycle of
every assertion. A rule that flags the good case and the bad case identically teaches people to
silence it. What distinguishes the two is not the syntax but whether the author knew the scope, and
the only way to check that is to make them write it down.

Making "nowhere" an explicit and expected answer is the other half. ADR-0017's phrasing — "say
which higher-level check discharges it" — reads as though a discharge always exists, and eleven of
the twenty-four sites here have none. An audit that had to invent a discharging check for each of
those would have produced twenty-four plausible sentences and no information.

## Consequences

- Every `assume` site in `formal/wrapper.v`, `formal/imemcheck.sv`, `formal/dmemcheck.sv`,
  `rtl/decoder.v` and `rtl/executor.v` carries fact / discharge / scope at the site.
  `formal/complete.sv` and `formal/pcloop.sv` are censused here but not annotated: both are owned
  by in-flight work, and this ADR's table is their record until that work lands.
- **F1 is handed to the executor real-arithmetic work**, which owns those lines. The fix is to
  narrow the cap to the divide assertions; the four multiply assertions need no cap (a single
  combinational stage, full 32-bit correctness is provable directly, as the comment above them
  already says).
- **F2 is closed, by `bb6e228` rather than by this ADR** — both halves: `f_may_stall` carries the
  operand-fetch term, and `components_pcloop` is a `make` target and a CI job. It is kept in the
  census because the *shape* is the point: an assume whose discharge is a task nothing invokes looks
  exactly like an assume whose discharge is real.
- **F3 is a standing caveat on `components_executor`**, in the same family as ADR-0010's ALTOPS
  caveat: the divide result is proved inductively and never simulated. It is not a reason to raise
  the depth without evidence (ADR-0025).
- ADR-0017 is supplemented, not superseded. Its `in.pc == pc` analysis stands; this ADR adds that
  the same assume is also in force over the trap-arm assertions written into that block two
  milestones later, which is exactly the kind of quiet scope growth clause 3 exists to catch.

# ADR-0037: An empty formal baseline is not M2, and the milestone criterion said it was

**Status:** Accepted · 2026-08-01 · *Amended by
[ADR-0045](0045-two-m2-terms-close-by-amendment-and-one-was-already-met.md) (terms 2 and 3) and by
[ADR-0047](0047-non-perturbation-is-proved-structurally-and-equiv-sh-is-retired.md) (term 4).
Recorded at integration. Amends the M2 row of `CLAUDE.md`'s
milestone ladder; corrects ADR-0028's `dmemcheck` claim; corrects ADR-0022's account of its own
enforcement; corrects ADR-0034's reproduction command. Ratifies a branch-protection
recommendation that a human must apply.*

## Context

`6309b3e` took `formal/EXPECTED_FAIL` to empty: 82 checks generated, 82 pass, both set equalities
matching in both directions. `CLAUDE.md`'s milestone ladder said, in as many words, that this was
M2's signal:

> the `formal/EXPECTED_FAIL` baseline of ADR-0022 reaching empty is the signal

The engineer who landed it declined to claim M2 and listed why. That is the right call, and the
fact that it had to be a *judgement call* is the defect this ADR is about. **A milestone criterion
that can be satisfied by a change nobody believes completes the milestone is not a criterion. It is
a coincidence with a checkbox next to it.**

This is the same failure mode ADR-0033 named for the ladder itself — a gate that reports green
without having checked what it claims to check — applied one level up, to the milestone table
rather than to a check. The table is where a reader goes to find out whether the project is done.

## Decision

### 1. M2 is NOT reached at `6309b3e`.

Six things are true of the ladder that just went green, and each is independently sufficient to
deny the milestone. M2's own wording is "the pipelined core re-proves everything the serialized
core proved", and the serialized core's result was not bounded, not ALTOPS, and not
self-refereed.

1. **Every check is `mode bmc`.** PASS means "no counterexample within this check's configured
   depth", not that the property holds. There is no `mode prove` anywhere on the ladder.
2. **The whole ladder runs under `RISCV_FORMAL_ALTOPS`** (ADR-0010), so `insn_mul`, `insn_div`,
   `insn_rem` and their siblings say nothing whatever about the real multiplier or divider.
3. **`reg_ch0` is inconclusive** (ADR-0023) — a single depth-21 BMC query that does not return in
   a practical budget. It is the *one* check that ties RVFI's self-report back to `rtl/regfile.v`.
   Without it the other 81 establish that the core's story about itself is spec-consistent, not
   that the story is true. ADR-0032 measured exactly this hole: an injected extra architectural
   write was missed by the entire ladder when gated to fire past the BMC bound.
4. **`formal/equiv.sh` does not converge** (ADR-0020), so ADR-0006's guarantee that RVFI
   instrumentation does not perturb the core is still argued, not proven — and `6309b3e` added one
   more `ifdef RISCV_FORMAL` field for that argument to cover (`rvfi_shadow.trap`). *(Closed by
   ADR-0047: the guarantee is now proved structurally and `equiv.sh` is deleted.)*
5. **`formal/complete` still fails**, on no gate.
6. **riscv-formal ships no spec model for `ecall`/`ebreak`/`mret`/`csrr*` at the pin.** So the
   behaviour `6309b3e` actually added — the whole of M3's trap semantics — is checked against
   assertions *this repo wrote*, in `test/asm/trap.S`, `test/csr_tb.v`, `test/decoder_tb.v` and
   `rtl/decoder.v`'s component proof. That is worth having and it is not an oracle. An empty
   baseline is loudest precisely where the ladder is quietest.

### 2. The M2 criterion is rewritten as a conjunction, and the empty baseline is demoted to one term of it.

`CLAUDE.md`'s M2 row now requires **all six** of the following, and says explicitly that the first
one alone means nothing:

1. `formal/EXPECTED_FAIL` empty **and** `formal/EXPECTED_CHECKS` matching — *met at `6309b3e`*;
2. the mul/div checks run without `RISCV_FORMAL_ALTOPS`, or ADR-0010's gap is closed by a named
   oracle that does;
3. `reg_ch0` returns a verdict rather than exhausting its budget;
4. `formal/equiv.sh` converges, or ADR-0006's non-perturbation guarantee is proven another way —
   **discharged by [ADR-0047](0047-non-perturbation-is-proved-structurally-and-equiv-sh-is-retired.md)
   on the second clause.** `equiv.sh` is deleted, not fixed: measured, `equiv_make` leaves 459 of 495
   `$equiv` cells unproven because it matches by name and the two builds optimise to differently-named
   netlists, and `equiv_induct` then diverges at ~660k clauses per step. The mechanism that discharges
   this term is `make -C formal nonperturbation` (`formal/check-nonperturbation.py`), a structural
   check that the `-D RISCV_FORMAL` build with its `rvfi_*` ports deleted sweeps to a netlist
   identical to the plain build. Both failure directions demonstrated on real mutations;
5. `formal/complete` passes, or every check it declines has a recorded reason;
6. ~~the nightly can go red (see §4) and is green~~ — **rewritten by ADR-0050, which deleted the
   nightly.** Now: *every check the repo owns is on a gate that can fail, and that gate is green* —
   the ladder, `imemcheck`, `dmemcheck` and `cover` as steps of the required `formal` job whose exit
   status is the job's, with no `continue-on-error` anywhere in it, `complete` joining them when its
   exclusion set lands (term 5), and no graded command in a pipeline in a `run:` block. The intent
   is unchanged and was always the same one — the ladder's verdict must be observed by something
   automated that is capable of failing — but a required PR check that blocks a merge is a strictly
   better instrument than a scheduled job ADR-0022 itself described as not gating merges. **That is
   the third move of an M2 criterion; ADR-0050 asks ADR-0045's question about it explicitly rather
   than restating the term and moving on.**

**Necessary-but-not-sufficient conditions must be written as such.** The old wording called item 1
"the signal", which is the language of sufficiency for something that was only ever necessary.
ADR-0033 had already recorded the distinction in prose; the table did not carry it, and the table
is what gets read.

### 3. ADR-0028's `dmemcheck` claim does not hold. Struck.

ADR-0028 lists three ladder checks that indirectly guard the trapping-retire convention. The first
is wrong:

> **`dmemcheck` catches a trapping store that still reaches the bus.** `formal/dmemcheck.sv:61-68`
> builds its environment shadow from the **real** `mem_wstrb`/`mem_wdata`, while `rvfi_dmem_check`
> builds its shadow purely from `rvfi_mem_*`. A store that is architecturally suppressed but still
> strobes the bus desynchronises the two.

**The two shadows cannot desynchronise that way, because they are not independent.**
`rtl/accessor.v` builds the RVFI write shadow *from the same bus signals* `dmemcheck` samples:

```verilog
out.rvfi_mem_wmask <= is_store ? mem_wstrb    : 4'b0;
out.rvfi_mem_wdata <= is_store ? write_request : 32'b0;
```

A store that reaches the bus is therefore reported faithfully in `rvfi_mem_*` too, and the two
shadows agree. Two mutations were run against the real ladder: dropping the store-flag suppression
(both shadows agree, no desync exists) and forcing `rvfi_mem_wmask <= 0` while the bus strobes —
ADR-0028's literal scenario. Both left `dmemcheck` **PASS** at 18 minutes.

The other two guards in that list (`reg` for a trapping `rd`, `pc_fwd`/`pc_bwd` for a dishonest
redirect) stand, with the standing caveat that `reg` is item 3 of §1 above and is inconclusive.

This matters beyond bookkeeping: ADR-0028's consequences section instructs the reader to check
future trap-reporting changes against those three guards by hand. One of the three does not exist.
A guard that is named but absent is worse than no guard, because it stops the next reader from
looking — `docs/THREAT_MODEL.md` calls this class out by name as a **prose-only guard** and rates
it a real finding. `rtl/writeback.v`'s comment at the drive site repeats the claim and is corrected
with it.

### 4. ADR-0022's central guarantee has never held. It holds now.

ADR-0022 records that replacing `make -C formal check || true` with an explicit baseline
comparison made "that comparison step's exit status the job's real signal". It did not. The
replacement step was:

```sh
formal/check-baseline.sh formal/checks formal/EXPECTED_FAIL | tee -a "$GITHUB_STEP_SUMMARY"
status=$?
```

A `run:` step without an explicit `shell:` key is `bash -e {0}` — **errexit but not pipefail**;
only an explicit `shell: bash` gets `-eo pipefail`. So `$?` was `tee`'s status, which is always 0,
and the nightly's gate could not go red no matter what the comparison found. Demonstrated on CI: a
deliberately wrong `formal/EXPECTED_FAIL` printed "Failure list does NOT match" and the job
reported success.

The `|| true` fix moved the defect rather than removing it. Both copies of the step (`ci.yml`,
`formal-nightly.yml`) now redirect to a file, take the status off the unpiped command, and exit on
it; both failure directions were demonstrated on real runs. **ADR-0022's conclusion stayed
accidentally true for its whole life** — the ladder happened to match its baseline every night —
which is precisely why nothing noticed.

The general rule, worth more than the instance: **`set -e` in a GitHub Actions `run:` block does
not imply `pipefail`. Never put the graded command in a pipeline.**

### 5. The `formal` job should join the required set. A human must do it.

ADR-0022's "no branch-protection entry" clause was scoped to the **nightly**, whose content is
unbounded — `complete`'s depth-50 walk and `equiv.sh`, neither of which has a wall-clock bound
since ADR-0031 retired `checks.cfg`'s `timeout=`. That reasoning does not transfer to the new PR
job, which runs neither.

Ratified: `formal` **should** be added to `main`'s required status checks. It is measured at
3m11s against a 20-minute job timeout, it is deterministic, and it closes a real hole — until it
existed, `formal/EXPECTED_FAIL` was a PR-modifiable baseline that only the nightly compared, so a
PR could widen the baseline and be contradicted the next morning with the change already on main.

Two costs, recorded rather than discovered later:

- it ties merges to **GitHub-hosted** capacity, deliberately, while `lint` and `test` have moved
  in-cluster. One ladder check peaks at 876 MiB RSS against a pod with roughly 700 MiB free, so a
  single check does not fit at any parallelism — `-j1` would not have saved it;
- there is still **no per-check wall bound**, so one non-converging check starves everything
  scheduled after it and `-k` does not help, because the job dies rather than the check. The
  20-minute job timeout is the only backstop, and raising any depth in `checks.cfg` re-opens it.

**This ADR does not change branch protection and no agent should.** It is a repository-settings
change for a human to make deliberately, exactly as ADR-0036 says of `lint`.

### 6. The iverilog leg was inert for the whole of M1, and the verification table did not know.

`rtl/decoder.v`'s hazard scoreboard called a `function automatic live_producer(r)` from two
continuous assigns. iverilog derives such an assign's sensitivity list from the **call's
arguments**, so a body that also reads module-level signals (`out`, `executor_out`,
`accessor_pending_*`) never re-evaluated when those changed. Under-sensitivity — the one direction
`CLAUDE.md`'s documented `sorry:` exception says is a real bug rather than harmless noise — and
iverilog emits no diagnostic for it whatsoever.

Reproduced on `main` before the fix: `hazard_rs1` went 0→1 at the first RAW hazard of the baked-in
program and **never fell again**; the PC advanced `0x0` → `0x4` and froze; `make waves` performed
**0 memory writes in 200 cycles** and issued 201 reads all to address `0x00000000`. After the fix,
the same program performs 22 writes with the PC advancing normally.

The window is exact and it is the whole of M1: `live_producer` was introduced by `a4662a2` — the
commit that *declared* M1 — and removed by `6309b3e`. yosys evaluates the function correctly, so
cxxrtl and the entire formal ladder were unaffected, which is why nothing caught it: **the leg the
verification table calls "the microscope" was reporting nothing, and nothing it reported was
wrong.**

M1's own criterion is "all RV32IM `.S` tests pass under cxxrtl", so M1 stands. What does not stand
is the three-leg claim in the verification table for that period, and `CLAUDE.md` now says so with
the dates. A leg that cannot fail is not a leg.

### 7. ADR-0034's attribution is right; its reproduction command does not run.

ADR-0034 corrected `CLAUDE.md`'s claim that the 20 `sorry:` notes come from `rtl/executor.v`, and
the correction is **confirmed**: the diagnostic is specific to `always_comb`, where sensitivity is
inferred, and `rtl/executor.v`'s `case (1'b1)` sits in an `always_ff`. Measured on merged main, all
20 notes are against `in[952:0]`, and `$bits` resolves the three candidate structs unambiguously —
`decoder_output` 943, `executor_output` 921, **`accessor_output` 953**, which is `rtl/writeback.v`'s
port type and the only one that matches. The count is exactly 20 both before and after `6309b3e`;
the width moves `in[951:0]`→`in[952:0]` for the single bit `rvfi_shadow.trap` adds. (Before the
change the same three were 955 / 920 / 952 — `decoder_output` loses 12 net, having shed
`csr_addr`+`is_csr_imm` and gained `trap`.)

Two things in that record are wrong and are fixed here:

- the command ADR-0034 prints, `iverilog -g2012 -s <mod> rtl/structs.v rtl/<mod>.v`, **fails
  preprocessing** — `Include file structs.v not found`. It needs `-I./rtl/`, and it needs the
  `RISCV_FORMAL*` macro set, without which every module reports 0 because the `always_comb` reads
  in question are inside `ifdef RISCV_FORMAL`;
- the width `in[311:0]` is the macro-less figure and does not match any real build. The build that
  produces the 20 is `make testbench.vvp`, where it is `in[951:0]`.

An attribution rule whose stated reproduction returns 0 is a prose-only guard of the same family
as §3.

### 8. Elaboration is not warning-free, and the note it pointed at did not exist.

`CLAUDE.md` said `yosys ... write_cxxrtl` "elaborates the current RTL cleanly with zero warnings".
It emits **one**: `Warning: Deep recursion in AST simplifier`. That warning is deliberately and
correctly allowlisted by the `elaborate` gate, which promotes every *other* `Warning:` to an error
— but the gate's own comment justifies the exception with "see `CLAUDE.md`'s technical notes", and
`CLAUDE.md` had no such note. A cross-reference to text that does not exist is the same defect
class as §3 and §7: a reader who goes looking cannot confirm the claim, so the exception reads as
undocumented. The note now exists and the count is stated as one.

## Consequences

- `CLAUDE.md`'s M2 row is a six-term conjunction. Closing M2 now requires deleting terms from a
  list, which is a burn-down with the same shape as `formal/EXPECTED_FAIL` itself.
- The empty baseline stays a real result and should not be talked down. It is item 1 of six, it
  was earned, and the attribution behind it held exactly — nine misalignment checks plus
  `ill_ch0` closed together while the three byte-granularity checks never moved.
- ADR-0028 loses one of its three indirect guards. Trap-reporting changes now have two, one of
  which is inconclusive. That argues for the `rtl/decoder.v` component proof carrying more of the
  weight, which is where `6309b3e` already put the `pc == mtvec` assertion the ladder cannot make.
- Nothing here changes `rtl/`. The RTL merged in this integration is unmodified.

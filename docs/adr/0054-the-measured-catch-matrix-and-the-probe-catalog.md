# ADR-0054: The measured catch matrix, and the probe catalog that outlives it

**Status:** Accepted · 2026-08-01 · *Deliverable of `docs/ideas/audit-the-oracle-stack.md`.
Measures ADR-0010, ADR-0032, ADR-0033, ADR-0049 and ADR-0051 rather than restating them.*

## Context

Five oracle surfaces run against this core — `make test`, `make test-units`, `make cosim-suite`,
the three component proofs, and the 85-check riscv-formal ladder — and until this ADR **nobody had
measured what any of them catches.** Every claim about coverage in this repo was either an argument
from construction (`RISCV_FORMAL_ALTOPS` hides the real multiplier, so the ladder cannot see it) or
a single anecdote (ADR-0032's `regs[31]` injection, ADR-0040's rs2-bypass probe, ADR-0049's four
executor mutations, ADR-0051's eleven). **Every one of those anecdotes found something real.**
Nobody had run them as a campaign.

ADR-0045 is the cautionary case, and it is recent: it closed an M2 term by *naming* an oracle, and
ADR-0049 measured that the named oracle was blind to three of the four defects it was named for.
**A term does not close on an assertion that an oracle exists, only on a demonstration of what it
catches** — ADR-0051's rule. This ADR applies that rule to all five surfaces at once.

## Method

**29 mutants, one per defect class, each run against all five surfaces.** Every cell is a real
invocation: the mutant is applied, the surface's own command is run to completion, the verdict is
read off that command's exit status and its graded output, and the tree is restored and verified
byte-identical before the next mutant. **No cell is inferred from another cell.**

- **Mutation protocol** (the brief's decision 7). One mutant at a time; each is a single anchored
  text substitution whose anchor is asserted to occur exactly once in the file; the driver asserts
  the substitution produced a real diff before running anything, and asserts the restored `rtl/` is
  byte-identical to a pristine snapshot afterwards. **No mutant appears in any commit on any
  branch**: the campaign runs in throwaway copies of the worktree outside git entirely, so there is
  no index for a `git add -A` to sweep. Two sibling tickets in this sprint hit exactly that hazard;
  taking the tree out of git removes the mechanism rather than relying on discipline.
- **No checked-in mutation framework** (the brief's decision 6). The driver, the mutant table and
  the workdirs are scratch. **What is committed is this ADR and the probe catalog in it.**
- **The surfaces are run as shipped.** No flag changed, no depth raised, no check weakened, no
  baseline edited — ADR-0010 and ADR-0025 forbid explaining a missed mutant that way, and a matrix
  produced under adjusted settings would measure the adjustment.

**The ladder cell is graded differently from the other four, and the difference matters.** A
`CAUGHT` verdict needs one non-PASS check, so the ladder runs in batches in a priority order that
puts the checks whose spec model covers the mutated instruction first, and stops at the first batch
that produces a red. **A `MISSED` verdict is recorded only after all 85 generated checks have run**
— which is what makes it a statement about the ladder rather than about a subset. The inventory is
`formal/checks/*.sby`, regenerated per mutant, and every cell records how many `status` files that
run actually wrote: ADR-0040's staleness defect is fixed, but the brief asked for it to be
re-checked per cell and it is cheap to keep checking.

**`make test` runs `make test-units`** (`Makefile`: `test: sim test-units`), so the two columns are
**not independent**: a mutant caught only by a unit bench is red in both, and a naive reading of the
`make test` column would credit the `.S` suite with every catch the benches make. **This is the
difference between a coverage map and a table of correlated greens**, so every `make test` cell is
attributed from that run's own log rather than from its exit status: the `.S` suite caught it iff
`test/run_tests.sh` itself reported a set-equality failure against `test/EXPECTED_FAIL` or
`test/OBSERVED_FLOOR`; otherwise `make` aborted in `test-units` and the bench is the catcher.

**Two measurement hazards were hit during the campaign and are recorded because both are cheap to
repeat.** First, **a status-file count read while `sby` is still running is not a result.** An
interim read of the baseline ladder showed 84 status files against 85 generated checks and was
briefly written down as though one check had not run — it was `csrw_mscratch_ch0`, the deepest check
on the ladder (`[depth]` 30), still executing. Had that reading stood, every "missed by the ladder"
cell graded against it would have been suspect. The driver never reads a status until the `sby`
process it launched has exited, and a `MISSED` ladder verdict additionally requires all 85 checks to
have run; the mistake was in a hand-typed progress check, which is exactly where this class of error
lives.

Second, **the campaign was briefly run ten workers wide on a ten-core machine also carrying other
work**, at a sustained load average near 60. That is not merely rude, it is a measurement hazard:
**a check starved of CPU and a check that genuinely does not converge look identical from the
outside**, so a contention artifact could enter the matrix as a finding — the exact failure this
campaign exists to detect in other people's work. The campaign was throttled to at most two
concurrent measurements, and every cell recorded before the throttle was audited against the
question *did this verdict depend on a wall clock?*:

- **`make test` and `make test-units` — no.** `test/run_tests.sh` grades each program against
  `CYCLES=5000`, a budget in **simulated** cycles, which no amount of contention can move; the unit
  benches run to `$finish`. One recorded cell (`G3`) does carry a `TIMEOUT`, `jal.S TIMEOUT
  retires=4996` — a program that genuinely never reaches `tohost` because the mutant broke the
  fetch window, and a deterministic result on any machine.
- **The driver's own 1800 s subprocess guard — no.** Exceeding it raises rather than returning, so a
  starved run **crashes the driver and records nothing**. Every row in the results file is a command
  that ran to completion and returned an exit status.
- Verdicts that could in principle depend on a wall clock — `sby`'s, on the ladder and the component
  proofs — had **no cells recorded at the time of the audit**, so none is at risk.

**No cell required an `UNMEASURED-UNDER-LOAD` mark.** Wall-clock times are nevertheless omitted from
the matrix: verdicts are what a coverage map is made of, and times taken under contention are
decoration that invites false comparison.

**A worked example of why, because a wrong number nearly entered this ADR.** A ladder wall time of
~46 minutes was reported during the campaign and was very nearly written down. It disagrees with
every other measurement of the same command: CLAUDE.md records 310 s, CI has measured 174 s and
217 s, and two independent runs this same day measured 287-315 s. The 46-minute figure was taken at
load 48-65 and is a measurement of the contention, not of the ladder. **The rule this ADR adopts:
any wall time recorded here is taken on a quiet machine and states the load average it was taken
at**, and a timing that disagrees with the repo's existing figures by an order of magnitude is
treated as a measurement of the environment until proven otherwise. A number with no load beside it
is not a measurement of the thing it names.

**Third, and the one worth carrying forward as a tooling fact: killing an `sby` does not kill its
solver.** `sby` shells out to `btormc` (`bash -c 'cd <check>; btormc … model/design_btor.btor'`),
and the solver is not in the killed process's tree in any way the kill reaches — so an interrupted
ladder run leaves a `btormc` per in-flight check grinding at 100% of a core, invisible in the sby
logs because it has already stopped writing to them. **This is the shape that makes it dangerous:
a `btormc` inside a k-induction step produces no output for minutes at a time, so "the workdir has
had no writes" and "the process is dead" look identical.** An interrupted-ladder cleanup in this
repo must check for the process, not for its output:

```sh
pgrep -fl 'btormc|sby'        # what is actually running
```

**Fourth — and this one produced three false cells before it was caught: three `rtl/` files are not
prerequisites of the simulator, so mutating one leaves `make` reporting the binary up to date.**
`Makefile:286` builds `test/rtl.cc` from nine RTL files, and `rtl/littlesoc.v`, `rtl/memory.v` and
`rtl/imemory.v` are not among them. Mutating `rtl/littlesoc.v` therefore produces
`make: 'sim' is up to date.` and the run measures **whatever binary was already on disk** — which,
in a workdir reused from an earlier mutant, was a different mutant's core. `E1` and `E2` came back
**caught by the `.S` suite and by co-simulation with all 56 programs red**, which is not a plausible
signature for a change to a module the simulator never compiles.

The brief's own instruction covers half of this: *if a mutant appears to be caught by nothing
including the controls, suspect your setup*. **The inverse deserves identical suspicion and is
easier to wave through, because a wall of red reads as a working oracle** — a green row invites the
question "should something have caught this?", and a red row invites no question at all. That
asymmetry is this repo's recurring failure mode stated in one line, and it is the more useful half
of the rule. The six cells were dropped rather than reinterpreted, the driver now deletes
`test/rtl.cc`, `sim` and `cosim` before every simulator-based surface so the build always comes from
the sources actually present, and the cells were re-measured from a clean workdir.

**The underlying fact is not a harness quirk. It is the coverage result for those three files, and
it is a stronger statement than "untested".** `rtl/littlesoc.v`, `rtl/memory.v` and `rtl/imemory.v`
are **untestable by the `.S` suite and by co-simulation** — not merely unexercised by the programs
that happen to exist. They are outside the simulator's dependency graph, so no `.S` program and no
co-simulation run *can* reach them, however many are added. A reader who sees three `missed` cells
and reaches for "write more tests" has misread the row: **more programs cannot close it.** Only
putting those files on a build that an oracle observes can, and `rtl/littlesoc.v` is not even
elaborated by CI's `elaborate` job (`.github/workflows/ci.yml` names the same nine files
`Makefile:286` does). The matrix marks these cells `n/a — untestable` rather than `missed` for
exactly that reason.

**Fifth, and the one that nearly put an invented finding at the top of this table: a `missed` cell
is ambiguous, and nothing in the campaign's design resolved it.** `missed` means either

- **(a)** no oracle watches this behaviour — a coverage gap, a finding; or
- **(b)** the mutation changes nothing architecturally observable — an **equivalent mutant**,
  evidence of nothing at all.

The driver cannot tell these apart: it applies a diff, runs a command, reads an exit status. **A
campaign that cannot distinguish them will manufacture blind spots**, which is worse than missing
real ones, because it sends the next reader hunting for oracles that are not needed and erodes trust
in the rows that *are* real.

`H1` was such a mutant and it was written up as the most alarming row in the table before being
caught. **The rule this ADR adopts: no `missed` cell is a finding until the mutant's architectural
effect is named** — a register value, a memory word, a trap, a retire count, a PC — together with
where in principle it could be seen. If no such effect can be named, the row is reclassified
`equivalent` and contributes nothing. **Any mutant caught by at least one surface is observable by
construction**, so the analysis is only needed for the caught-by-nothing rows; that is the whole of
the "observability" section below.

The distinction also tells you *whose* run you are looking at, which matters on a shared machine.
`formal/checks/Makefile` — what `make -C formal check` drives — invokes `sby <name>.sby`, with no
`-f` and no path, alongside GNU make's `--jobserver-helper`. Anything invoking `sby -f <path>` is
somebody's hand-rolled sweep. **This ADR's own first cleanup got the reasoning wrong** — it
concluded "no orphans survived" from an absence of file writes, which is not evidence about a
process at all. Re-checked with `pgrep`, the conclusion happened to hold; the argument for it did
not. It is the same error as inferring a matrix cell instead of measuring one, committed by the
campaign whose entire purpose is to stop doing that, which is why it is written down here rather
than quietly corrected.

**Controls are the integrity check.** A matrix in which nothing is caught indicts the harness rather
than flattering the core, so three mutants are included that *must* be caught: a wrong store lane
(the `sb` byte strobe stops shifting to the addressed byte), a sign flip (`SRA` becomes a logical
shift), and an off-by-one in `ADD`. Had any been missed by every surface the campaign would have
halted and that would be the top finding.

## The SHA every cell carries, and what landed after it

**Every cell in this matrix was measured at `021ad7f`.** Two grading defects were found and fixed by
concurrent work *after* that commit, and both touch machinery this campaign runs through. Per-cell
SHA discipline exists for exactly this case, so the exposure is stated rather than inherited:

1. **`formal/check-baseline.sh` read an unreadable baseline as an empty one** — `set -u` with no
   `-e` and no `pipefail`, and `-f` tested without `-r`, so a failing `sed` yielded the empty string,
   which matches an all-passing ladder. **No cell here is exposed**, and not by luck: this ADR's
   ladder column is computed by reading `formal/checks/<name>/status` directly and comparing against
   `formal/EXPECTED_FAIL` parsed in the driver. It never invokes `check-baseline.sh`. The one run
   that did — an unmutated baseline ladder — was interrupted and is not a cell.
2. **`test/cxxrtl.cc`'s exit 5 (`TRAP-TO-ZERO`) was reachable only outside its own scenario**: a
   fault in the first instructions of `_start` retires nothing, and the silence gate turned exit 5
   into exit 6, labelling it `MONITOR-SILENT`. **One cell's log carries the label — `G3`, the fetch
   window mutant, where 92 programs reported `TRAP-TO-ZERO`.** The fix changes which label a
   failing program gets; it does not change whether the program fails, and `test/run_tests.sh`
   grades on set equality against `test/EXPECTED_FAIL`, which a non-`PASS` status breaks under
   either spelling. **`G3`'s verdict is `caught` before and after.** No verdict in this matrix
   moves; a re-run at the fixed commit would relabel, not re-grade.

The general rule this illustrates: **a matrix cell is a measurement of a commit, not of a repo.**
Re-deriving one against a later tree is cheap — apply the mutant, run the surface — and the SHA on
each row is what tells a reader whether they need to.

## Decision

1. **The catch matrix below is this repo's coverage map**, and it claims only what was measured on
   `021ad7f`. Each cell records mutant, surface, command, SHA and verdict. It supersedes any prose
   claim about what a surface covers where the two disagree.
2. **A blind spot is a finding only when it has a named liveness probe.** The probe catalog gives
   one per blind spot: one line of diff, the command, the expected red. A blind spot with no probe
   is a bullet, and bullets in this repo rot — ADR-0037 counted five that outlived their own fixes.
3. **The matrix is not re-run on CI and no mutation framework is checked in.** CI cannot afford 29
   ladder runs; the durable output is the probe catalog, which costs one line and one command each.
4. **A change that claims to close a blind spot names the probe it makes red.** ADR-0051's rule,
   restated as a requirement on this map.
5. **No `rtl/` file changed in this ADR's PR.** The campaign is measurement; none of the 29 mutants
   is a defect that exists on `main`.

## The matrix

Every cell: mutant · surface · command · SHA `021ad7f` · verdict. **No cell inferred from another.**
Commands, one per column:

| column | command |
|---|---|
| `.S` suite | `make sim && ./test/run_tests.sh ./sim test/asm test/EXPECTED_FAIL test/OBSERVED_FLOOR` |
| benches | `make test-units` |
| co-sim | `make cosim-suite` |
| proofs | `make -C formal components_decoder`, `_executor`, `_pcloop`, run serially |
| ladder | `make -C formal checks` then `sby -f checks/<name>.sby` per generated check |

**Three verdicts are not `missed`, and the distinction is load-bearing.** `n/a — untestable` marks a
cell where the surface **cannot** reach the mutated file, so no amount of additional test material
would change it. `equivalent` marks a mutant with **no architectural effect at all** — every oracle
is correct to miss it and the row is evidence of nothing (finding 3). Only `missed` on a mutant with
a named architectural effect is a coverage gap.

| mutant | class | what it breaks | `.S` suite | benches | co-sim | proofs | ladder |
|---|---|---|---|---|---|---|---|
| `CTRL-1` | control | wrong store lane: sb strobe never shifts to the addressed byte | **caught** | missed | **caught** | missed | *not measured* |
| `CTRL-2` | control | sign flip: SRA becomes a logical right shift | **caught** | **caught** | **caught** | missed | *not measured* |
| `CTRL-3` | control | off-by-one in ADD | **caught** | missed | **caught** | missed | *not measured* |
| `A1` | altops-muldiv | multiply high half masked to zero (MULH/MULHU/MULHSU return 0) | **caught** | **caught** | **caught** | **caught** | *not measured* |
| `A2` | altops-muldiv | MULHSU treats rs2 as signed (sign-enable swap) | **caught** | **caught** | **caught** | **caught** | *not measured* |
| `A3` | altops-muldiv | signed DIV sign-restore deleted (ADR-0012's wrapper) | **caught** | **caught** | **caught** | **caught** | *not measured* |
| `A4` | altops-muldiv | divide-by-zero quotient returns 0 instead of all-ones | **caught** | **caught** | **caught** | missed | *not measured* |
| `B1` | past-bmc-bound | extra architectural write to x31 after cycle 40 (ADR-0032's probe) | missed | missed | **caught** | missed | *not measured* |
| `B2` | past-bmc-bound | spurious bus write to 0x10F00 after cycle 40, absent from the RVFI report | missed | missed | missed | missed | missed |
| `C1` | no-spec-model | ECALL raises cause 9 (S-mode ecall) instead of 11 | **caught** | **caught** | **caught** | missed | *not measured* |
| `C2` | no-spec-model | mret does not restore MIE from MPIE | missed | **caught** | missed | missed | *not measured* |
| `C3` | no-spec-model | mepc WARL mask drops the bit-0 clear | **caught** | **caught** | **caught** | missed | *not measured* |
| `C4` | no-spec-model | csrrs/csrrc with rs1==x0 still write the CSR (Zicsr suppression lost) | **caught** | **caught** | **caught** | missed | *not measured* |
| `C5` | no-spec-model | misa drops its C bit (0x4000_1104 -> 0x4000_1100) | **caught** | **caught** | **caught** | missed | *not measured* |
| `D1` | rvfi-report | store RVFI payload blanked (rvfi_mem_wdata always 0) | **caught** | missed | missed | missed | *not measured* |
| `D2` | rvfi-report | load RVFI read mask blanked (rvfi_mem_rmask always 0) | **caught** | missed | missed | missed | *not measured* |
| `E1` | unpathed-module | out-of-range PC aliases back into ROM (the guard deleted) | missed | missed | missed | missed | *not measured* |
| `E2` | unpathed-module | memory.v out-of-range read returns a constant instead of mem_wdata | missed | missed | missed | missed | *not measured* |
| `E3` | unpathed-module | memory.v drops the top byte of every word store | missed | **caught** | missed | missed | *not measured* |
| `F1` | stall-protocol | operand_stall drops its rs2 term (invariant 9 half-broken) | **caught** | missed | **caught** | missed | *not measured* |
| `F2` | stall-protocol | scoreboard loses the accessor's pending-load slot (invariant 8b) | **caught** | missed | **caught** | missed | *not measured* |
| `F3` | stall-protocol | CSR drain predicate loses accessor_out_valid (invariant 8c) | missed | missed | missed | missed | missed |
| `F4` | stall-protocol | rs2 write-through bypass deleted (ADR-0040's reg_ch0 liveness probe) | **caught** | **caught** | **caught** | missed | *not measured* |
| `G1` | decode-trap-fetch | halfword load misalignment no longer detected | missed | missed | missed | missed | missed |
| `G2` | decode-trap-fetch | a write to a read-only CSR is no longer illegal (ADR-0005 rule 2) | **caught** | **caught** | **caught** | missed | *not measured* |
| `G3` | decode-trap-fetch | fetch window's second word comes from the wrong address (+8) | **caught** | missed | **caught** | missed | *not measured* |
| `G4` | decode-trap-fetch | trap_epc records pc+4 instead of the faulting pc | **caught** | **caught** | **caught** | missed | *not measured* |
| `H1` | retire-counters | x0 write no longer suppressed at writeback | *equivalent* | *equivalent* | *equivalent* | *equivalent* | *equivalent* |
| `H2` | retire-counters | minstret counts trapping issues too (ADR-0027 broken) | **caught** | **caught** | **caught** | **caught** | *not measured* |

## What the matrix says

**1. All three controls are caught, so the matrix is measuring something.** `CTRL-1` (wrong store
lane) by the `.S` suite and co-sim, `CTRL-2` (sign flip) by all three simulation surfaces, `CTRL-3`
(ADD off-by-one) by the `.S` suite and co-sim. Had any been missed everywhere the campaign would
have halted and that would be this ADR's only finding.

**2. The component proofs are by far the narrowest surface: 4 of 29.** They catch `A1`, `A2`, `A3`
(the multiply and signed-divide mutants ADR-0051 decomposed the proof for) and `H2` (`minstret`'s
non-trapping rule, `rtl/decoder.v`'s own assertion). **They miss all three controls.** That is not a
defect — after ADR-0051 the executor proof asserts the multiply and divide invariants, not the
single-cycle ALU ops, and `CTRL-2`'s `SRA` is simply not in its assertion set. It is worth having
measured, because "the component proofs pass" is a sentence that reads like broad coverage and is
not.

**3. Observability: which caught-by-nothing rows are findings, and which is an equivalent mutant.**
Applying the bar above to every row that no surface caught:

| mutant | architectural effect | verdict |
|---|---|---|
| `B2` | writes `0xdeadbeef` into RAM word `0x10F00`. `test/testbench.v:77-82` commits any in-range write with a set strobe, and `RAM_BASE + 0xF00` is in range (`RAM_BASE = 0x0001_0000`, `RAM_WORDS = 1024`). A real memory word differs, observable by any load from it | **GAP** |
| `F3` | a CSR instruction issues one cycle early when a store is in the accessor, so `minstret` — which increments at issue — is read at a different count. The effect is the *value written to `rd`* by a `csrr minstret`, ordinary architectural register state | **GAP** (see the caveat below) |
| `G1` | a misaligned `lh` completes instead of trapping: no `mcause = 4`, no redirect to `mtvec`, and `rd` takes a loaded value. A trap that should fire and does not | **GAP** |
| `H1` | **none.** See below | **EQUIVALENT** |

**`H1` is an equivalent mutant, not a blind spot, and this ADR nearly shipped it as its headline.**
The mutation makes `rtl/writeback.v`'s `wen` high for instructions whose `rd` is `x0`. `wen` has
exactly one consumer — `rtl/regfile.v`, via `rtl/littlecpu.v:242` — and two independent suppressions
sit downstream of it, **neither touched by the mutation**:

```systemverilog
if (wen && waddr != 5'd0) begin        // rtl/regfile.v:48 — the array write is
  regs_a[waddr] <= wdata;              // still suppressed here, so regs_a[0]
  regs_b[waddr] <= wdata;              // is never written at all
end
...
reg_rs1 = (rs1 == 5'd0) ? 32'b0 : …    // rtl/regfile.v:66-67 — and x0 reads as
reg_rs2 = (rs2 == 5'd0) ? 32'b0 : …    // zero unconditionally regardless
```

The only other effect is on `read_a`/`read_b`, whose contaminated value is masked by the same read
path. **No architectural difference exists, so every oracle is *correct* to miss it** and "x0 write
suppression is unchecked" would have been a false claim. The line is redundant with the regfile's
own guard — a defence-in-depth fact worth knowing, and not a coverage gap. `make cosim-suite`
agreeing 56/56 while comparing the real `regs_a` array is the empirical half of the same statement.

**The `F3` caveat, stated rather than glossed:** its architectural effect is named and its
observation point is named, but **no witness was constructed** — no program in the suite issues a
`csrr minstret` with a store in flight, and this campaign did not write one. So `F3` is a gap
under the bar above, but one degree weaker than `B2` and `G1`, whose effects follow from the
memory model and the trap spec directly. Recorded as such rather than promoted.

**4a. `insn_lh_ch0` passes a defect the `.S` suite catches in seconds.** Chasing `G1`'s ladder cell
produced the campaign's sharpest single result. `formal/riscv-formal/checks/rvfi_insn_check.sv:198`
does assert `spec_trap == trap`, and with `RISCV_FORMAL_ALIGNED_MEM` defined
(`formal/checks.cfg:545`) `insn_lh`'s spec model does compute
`spec_trap = ((addr & (2-1)) != 0) || !misa_ok` — so the check *should* catch `G1`. It does not. A
liveness probe on the same check, in the style this ADR's catalog uses throughout:

| | |
|---|---|
| change | `rtl/accessor.v`: `1'b0: out.rd_data <= {{16{mem_rdata[15]}}, mem_rdata[15:0]};` → `{16'b0, …}` — `lh` stops sign-extending |
| command | `sby -f checks/insn_lh_ch0.sby` |
| expected | red |
| **measured** | **`PASS 0 10`** |
| materiality control | the same tree under `./test/run_tests.sh` → `sh.S MONITOR-ERROR 105`, failure list does not match `test/EXPECTED_FAIL` |

The mutation is unambiguously material — the `.S` suite catches it — and `insn_lh_ch0` is green
against it, against `G1`, and on the unmutated core. **A check that is green in all three states is
not distinguishing them.** All 70 `insn_*` checks run at one depth (`insn 15`, the whole `[depth]`
entry), and the most likely explanation is that no `lh` retire is reachable within 15 steps on this
pipeline — **but that is an inference, not a measurement**: proving it needs a `cover` statement
inside riscv-formal's checker, which ADR-0031 puts out of bounds for this ticket. What is measured
is the three PASSes and the materiality control.

This is not a reason to raise `insn`'s depth (ADR-0025 — depths are derived, not tuned to make a
result appear). It is a reason to audit per-check reachability across the `insn_*` family, which is
its own ticket and is scoped in the Consequences below.

**4. Three `rtl/` files are outside the simulator's dependency graph.** `rtl/littlesoc.v`,
`rtl/memory.v` and `rtl/imemory.v` are not among `Makefile:286`'s nine sources, so `E1`/`E2`/`E3`
are `n/a — untestable` for the `.S` suite and co-sim. Only `test/mem_tb.v` observes `rtl/memory.v`
at all, which is why `E3` is caught by the benches and by nothing else, and **`rtl/littlesoc.v` is
observed by nothing whatsoever** — it is not even elaborated by CI's `elaborate` job, which names
the same nine files.

**5. Single-surface catches are the fragile rows.** `C2` (`mret` drops the MIE restore) is caught
**only** by `test/csr_tb.v` — it passes all 56 `.S` programs and co-simulation. `E3` only by
`test/mem_tb.v`. `B1` (ADR-0032's extra architectural write past the BMC bound) **only** by
co-simulation, reproducing that ADR's result exactly on the current tree. Each of these is one
deleted bench away from being caught by nothing, which is what the probe catalog is for.

**6. Co-simulation is the strongest single simulation surface** — 18 of 29 — and it catches two
things nothing else does in its class (`B1`, and `CTRL-3` where the benches are silent). It is also
the one surface deliberately **not** on `make test`'s path and **not** in CI's required set
(ADR-0032, ADR-0039). This matrix is not an argument for changing that; it is a measurement of what
is being left out of the gate, and it should be read alongside ADR-0032's reasons for leaving it
there.


## The probe catalog

**This is the durable output of the campaign** (the brief's decision 6 — no mutation framework is
checked in, because CI cannot afford 29 ladder runs and the probes are what outlive the matrix).

Each probe is one line of diff, one command, and the red it must produce. They are written in the
style of ADR-0040's rs2-bypass probe, which is the one this repo already had and already trusted —
and which appears below as `P9`, re-measured rather than quoted. **Use one when you need to know
that an oracle is still alive**, before believing a green run from it under a changed
configuration.

Every probe below was **executed** during this campaign; the "expected" column is what it actually
produced at `021ad7f`, not what it ought to produce.

| probe | one-line change | command | expected red |
|---|---|---|---|
| **P1** the `.S` suite's per-retire monitor is live | `rtl/accessor.v`: `mem_wstrb = 4'b0001 << addr24;` → `4'b0001;` | `make test` | `sb.S MONITOR-ERROR 108`, 55/56, failure list does not match `test/EXPECTED_FAIL` |
| **P2** `exec_tb` covers the shifters | `rtl/executor.v`: `in.is_sra: … $signed(rs1) >>> rs2[4:0];` → `rs1 >> rs2[4:0];` | `make test-units` | `MISMATCH sra rs1=80000000 rs2=00000001 got=40000000 expected=c0000000` |
| **P3** co-simulation's architectural comparison is live | `rtl/executor.v`: `in.is_add: … rs1 + rs2;` → `rs1 + rs2 + 1;` | `make cosim-suite` | `0/56 agreed` |
| **P4** co-sim sees state no retiring instruction names | `rtl/regfile.v`: add `if (mut_cyc > 40 && wen) regs_a[31] <= wdata;` (with a cycle counter) | `make cosim-suite` | divergence list does not match `test/COSIM_EXPECTED_FAIL`; **`make test` stays 56/56** |
| **P5** `csr_tb` still drives the trap-entry / `mret` port | `rtl/csrs.v`: `mstatus_mie <= mstatus_mpie;` → `<= 1'b0;` | `make test-units` | `MISMATCH mret restores MIE from MPIE and sets MPIE: got=00001880 expected=00001888` |
| **P6** `mem_tb` still observes `rtl/memory.v` | `rtl/memory.v`: `if(mem_wstrb[3]) ram[…][31:24] <= …;` → `if(1'b0) …` | `make test-units` | `MISMATCH write-then-read same address: got=xxfef00d expected=cafef00d` |
| **P7** the decomposed multiply lemmas are live | `rtl/executor.v`: `out.rd_data <= multiply[63:32];` → `32'b0;` | `make -C formal components_executor` | non-`PASS` status |
| **P8** the signed-divide assertions are live (ADR-0051 F5) | `rtl/executor.v`: drop `op_sign_x != op_sign_y ? -mul_div_store[31:0] :` | `make -C formal components_executor` | non-`PASS` status |
| **P9** `components_decoder`'s `minstret` rule is live | `rtl/decoder.v`: `assign instret = committing;` → `= issuing;` | `make -C formal components_decoder` | non-`PASS` status |
| **P10** the fetch window is exercised by the suite | `rtl/fetcher.v`: `imem_addr2 = imem_addr + 4;` → `+ 8;` | `make test` | `bne.S FAIL`, `straddle.S FAIL`, `jal.S TIMEOUT` |
| **P11** the trap-cause path is graded | `rtl/decoder.v`: `CAUSE_ECALL_M = 32'd11;` → `32'd9;` | `make test` | `.S` suite red **and** `test-units` red |

### Blind spots with no probe, because nothing catches them

**These are the findings, and the honest form of them is that the probe column is empty.** A probe
asserts an oracle is alive; where no oracle observes the behaviour there is nothing to assert, and
inventing a probe against a check that does not exist would be the same error as ADR-0045's.

| blind spot | mutant that demonstrates it | what would have to exist |
|---|---|---|
| a bus write nothing reads back, absent from the RVFI report | `B2` | memory comparison in co-simulation — ADR-0041's owed work — or an unbounded `dmemcheck` |
| invariant 8(c): the CSR drain predicate's fourth slot | `F3` | a `minstret` assertion with a store in flight; no surface constructs that state |
| a trap that should fire and does not, for an operation no program performs | `G1` | a directed `.S` program doing a misaligned `lh`, or a ladder check for it |
| an `lh` retire reachable inside the `insn_*` depth | the `insn_lh_ch0` probe in finding 4a | per-check reachability evidence across the `insn_*` family; **not** a raised depth (ADR-0025) |
| `rtl/littlesoc.v` | `E1` | any build that elaborates it — CI's `elaborate` job names nine files and this is not one |
| `rtl/memory.v` beyond `mem_tb`'s two assertions | `E2` | ADR-0044's memory system, which replaces it |

**Do not close one of these by weakening a check or raising a depth** (ADR-0010, ADR-0025). Each row
is a hole in the oracle stack, and the fix is an oracle that observes the behaviour — not a
reinterpretation of one that does not.

## Consequences

*(filled in from measurement)*

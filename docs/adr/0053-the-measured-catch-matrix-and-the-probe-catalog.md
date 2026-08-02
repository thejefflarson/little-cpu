# ADR-0053: The measured catch matrix, and the probe catalog that outlives it

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

*(matrix and findings below; this ADR is written from measurement and is filled in as the campaign
completes)*

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

*(filled in from measurement)*

## The probe catalog

*(filled in from measurement)*

## Consequences

*(filled in from measurement)*

# Audit the oracle stack — measure what green means

**Status:** brief · 2026-08-01 · written against `0f561f9`

A deep code audit of this repo, designed around one measured fact: **the defects are not
where an audit would instinctively look.**

## Why the apparatus, not the RTL

Four defects were live on `0f561f9` when this brief was written. Two are proof-harness
defects, two are prose decay, and **none is a datapath bug**:

1. `rtl/executor.v:392-393` — `always_comb assume(in.rs1 <= 32'h0000000f)` and the same for
   `rs2`. An `always_comb` assume is **proof-global**: the comment says it serves the divide
   invariant, but it also caps the operands the *multiply* assertions see, so
   MULH/MULHU/MULHSU were asserting about zero high halves. Zeroing `multiply[63:32]`
   **passed**. This is the named oracle for M2 term 2, so the term-2 closure ADR-0045
   recorded rested on a broken oracle.
2. `formal/components_pcloop/status` = `FAIL 2 0` — a real red verdict no gate observes.
   `pcloop` is absent from `formal/Makefile`'s `all` and from both CI workflows.
   `formal/pcloop.sv:214`'s `f_may_stall` transcribes **four** stall reasons where ADR-0042
   made it five; the missing `operand_stall` is the plausible basecase counterexample.
3. `CLAUDE.md` lines 202/214/225/689 — false current-state claims, in the file whose header
   says to read it before believing anything else.
4. `.github/workflows/ci.yml:185-188` — a comment naming component tasks that no longer exist.

That distribution is not an accident. The RTL is ground daily by 52 self-checking programs,
an 82-check ladder, six unit benches and a 52/52 co-simulation. The verification apparatus —
roughly 6,000 lines of `.sv`/`.py`/`.sh`/`.cc`/`.v` against 3,152 lines of RTL — is checked by
nothing but itself, and the prose is checked by nobody.

The repo's own history says the same thing four more times, and every instance lived in the
layer that decides what "green" means: the `tee`/pipefail hole (ADR-0037), the re-grading
`make check` and the directory-mtime staleness (ADR-0040), and the sanitizer span that
mutation showed accepted every attack at exit 0.

**So the audit's centre of gravity is the verification stack first and the RTL second** —
justified by measurement, not by preference.

## Approaches considered

**A — bottom-up RTL sweep.** Read all 3,152 RTL lines against the ISA manual and the nine
invariants. ~2 weeks, most of it re-deriving what the ladder already proves per-instruction.
It would have found **zero of the four** known defects, and its output is prose annotation,
the form this repo has proven rots. Its one real strength — independent spec reading of the
no-oracle semantics — survives as phase 4 below.

**B — adversarial, oracle-first (mutation-led).** Audit the apparatus as the primary object:
inventory every runnable-vs-gated artifact, census every `assume`, vacuity-check every
`assert`, then run a systematic mutation campaign against all five oracle surfaces to produce
a measured catch matrix. RTL reading is then *targeted* at the cells nothing covers.

**C — spec-differential.** Extend co-simulation to CSRs and memory, generate randomized
instruction streams, fuzz against Sail. Requires real harness work before the first finding
(cosim compares the register trace only today — ADR-0041's owed work), and it cannot produce
the coverage map at all, because it adds a sixth oracle rather than measuring the five that
exist. Its cheapest slice — directed `.S` programs needing no new machinery — is absorbed as
phase 4b.

## Recommended: B

It is the only approach whose target matches the measured defect distribution; the only one
that produces the coverage map, which outlives the audit; its per-finding form is the repo's
native idiom, so findings cannot rot; and it needs zero new checked-in machinery. Every one
of its techniques is something this repo has already done once, by hand, successfully — the
rs2-bypass probe for `reg_ch0`, the `regs[31]` co-sim probe, the sanitizer mutations. B runs
them systematically instead of ad hoc.

## Reading order, with a hypothesis per target

An audit without hypotheses degenerates into re-reading code that looks fine.

1. **Gate inventory** (`formal/Makefile`, both CI workflows, `test/run_tests.sh`, `Makefile`).
   *Hypothesis:* `pcloop` is not the only runnable-but-ungated artifact — the class is "a
   verdict exists that no set-equality observes." Members to confirm: `make waves` (runs, but
   nothing is graded about it), and `formal/EXPECTED_FAIL`'s possible name-only matching —
   ADR-0036 recorded that a FAIL→ERROR flip keeps the ladder green and may never have been
   fixed. If so, that is a live defect of exactly the `pcloop` class.
2. **Every `assume` and every `assert` guard** in `formal/*.sv` and the `rtl/` FORMAL blocks.
   *Hypothesis:* more mis-scoped environment modelling of the `always_comb` class. For each:
   what structural fact does it model, where is that fact discharged (ADR-0017's rule), and
   does its scope leak onto assertions it was not written for. Companion vacuity check — for
   each `assert` with a compound guard, demonstrate the guard is reachable.
3. **The three non-local stall rules (invariant 8 a/b/c) and invariant 9**, read across
   `littlecpu.v` + `decoder.v` + `executor.v` + `accessor.v` as one system. *Hypothesis:* the
   repo itself labels these "true today only because X enforces it, breakable silently" — and
   `operand_stall` is the newest term in a protocol whose formal transcription demonstrably
   was not updated. Audit by mutation, not by reading alone.
4. **The no-oracle RTL**, read against the privileged spec *independently* — not against
   `csr_tb`'s expectations: `rtl/csrs.v` in full, `decoder.v`'s trap arm, `writeback.v`.
   *Hypothesis:* correlated-author error, since the only checks here assert what the same
   reading of the spec believed. **4b:** each suspicion becomes a directed `.S` program graded
   by the existing monitor and co-sim before it becomes a bug report.
5. **The grading scripts** (`run_tests.sh`, `run_cosim.sh`, `cosim.py`/`.cc`, `cxxrtl.cc`,
   `sanitize_monitor.py`, `check-baseline.sh`, `genchecks-audit.py`). *Hypothesis:* the
   green-without-testing class — four prior instances, every one in a script. Attention to
   exit-status propagation, pipelines in `run:` blocks, and any comparison whose failure path
   is untested.
6. **Prose, load-bearing claims only.** A claim is checked if acting on it wrongly would cost
   someone an afternoon; historical narrative is left alone.

**Skimmed or skipped, with reasons.** `fetcher.v`, `memory.v`, `regfile.v` — small, recently
reworked under heavy measurement (ADR-0042), covered by `imemcheck`/`regfile_tb`/`reg_ch0`
with a proven liveness probe; mutation cells only. `accessor.v`'s byte lanes, the ALU ops,
`decoder.v`'s immediate formats — exactly what 70 `insn_*` checks and the compressed suite
grind. `littlesoc.v` — outside every oracle, but also outside every execution path until the
ADR-0044 memory system exists; one matrix row recording "covered by nothing."

## Techniques, ranked by expected yield

1. **Mutation campaign → the catch matrix.** ~24 mutants, one per defect class, each run
   against `make test`, `make test-units`, `make cosim-suite`, the component proofs, and one
   fresh ladder run. **Controls matter**: mutants that *must* be caught (wrong store lane,
   sign flip) are included, because a matrix where nothing is caught indicts the harness
   rather than flattering the core.
2. **Assume census + vacuity audit.** Proven class, small population; each finding is either a
   live proof hole or a written discharge note at the site per ADR-0017.
3. **Gate inventory.** Cheapest per finding — defect 2 fell out of exactly this question.
4. **Prose-claim cross-check.** Mechanical, demonstrated yield.
5. **Independent spec reading + directed `.S` programs**, no-oracle zone only.

**Rejected:** instruction-stream fuzzing and the co-sim CSR/memory extension — new machinery
before the first finding; the latter is ADR-0041's owed work and belongs to its own ticket.
Any lint beyond what is gated — clean twice over.

## The coverage map

The principal output. Starting rows, known by construction:

- `RISCV_FORMAL_ALTOPS` hides real mul/div from all 70 `insn_*` checks
- every ladder check is bounded BMC — no `mode prove` anywhere on the ladder
- no spec model for `ecall`/`ebreak`/`mret`/`csrr*`, so those semantics are checked only by
  same-author assertions
- co-sim compares the register trace only — **a wrong store never loaded back is invisible to
  co-sim and to every `.S` test**, with bounded `dmemcheck` the only eye
- `rvfi_intr` = 0 with error 133 unreachable means the trap→handler *linkage* is
  monitor-unchecked
- the iverilog leg has no graded multi-program runner
- `littlesoc.v` and `memory.v` are covered by nothing

**The blank cells the audit discovers are its findings.**

## Decisions

1. The audit's object is the verification stack first, RTL second.
2. **Findings become executable artifacts, never bullets.** Trivial decay is fixed directly in
   small PRs; a confirmed defect that cannot be fixed inside the audit lands as a **red entry**
   in a name-and-status baseline or as a failing bench vector (ADR-0033: a known-red check on
   the ladder is the system working); each blind spot gets **one named liveness probe**
   documented in the rs2-bypass-probe style — one line, the command, the expected red.
3. **`pcloop` gets fixed and then gated** — `operand_stall` added to `f_may_stall`, then added
   to `formal/Makefile`'s `all` and CI's components job. A proof that can silently be red is
   worse than no proof (ADR-0022's reasoning). If the fix does not close the counterexample,
   the real counterexample is a finding and the red status goes into a graded baseline rather
   than staying invisible.
4. If the gate inventory confirms `formal/EXPECTED_FAIL` still matches names only, apply
   ADR-0035's name-and-status contract to it. ADR-0036 already made that decision; the audit
   executes it.
5. **The coverage map lives in one ADR**, each cell a command + SHA + verdict — measurements in
   the ADR-0038/0040 style, the one prose form this repo has shown does *not* rot, because it
   claims only what was measured on a named commit.
6. **No checked-in mutation framework.** The campaign runs from a scratch harness; what is
   committed is the probe catalog. A framework is machinery, CI cannot afford 24 ladder runs,
   and the durable value is the named probes. Revisit only if the matrix needs a full re-run
   more than once a quarter.
7. **Mutation protocol:** every mutant on a throwaway branch, one at a time, `git status` clean
   after each, never on `main`, never in a PR. Every matrix cell records SHA + command + verdict.
8. Audit PRs are small and single-purpose so they interleave with the M2 push without conflict.

## Risks

- **The audit finds an RTL defect the ladder should have caught.** That indicts a depth or an
  assume, and *both* findings get recorded — the bug and the hole. Do not weaken the check to
  explain it away (ADR-0010, ADR-0025).
- **An all-green matrix row for a class the oracles claim to cover** most likely means a mutant
  that synthesized away or an oracle run against a stale build. The controls detect this.
- **`pcloop`'s counterexample may be real** rather than the missing fifth stall reason.
  Genuinely open; both outcomes are findings.
- **Ladder rows can be falsely green until the depth re-derivation lands.** Run them last, or
  carry an explicit caveat.

## Sequence — ~7–9 working days, parallel to M2

| Phase | Work |
|---|---|
| 0 (0.5d) | Gate inventory; fix the prose and `ci.yml` comment in one small PR; confirm defect 1's fix landed; execute decision 4 |
| 1 (1d) | `pcloop` fix + gating |
| 2 (1–1.5d) | Assume census + vacuity audit |
| 3 (2–3d) | Mutation campaign, matrix, probe catalog; the coverage-map ADR drafted from measurements |
| 4 (1.5–2d) | No-oracle spec reading + directed `.S` programs, which become permanent suite members |
| 5 (1d) | Grading-script audit |
| 6 (0.5d) | Prose sweep completion; ADR finalized |

**Cut order if it runs long:** phase 6 breadth → phase 5 narrows to exit-status paths → phase 4
narrows to `csrs.v` → phase 3 shrinks to one probe per blind-spot class. **Never cut:** phases
0–2, and the matrix in some form — it is the deliverable.

## Deferred

Extending co-simulation to CSR/memory comparison (ADR-0041's owed work, its own ticket).
Instruction-stream fuzzing (worthwhile only after the comparison surface grows). A graded
multi-program iverilog runner — a real gap the matrix will document; building it is harness
work. Closing any blind spot the map names — that is the M2 burn-down, *informed by* the map,
not performed by the audit.

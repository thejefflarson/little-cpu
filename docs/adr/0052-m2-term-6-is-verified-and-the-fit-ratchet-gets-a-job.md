# ADR-0052: M2 term 6 is verified against the gate's own run, and the fit ratchet gets a job

**Status:** Accepted · 2026-08-02 · *Closes [ADR-0037](0037-an-empty-baseline-is-not-m2.md)'s M2
term 6 as rewritten by [ADR-0050](0050-the-nightly-is-deleted-and-its-checks-move-to-the-gate.md).
Amends [ADR-0038](0038-area-is-measured-in-logic-cells-and-two-levers-are-rejected.md) decision 1a
and [ADR-0042](0042-the-regfile-read-is-synchronous-and-costs-a-cycle.md)'s area number.*

## Context

Term 6 had two halves and only one of them had ever been checked. *"Can go red"* was fixed at
`1961234` and demonstrated on real runs. *"Is green"* was never verified — and then everything
underneath it moved: the ladder went 82 → 85 checks, `formal/EXPECTED_FAIL` gained a status field
(ADR-0036), four hand-authored sby targets stopped re-grading stale runs (ADR-0040), the components
job gained `pcloop` (ADR-0046), `complete` and `complete_cover` landed as hard gates (ADR-0050's
held-back piece, paid off with `formal/COMPLETE_EXCLUSIONS`), and then the mechanism the term
described was deleted outright.

ADR-0050 rewrote the term to:

> **Every check the repo owns is on a gate that can fail, and that gate is green.** Concretely: the
> ladder, `imemcheck`, `dmemcheck` and `cover` are steps of the required `formal` job whose exit
> status is the job's, **with no `continue-on-error` anywhere in it**; `complete` joins them when its
> exclusion set lands. No graded command sits in a pipeline in a `run:` block.

Nobody had confirmed that against merged `main`.

## What the verification found

**The term was not met.** The `formal` job carried a `continue-on-error: true` — on the step that
generates and runs the entire ladder — and it survived ADR-0050 because its comment argued the step's
exit status "is not the signal and never was". That argument is false in an interesting way:
`formal/Makefile`'s `check` target puts a leading `-` on the sby sub-make and then **ends in
`check-baseline`**, so the command's exit status already *is* the graded comparison's, byte for byte
the same verdict the next step recomputes.

What the suppression actually bought, therefore, was not "ignore sby's meaningless exit code" — the
Makefile had already done that. It was: **generation aborting, `genchecks-audit.py` dying, or the
disk filling up reach the step summary as a green step.** The gate step below would still catch those
(no `*.sby` files resolves to exit 2), so this was never an unsound gate — but it is precisely the
shape ADR-0047 and ADR-0050 both retired elsewhere, a suppressed step that reads as coverage, left in
the one job those ADRs were about.

Everything else the term asserts held. Verified on the gate's own run, not locally:

| step of the required `formal` job | outcome | wall |
|---|---|---|
| Set up OSS CAD Suite | success | 18s |
| Confirm the BMC toolchain is complete | success | <1s |
| Generate and run the ladder | success | **174s** |
| Gate on `EXPECTED_FAIL` + `EXPECTED_CHECKS` | success | 1s |
| `imemcheck` / `dmemcheck` / `cover` | success | 31s |
| `complete` (depth-50 whole-ISA walk) | success | 19s |
| `complete_cover` (anti-vacuity) | success | 13s |
| **job total** | **success** | **4m22s against `timeout-minutes: 20`** |

`main` at `d2e736e`, run `30724575535`. The ladder printed `85 checks: 85 pass, 0 fail`, *`Failure
list matches formal/EXPECTED_FAIL exactly (name and status)`* and *`Generated check set matches
formal/EXPECTED_CHECKS exactly (85 checks)`* — both set equalities, both directions, on the gate's
own run rather than reproduced from a workstation.

**Re-run against the changed job**, which is what makes the split steps a record rather than a
promise: same verdicts, each check its own step — ladder 217s, gate 0s, `imemcheck` 21s, `dmemcheck`
16s, `cover` 5s, `complete` 25s, `complete_cover` 17s, **job 5m15s**. The 4m22s / 5m15s spread
between two runs of the same ladder is solver and runner variance, not a change in the work.

**The pipe rule holds.** `grep -rnE '\|' .github/workflows/` over both remaining workflows returns
exactly one pipe on a graded command: `elaborate`'s `yosys ... 2>&1 | tee /tmp/yosys-elaborate.log`,
and that `run:` block opens with an explicit `set -euo pipefail`, which is the documented remedy
rather than an instance of the defect. Every other match is a `run: |` block scalar or a `$(... |
head -1)` inside an `echo`.

## Decisions

**1. Delete the `continue-on-error`, and make the gate step run on `!cancelled()`.** The two steps now
carry the same verdict and fail together; the second still publishes its report when the first goes
red, so nothing is lost by removing the suppression. `grep -c continue-on-error` across
`.github/workflows/` is now **0 in every file**.

**2. Split `imemcheck` / `dmemcheck` / `cover` into three steps.** Term 6 asks for each check's
outcome individually, and a three-`make` step structurally cannot supply it: `make` stops at the
first failure, so a red `imemcheck` leaves the other two **unrun** while the job shows one red step.
Split, the step list *is* the per-check record. Cost: nothing.

**3. `make fit` gets a job, and it is NOT required.** It has been declared a ratchet in CLAUDE.md and
implemented as one in the `Makefile` since ADR-0042, while `grep -rn fit .github/workflows/` returned
two prose matches about runner memory and no invocation — a ratchet nothing pulls. It runs on every
PR now.

**Non-required is the decision, not an oversight.** Area is a design constraint, not a correctness
one: a change that grows the core past its budget should be loud on the pull request and should be a
conversation, not an automatic block. Branch protection is a repository setting no pull request may
touch (ADR-0036), so promoting it is a human action taken deliberately or not at all. `fit` and
`nonperturbation` are now the two non-required jobs of eight.

**4. The area number is re-measured — and running it on CI is what found that the number is
toolchain-dependent, which nobody here had measured.** Same commit, same RTL; no `rtl/` file is
touched by this ADR.

| toolchain | logic cells |
|---|---|
| CI's pinned OSS CAD Suite, Yosys 0.66+179 (`e74db6dea`) — **the `fit` job** | **4208 / 5280, 79%** |
| local Homebrew Yosys 0.67+post (`b8e7da6f4`) | 4187 |
| quoted in this repo since ADR-0042 (also local) | 4236 |

**Quote 4208, and say which toolchain took it.** The pinned suite is the reference for the same
reason `formal/pin.mk` and `OSS_CAD_SUITE_SHA256` exist, and the `fit` job is where the number is
now taken. Every area figure this repo had on file was a local one, so the apparent 4236 → 4208
"reduction" is partly two toolchains being compared rather than cells being saved.

This is a **second axis of instability** on top of the one already recorded. CLAUDE.md's ±50 churn
floor was measured across *edits that synthesise to identical hardware*; this is 21 cells across
*yosys builds* at identical RTL. `FIT_MAX_LC` at 4400 sits 192 cells over the pinned measurement,
which clears both comfortably — but a ratchet set from a laptop would have been set against the
wrong number, and that is the practical consequence. Corrected in `CLAUDE.md`, the `Makefile`'s own
comment, `ci.yml` and the fit brief — with the provenance, not just the digits.

**This was found the only way it could be**: by running the thing rather than reasoning about it.
The ticket that produced this ADR noted a suspected 4236 → 4187 drift and asked for a re-measurement;
re-measuring locally would have confirmed the drift and recorded a third local number.

## A green job is evidence only if you can name what it would fail on

CLAUDE.md's rule, applied to the two things this ADR adds.

**The `fit` job.** Two probes, both run on this tree, neither inferred:

- `make fit FIT_MAX_LC=4100` → **exit 2**, `*** make fit: 4187 logic cells is over the 4100-cell
  budget.` The ratchet arithmetic fires. (Run locally, hence the local cell count; the arithmetic is
  the same one the job runs.)
- nextpnr printing no utilisation table → **exit 1**, `*** make fit: nextpnr printed no utilisation
  table, so NO measurement was taken. That is a failure, not a 0% fit.` This is the guard that keeps
  a green `fit` from meaning "the tool did not run": placement *always* fails here on an IO pad
  (231 `SB_IO` against sg48's 39, ADR-0038 decision 1a), so the exit status of nextpnr cannot be the
  signal and the presence of the table is.

**The `formal` job's steps.** Each has a demonstrated failure direction already on file rather than
newly built here, which is the honest statement: `1961234` demonstrated both directions of the
baseline gate on real runs; ADR-0036 measured the `btorsim`-absent ladder reporting green with ten
`ERROR`s, which the status field now catches; ADR-0040 records deleting the rs2 write-through bypass
as this repo's liveness probe for `reg_ch0`; ADR-0050 records `complete` red at
`DONE (FAIL, rc=2)` before its exclusion set; `complete_cover` is itself the anti-vacuity control for
`complete`. **What this ADR did not do is re-run all of those.** The one falsification it did run
against the changed file is the `fit` pair above.

## Consequences

- **Term 6 is met**, which makes all six M2 terms met. **That is not the same as declaring M2
  reached**, and this ADR deliberately does not: M2's own wording is *"re-proves everything the
  serialized core proved"*, and the two standing caveats are untouched — every ladder check is
  `mode bmc`, so PASS means no counterexample within a configured depth, and riscv-formal ships no
  spec model at all for `ecall`/`ebreak`/`mret`/`csrr*` at the pin. Declaring the milestone is a
  human call on evidence that is now complete rather than a consequence of this change.
- **The term moved three times and this is the fourth reading of it, not a fourth move.** ADR-0050
  said a fourth move should be treated as evidence the criterion describes nothing. This ADR changes
  no criterion; it checks the one that exists, finds it unmet by one line, and fixes the line.
- **`make cosim-suite` and `make waves` remain outside every gate**, and term 6's "every check the
  repo owns" should be read with ADR-0050's concrete clause, which is the formal checks. The co-sim's
  exclusion is decided (ADR-0032); `make waves` grading nothing is recorded under "what does not
  work" in CLAUDE.md. Neither is a regression this term introduced, and neither is closed here.
- **The `formal` job's real cost is now a number**: 4m22s of a 20-minute timeout, ~78% headroom, with
  the ladder at 174s of it. `formal/checks.cfg`'s note that the nightly's `timeout-minutes: 300` is
  "still the backstop" was stale in the direction that matters — the backstop is 20 minutes, and a
  non-converging check now takes the four hand-authored tasks down with it. Corrected in place.
- **Stale nightly references are struck where a reader would act on them** — `formal/Makefile`,
  `formal/checks.cfg`, `formal/check-baseline.sh`, `formal/EXPECTED_FAIL`, `docs/THREAT_MODEL.md`.
  Historical ADR text is left alone; it is history and reads as such.

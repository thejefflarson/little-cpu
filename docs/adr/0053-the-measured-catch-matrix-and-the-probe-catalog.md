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
**not independent**: a mutant caught only by a unit bench is red in both. Where that happens the
matrix says so, because "the `.S` suite caught it" and "a bench inside the same target caught it"
are different facts about coverage.

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

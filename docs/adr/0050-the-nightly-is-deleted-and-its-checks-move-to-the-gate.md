# ADR-0050: The formal nightly is deleted and its checks move to the PR gate

**Status:** Accepted · 2026-08-01 · *Supersedes [ADR-0022](0022-the-formal-nightly-reports-against-an-explicit-baseline.md).
Rewrites [ADR-0037](0037-an-empty-baseline-is-not-m2.md)'s M2 term 6.*

## Context

`formal-nightly.yml` existed for two reasons. Both expired this week.

**Reason one: `equiv.sh` took an hour.** Its `timeout 3600` accounted for nearly all of every
nightly's ~1h04m runtime, and no PR gate can carry that. ADR-0047 deleted `equiv.sh` and replaced it
with a nine-second structural check that now runs on every PR.

**Reason two: the ladder was not on the PR gate.** `1961234` put it there — the commit is titled
*"Run the riscv-formal ladder on the PR gate, not only in the nightly."*

What remained was a scheduled job whose only unique content is four hand-authored tasks, measured on
this tree at roughly three minutes:

| task | verdict |
|---|---|
| `imemcheck` | PASS |
| `dmemcheck` | PASS |
| `cover` | PASS |
| `complete` | **RED** — `DONE (FAIL, rc=2)`, `failed assertion ... at complete.sv:113.7-113.39 step 5` |

Everything else it did was a second copy of the ladder the gate already runs.

## And it was broken, in a way that is worth recording

The last nightly (`30694052608`, 2026-08-01T09:35Z) **failed**, with one annotation:

> The self-hosted runner lost communication with the server … starves it for CPU/Memory

That is verbatim the failure mode `ci.yml` documents as the reason the **PR-gate** `formal` job was
moved off `little-cpu-runners`: pod `memory.max` 1.5 GiB, 836 MiB already resident at job start, one
check peaking at 876 MiB, with the recorded conclusion that *"a single check does not fit in what the
pod has left, at any parallelism — `-j1` would not have saved it."*

**The nightly never got that fix.** It still ran in-cluster with `JOBS` unpinned, so `nproc` gave it
twelve-way parallelism inside a cgroup that cannot hold one check. Its own header claimed the two
ladder invocations were kept identical *"so the nightly cannot start grading a different ladder than
the gate"* — they had not been identical since `1961234`.

**And the current file had never executed.** Its last edit was `18d17a2` at 12:23; the last scheduled
run was 09:35 the same day. The workflow that retired `equiv.sh` — removing the hour that justified
the whole job — was never once observed running.

So a red nightly told us nothing about the ladder: the runner died before the graded comparison step
executed at all, and `continue-on-error` on the ladder step cannot help when the job loses its runner.

## Decision

**Delete `formal-nightly.yml`. Move `imemcheck`, `dmemcheck` and `cover` into the PR gate's `formal`
job as hard gates, with no `continue-on-error`.**

`imemcheck` is the check CLAUDE.md names as the guard on ADR-0003's dual-word fetch window. It ran
once a day on a job that died. It now runs on every pull request, and its exit status is the job's.

**`complete` does not land yet, and the reason matters.** It is red on this tree, and `formal` is a
required check, so adding it now would block every merge in the repo for a defect already recorded as
M2 term 5. It lands ungated in the *same change* that declares its exclusion set — MISC-MEM and
SYSTEM, measured to PASS at depth 50 in 45 s with those excluded.

The nightly's `continue-on-error: true` on `complete` was never wrong because the task was red. It was
wrong because suppression made a check that had never produced a verdict **indistinguishable from a
working control** — precisely what ADR-0047 retired `equiv.sh` for. Landing it red-and-suppressed here
would import that defect into the gate; landing it red-and-gating would import an outage. Neither is
the deal, so it waits, and its red stays on file rather than in CI.

## What is lost, stated rather than waved away

A scheduled job re-runs an unchanged `main`, which catches two things a PR-only world never sees:
**solver nondeterminism**, and **drift in the runner image's toolchain**. Both are real.

They are not worth a 300-minute timeout, a second hand-maintained copy of the graded step that had
already diverged, and a job that has been failing on infrastructure. If either becomes a live concern,
the answer is a small scheduled re-run of `main` whose *only* purpose is drift detection — which is a
different thing from what this workflow had become, and should be built as that rather than recovered
from this.

## ADR-0037's M2 term 6, rewritten

Term 6 read: **"The nightly can go red, and is green."**

There is no nightly. The term is now:

> **6. Every check the repo owns is on a gate that can fail, and that gate is green.** Concretely:
> the ladder, `imemcheck`, `dmemcheck` and `cover` are steps of the required `formal` job whose exit
> status is the job's, with no `continue-on-error` anywhere in it; `complete` joins them when its
> exclusion set lands (term 5). No graded command sits in a pipeline in a `run:` block.

**This is the third time an M2 criterion has moved, and ADR-0045's closing line says a third should
prompt asking whether the criterion describes anything real.** So, asked and answered:

Term 6's *intent* has been stable throughout — **the ladder's verdict must be observed by something
automated that is capable of failing.** That intent survives this change untouched. What changed is
the mechanism, and it changed in the strengthening direction: a required PR check that blocks a merge
is a strictly better instrument than a non-gating scheduled job, which is why ADR-0022's original
framing (*"none of this gates merges"*) was always the weaker guarantee.

The two earlier moves were different in kind and it is worth being precise about that. ADR-0037's
rewrite of the whole milestone was a **correction** — the criterion had been satisfiable without the
milestone. ADR-0045's amendments to terms 2 and 4 **took clauses ADR-0037 had written for exactly that
case**. This one **replaces a mechanism that no longer exists**. None of the three is an erosion, but
the count is now three, and a fourth should be treated as evidence the criterion is not describing
anything.

## Consequences

- **`imemcheck`, `dmemcheck` and `cover` gain their first gate.** Previously each ran once a day and
  nothing was graded against them; a red went to a job that had been failing on infrastructure.
- **The `formal` job grows by roughly three minutes**, from ~5 to ~8, against a 20-minute timeout.
- **The four hand-authored depths are still underived.** ADR-0046 re-derived every depth in
  `checks.cfg` and explicitly nothing outside it; `imemcheck.sby` and `dmemcheck.sby` carry hand-set
  depth 15, `cover.sby` 100, `complete.sby` 50, all set before ADR-0042 added the operand-fetch stall
  that moved F. Putting them on a gate makes deriving them *more* urgent, not less — a green that has
  stopped asking is now a green that blocks nothing while appearing to.
- **`dependabot-automerge.yml`'s comment names four required checks where the API returns six**; it is
  stale prose on a workflow whose only real dependency is that a required set exists. Corrected here
  rather than left for a ticket.
- **Nothing about branch protection changes.** `formal` was already required; this ADR adds steps to
  it and removes a workflow that was never in the required set.

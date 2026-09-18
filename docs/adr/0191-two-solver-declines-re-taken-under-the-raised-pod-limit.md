# ADR-0191: Two solver declines, re-taken under the raised pod limit

**Status:** Accepted · 2026-09-18

## What was true, and why it needed a re-take

Two solver choices for `formal/components.sby`'s component proofs were declined against a CI pod
memory ceiling of 1.5Gi, raised to 3Gi on 2026-09-06 when little-cpu rejoined the shared
`little-cpu-runners` ARC scale set and started running yosys/iverilog/sby inside the runner pod
itself rather than shelling out to buildkit (`cluster` repo, `charts/actions/runners/values.yaml`).
The CPU limit is 4 (also raised on 2026-09-06, from 3). Neither decline was ever written up as an
ADR — both live only in PR bodies (#281, #320) — so this is the first formal record as well as the
re-take.

- **`traps` on bitwuzla**, PR #320: 22s against 28s locally, a ~21% win, left unshipped "citing an
  OOM-kill precedent" set when PR #281 found `rIC3` using 1937-2259 MB against the *old* ~2Gi pod.
  Nothing about `traps` itself ever measured that high — its own peak was 295 MB solo (this
  branch's own instrumentation, below) — so the decline borrowed a ceiling that belonged to a
  different task and a since-superseded limit.
- **`rIC3` for the component proofs**, PR #281: measured on `executor`, the heaviest proof, at
  107s/1937 MB (`btor rIC3`) and 122s/2259 MB (`aiger rIC3`) against bitwuzla's 101s/779-847 MB,
  and declined because 2259 MB was an OOM risk against the pod's *then* ~2Gi ceiling.

CLAUDE.md's own rule for this class of question: "a machine with spare cores and a CI pod saturated
at its quota are different instruments" — a solver swap that reads faster solo can read slower once
it is one job among the dozen-plus that a real PR triggers on the same two physical nodes. So this
is measured as part of three ordinary, full CI runs on this branch, never a trimmed or solo
trigger, each configuration in its own job so `/sys/fs/cgroup/memory.peak` (cumulative for the
pod's whole life) is not carrying over another configuration's allocation.

## What's measured, and where

`components-proof` is a six-job matrix (`.github/workflows/ci.yml`), one self-hosted pod per proof
— `traps` and `executor` never share a pod with each other, so a change to one's engine cannot
pollute the other's peak reading even inside the same push. Every job now prints
`wall time: <N>s, pod peak memory: <bytes>` (added by this branch, permanently — the previous step
only wrote it to `$GITHUB_STEP_SUMMARY`, which `gh`'s job-log API cannot read back, so every future
re-take needs the number in the log itself, not just the summary). Confirmed against the cluster
repo: `runnerCpuLimit: "4"`, `memory: { limit: 3Gi, request: 1Gi }`
(`charts/actions/runners/values.yaml`, raised from 3/1.5Gi on 2026-09-06).

Three full CI runs on this branch, 2026-09-16/2026-09-18, `little-cpu-runners`, cgroup v2
(`memory.max: 3221225472`, `cpu.max: 400000 100000`, confirming the 3Gi/4-CPU pod), with every other
CI job (elaborate, test, cosim, lint, fit, soc-timing, ecp5-timing, formal-checks shards,
mutation-check shards, nonperturbation, monitor-freshness — 20+ jobs) running concurrently on the
same two physical nodes, which is the real CI concurrency the rule above asks for. `traps` and
`executor` each got their own job/pod on every run, so a change to one's engine never shares a
`memory.peak` reading with the other's:

| configuration | proof | wall | pod peak |
|---|---|---|---|
| shipped (`smtbmc`) | traps | 166s | 300 MB |
| shipped (`smtbmc bitwuzla`) | executor | 216s, 234s (two runs) | 741 MB, 724 MB |
| candidate (`smtbmc bitwuzla`) | traps | **138s** | **508 MB** |
| candidate (`btor rIC3`) | executor | **crashed at ~722s** (contended window, see below) | not recorded — pod lost |

## Verdict

**`traps` on bitwuzla: the decline is reversed. Ship it.** 166s → 138s, a 16.9% win measured as
one job inside a real, fully concurrent CI run — not the laptop-vs-pod gap CLAUDE.md warns about,
because the two figures are both pod figures, taken the same way, on the same class of run. Peak
memory rises 300 MB → 508 MB, which is 508 MB of 3072 MB, 2564 MB of headroom — nowhere near the
ceiling that PR #320 borrowed from a different task's `rIC3` figure against a since-superseded
1.5-2Gi limit. `formal/components.sby`'s `traps` line ships as `smtbmc bitwuzla`, matching
`executor`'s existing spelling; `formal/Makefile`'s `components_traps` target gains
`check-solver-bitwuzla` as a prerequisite, the same guard `components_executor` already carries.

**`rIC3` for `executor`: the decline stands, on the evidence this branch can actually separate.**
The `btor rIC3` run crashed: the `Executor arithmetic BMC` step ran roughly 12 minutes with no
result, the runner pod's logs never reached the server (`BlobNotFound`), and GitHub's own
annotation reads "The self-hosted runner lost communication with the server... Anything in your
workflow that... starves it for CPU/Memory... can cause this error." **That job ran 15:58-16:10 on
2026-09-18, and that window was independently saturated on the shared pool**: an unrelated PR's
`test` job on the same runners timed out at its own 25-minute limit spanning 15:50-16:16 that same
day, with no rIC3 and no solver of any kind involved, and other jobs in that window died or were
cancelled too. So "rIC3 exhausted the 3072 MB limit" and "the pool was starved by concurrent load
that had nothing to do with rIC3" are confounded by this one data point, and the crash cannot be
cleanly attributed to either cause alone — asserting the memory mechanism would be exactly the
inherited-conclusion mistake this ticket exists to correct, not a fix of it. What the evidence
*does* support, without needing to isolate the cause of the crash: `rIC3` measured no faster than
the shipped engine even before it crashed (107-122s locally against bitwuzla's own 101-122s, PR
#281), and the one CI attempt at it lost the pod. Between an engine with no measured speed
advantage and one observed pod loss on one side, and a shipped engine with no such loss on the
other, the decline stands on speed alone and the memory question is left open rather than answered.
`formal/components.sby`'s `executor` line is unchanged (`smtbmc bitwuzla`). A clean re-take —
confirming no other run is active on the pool first — would settle the memory question either way,
and is the natural next step if `rIC3` is ever reconsidered.

## What did not change

`formal/components.sby` ships with `traps` moved to `smtbmc bitwuzla` and every other proof's
engine — including `executor`'s `smtbmc bitwuzla` — exactly as it shipped before this branch. No
RTL moved; `make -C formal components_traps` and `components_executor` still pass locally. The
permanent addition is the log line above — every future re-take of this question reads it directly
rather than re-deriving wall time from job-log timestamps the way this one had to for its first
baseline run.

## Alternatives considered

- **Trust PR #281/#320's own local numbers.** Rejected: CLAUDE.md is explicit that a laptop with
  spare cores and a saturated CI pod are different instruments, and the pod's own limit has since
  moved, which is exactly the "measurement with a date on it" problem the rest of CLAUDE.md's
  fast/simple/readable/formally-verified habits warn about.
- **Race both engines in one task (drop the single-engine discipline PR #320 fixed).** Rejected —
  that is the exact bug PR #320 existed to remove: `sby` adds a bare engine line to whatever a task
  already names rather than replacing it, and two solver processes racing on a pod that is
  effectively one CPU under load cost more than either alone.
- **Ship `rIC3` for `executor` since 1937-2259 MB solo reads well under the new 3072 MB ceiling.**
  Rejected: it never measured faster than the shipped engine, and its one CI attempt lost the pod.
- **Assert that `rIC3`'s memory footprint caused the crash.** Rejected: the crash's window
  (15:58-16:10 on 2026-09-18) overlaps a shared-pool saturation episode with no rIC3 involvement
  (an unrelated PR's `test` job timed out at 15:50-16:16 the same day), so the mechanism is
  confounded with pool contention and this branch has no data point that isolates one from the
  other. Stating the memory mechanism as established would be the exact inherited-conclusion
  mistake this ticket exists to correct.

# ADR-0133: Fork PRs route off the self-hosted pool instead of skipping

**Status:** Accepted · 2026-08-24

## Context

`.github/workflows/ci.yml` runs eleven jobs — `elaborate`, `test`, `mutation-check`, `cosim`,
`components`, `lint`, `fit`, `soc-timing`, `ecp5-timing`, `nonperturbation`, `monitor-freshness` —
on `little-cpu-runners`, a self-hosted Kubernetes pool, triggered by `on: pull_request` with no
guard on where the PR's branch lives. The repository is public. Every one of those jobs executes
`make` recipes and shell scripts the PR head wrote — `make test`, `make lint-setup`, `make -C
formal ...` are not read-only.

GitHub grants a maintainer's approval of a first-time outside contributor's workflow run as a
**standing grant**: every later `pull_request` from that same contributor runs without asking
again. So approving one run to see whether an outside contributor's CI is green — the ordinary
reason to approve one at all — hands that contributor durable code execution inside the cluster.
The pods are `runAsNonRoot` with `allowPrivilegeEscalation: false` and fresh per job, which blocks
host takeover and cross-job persistence on the node. It does not block code execution inside the
pod, lateral network reach to whatever the pod can dial, exfiltration, or abuse of the
`actions/cache` entry that outlives a job and is shared across PRs. GitHub's own guidance is that
self-hosted runners and public repositories should not be combined for exactly this reason.

This gap is pre-existing — verified byte-for-byte identical to `main` before the recent PR swarm
(#223–#227) — and is not introduced by anything on this branch.

## Decision

**Route, do not skip.** Every job that ran on `little-cpu-runners` now picks its runner with:

```
${{ (github.event_name != 'pull_request' ||
     github.event.pull_request.head.repo.full_name == github.repository)
    && 'little-cpu-runners' || 'ubuntu-latest' }}
```

A `push` to `main` and a `pull_request` whose head repository is this one take the self-hosted
pool, unchanged. A `pull_request` from anywhere else — a fork — takes GitHub-hosted
`ubuntu-latest`, under the **same job name**. `formal` was already on `ubuntu-latest`, for an
unrelated reason (a single riscv-formal check peaks near 876 MiB resident against the self-hosted
pod's 1.5 GiB limit), and needed no change; it is the existing proof that the toolchain installs
and the jobs run on GitHub-hosted infrastructure at all.

Five jobs assert the RISC-V cross compiler is present via `.github/actions/verify-toolchain`
rather than installing it, because installing on the self-hosted pool is blocked by
`runAsNonRoot`. `ubuntu-latest` carries no such restriction and has no cross compiler baked in, so
each of those five jobs gets one added step, conditioned on the same fork check, that installs
`gcc-riscv64-unknown-elf` (and `clang` for the three jobs whose assertion also names `clang++`)
before the assertion runs. On the self-hosted pool the condition is false and the step is a no-op;
`verify-toolchain`'s own assertion is unchanged.

The expression is written out on all eleven `runs-on:` lines rather than factored through a shared
setup job. A setup job every one of the eleven depends on would make each of them wait on it, and
turns a single flaky or slow `needs:` evaluation into a stall across the whole workflow for no
benefit — the value computed is one string, from context available at every job's own evaluation
time with no side effect to share. The repeated expression is stated once, in a comment above
`jobs:`, and every job below it points back to that comment rather than restating the reasoning.

## Why not skip

A bare `if:` guard that skips these jobs on a fork PR was the shape first proposed and is wrong
for two reasons. First, an outside contributor gets no CI signal at all on their own PR — nothing
tells them whether their change elaborates, let alone passes. Second, and worse: a skipped job
that is also a **required** status check either blocks the PR from merging permanently (branch
protection waiting on a check that will never report) or, depending on how "require branches to be
up to date" and the skip semantics interact, is treated as satisfied — which is strictly worse than
the exposure this ADR closes, because it would make branch protection assert something CI never
checked.

Routing keeps the check name identical, so branch protection's required-check list needs no
change and a fork PR gets a real answer from a real run.

## What is verified

Read live rather than trusted from a comment
(`gh api repos/thejefflarson/little-cpu/branches/main/protection -q
'.required_status_checks.contexts'`), the required set is: `elaborate`, `test`, `components`,
`monitor-freshness`, `lint`, `formal`, `soc-timing`, `nonperturbation`, `cosim`, `mutation-check`,
`fit` — eleven checks, all still present after this change because every job kept its name.
`ecp5-timing` is the twelfth job on the self-hosted pool and is not in the required set; it is
routed the same way regardless, because the exposure this ADR closes has nothing to do with
whether a check is required — an unrequired job on the self-hosted pool is still code execution
inside the cluster.

The three cases the expression has to get right:

- **`push` to `main`.** `github.event.pull_request` does not exist on this trigger, so
  `github.event_name != 'pull_request'` is `true` and the left disjunct short-circuits the `||`
  before the right side — which would otherwise read a property off something absent — is
  evaluated. Self-hosted.
- **A same-repo `pull_request`.** Every PR in the recent swarm (#223–#227) pushed a
  `thejefflarson/jef-8xx-*` branch directly to this repository, not from a fork, so
  `github.event.pull_request.head.repo.full_name == github.repository` is `true` for all of them.
  Self-hosted — the fast pool is unchanged for the pattern this repo actually uses.
- **A fork `pull_request`.** Both disjuncts are `false` — the event is `pull_request` and the head
  repository differs — so the expression evaluates to `'ubuntu-latest'`.

`actionlint` passes on the edited workflow with no findings.

## What this does not close

**"Require approval for all external contributors"** is a repository setting under Actions →
General, not a line in `ci.yml`, and turns a first-time contributor's standing grant into a
per-run approval — the other half of GitHub's own guidance. No workflow file can set it; it is a
maintainer action tracked in `docs/THREAT_MODEL.md` rather than enforced by anything in this repo.

This ADR does not change what any job does once it runs, or what it can reach from either runner
class — only which runner class a fork PR's code reaches at all.

## Consequences

- Every required check still resolves under its existing name; branch protection needs no edit.
- A fork PR now gets a genuine CI signal, on GitHub-hosted infrastructure, for jobs that need a
  RISC-V cross compiler this repo does not otherwise install outside the self-hosted image.
- The self-hosted pool is reachable only from a `push` to `main` or a same-repo `pull_request` —
  never from a fork — which is the property this ADR exists to establish.
- `docs/THREAT_MODEL.md` names which runner class a fork PR can reach, so the boundary this ADR
  draws is legible to a future security review without re-deriving it from `ci.yml`.

## Alternatives considered

- **Skip on a fork PR.** Rejected — see "Why not skip" above.
- **A shared setup job whose output feeds `runs-on` on the other eleven.** Rejected: it adds a
  `needs:` edge from every job to one job for a value with no side effect to share, which is exactly
  the "job dependency that serialises the whole workflow" this repo's own habit of measuring before
  restructuring warns against paying for nothing.
- **Require every contributor's first PR to route through a bespoke low-trust workflow before any
  self-hosted job runs.** Rejected as unneeded complexity — GitHub's own per-run approval setting
  already does this, and duplicating it in-repo would be two controls for one property with no
  measured benefit to the second.

# ADR-0018: Dependabot auto-merge is re-armed, gated on required status checks

**Status:** Accepted · 2026-07-29 · **Supersedes [ADR-0016](0016-no-unreviewed-dependabot-automerge.md)**

## Context

ADR-0016 deleted `.github/workflows/dependabot-automerge.yml` because, at the time it was written,
auto-merge had nothing to merge *on*: `main` carried no required status checks, so
`gh pr merge --auto` merged immediately and the workflow's only gate was the PR author's name.
That made the auto-merged class of PR — the one rewriting the SHA pins of actions that execute in
CI — the least reviewed change in the repository.

ADR-0016 named two preconditions for re-arming. **Precondition 1 is now met.** The CI gate landed
in `c66527d` and has run green on `c66527d`, `d5a1ca5`, and PR #14, so required status checks were
configured on `main`:

```
checks: elaborate, test, components, monitor-freshness
strict: true
enforce_admins: true
```

The maintainer's standing workflow rule is *"if CI is green on a pull request, merge it — don't
second-guess, don't re-review."* With a real gate in place, auto-merge is that rule expressed as
automation rather than a bypass of it.

## Decision

**Restore `.github/workflows/dependabot-automerge.yml`, gated on the required status checks.**

Two changes from the pre-ADR-0016 version:

1. **`--squash`, not `--merge`.** `main` requires linear history, so a merge commit is rejected
   outright. The original file would have failed on every run — a latent break that predates the
   deletion and would only have surfaced now.
2. A header comment stating the dependency explicitly, with the verification command, and the
   instruction to delete the file if the required checks are ever removed.

**Precondition 2 — the `dependabot/fetch-metadata` semver-patch gate — is deliberately NOT
adopted.** See below.

## Rationale

Precondition 1 was the load-bearing one. Required status checks change auto-merge from "merge
immediately, unobserved" to "merge when four jobs prove the design elaborates, 47/47 tests pass,
the executor BMC holds, and the generated monitor is fresh." That is a genuine control.

Precondition 2 is rejected on three grounds:

- **It blocks correct work.** The `actions/checkout` 6.0.2 → 7.0.1 bump (PR #14) was a *major*
  version bump, was CI-green, and was correct — v7's only behavioural change (`block checking out
  fork PR for pull_request_target and workflow_run`) is a hardening that does not apply here, since
  every workflow in this repo uses plain `pull_request`. A patch-only filter would have forced a
  manual merge for exactly the routine case auto-merge exists to handle.
- **It protects less than it appears to.** A malicious release of a pinned action executes in CI
  when the PR *runs*, not when it merges. The merge gate controls persistence, not execution. The
  required checks do not change that either — but neither does a semver filter, so it buys little
  against the threat it was aimed at.
- **It adds trusted surface to answer a supply-chain finding.** `dependabot/fetch-metadata` is
  another third-party action running with the workflow's token.
- **It optimises against the wrong risk.** See "Staying current is the security posture" below.

### Staying current is the security posture

The obvious worry — *a compromised release merges without a human reading it* — is real but is the
**smaller** of the two risks on the table, and optimising against it makes the larger one worse.

Pinning old is not safe by default. A repo that merges dependency updates slowly accumulates
unpatched bugs and CVEs in the exact code that runs its CI, and that exposure is **certain and
cumulative**, where a malicious release is speculative. The `actions/checkout` 6→7 bump is the
concrete case: v7.0.0 blocks checking out fork PRs under `pull_request_target`/`workflow_run`, and
v7.0.1 escapes values passed to `--unset` — three hardening fixes that a conservative merge policy
would have deferred indefinitely. **Newer software with bugfixes beats older software.**

Dependabot's own timing does the work the semver filter was reaching for. GitHub applies a
**default 3-day cooldown to version updates**, so a release is not even proposed until it has been
public — and exposed to the rest of the ecosystem — for three days. That is the window in which
malicious or broken releases are typically yanked, and it costs nothing. `.github/dependabot.yml`
sets `cooldown: default-days: 3` explicitly rather than relying on the implicit default, so the
intent survives a change in GitHub's defaults. (Note `github-actions` supports `default-days` but
not the per-semver-bump variants, so a single value is all this ecosystem can express.)

Cooldown deliberately does **not** apply to security updates — those should land fast, and that
asymmetry is correct.

Stacked with the rest, the posture is: a release must survive three days of public exposure, then
pass four CI jobs proving the design elaborates, 47/47 tests pass, the executor BMC holds, and the
generated monitor is fresh. Every `uses:` is 40-char SHA-pinned, so a retag cannot silently change
what runs; `main` is force-push protected, so history stays auditable. That is a stronger position
than merging late by hand.

## Consequences

- Dependabot PRs merge unattended once CI is green. That is the intended behaviour.
- **The workflow is now coupled to a repo setting no file in this repo controls.** If required
  status checks are ever removed, this workflow silently reverts to a rubber stamp. The header
  comment says so and gives the verification command; ADR-0016's reasoning stands as the record of
  what that state looks like.
- `enforce_admins: true` means the maintainer also cannot merge past a red gate — deliberate, since
  a gate the owner routes around is not a gate.
- **The CI gate is now load-bearing in a way it was not before.** Every dependency update reaching
  `main` is vouched for by those four jobs and nothing else. Weakening them — dropping a required
  check, or letting one go red-and-ignored — silently weakens dependency review too.
- If this posture ever needs reversing, the cheapest response is to re-delete the workflow
  (ADR-0016's position), not to add a semver filter that would block the hardening releases this
  ADR exists to let through.

## Alternatives considered

- **Keep ADR-0016's deletion.** Rejected: its stated precondition is now met, and the maintainer
  explicitly wants auto-merge. Leaving a decision in force after its premise expires is how a repo
  accumulates rules nobody can justify.
- **Adopt the semver-patch gate as well.** Rejected on the three grounds above. Available as a
  one-step addition if the risk appetite changes.

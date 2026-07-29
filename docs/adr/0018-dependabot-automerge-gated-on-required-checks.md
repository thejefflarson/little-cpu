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

The residual risk is accepted and named: **a compromised patch release of a pinned action, whose
diff passes all four CI jobs, merges to `main` without a human reading it.** The mitigations are
that CI must pass, that `github-actions` is the only ecosystem (a handful of pins, all first-party
except `soundcheck-action`), that every `uses:` is 40-char SHA-pinned so a retag cannot silently
change what runs, and that `main` is protected against force-push so history is auditable after the
fact.

## Consequences

- Dependabot PRs merge unattended once CI is green. That is the intended behaviour.
- **The workflow is now coupled to a repo setting no file in this repo controls.** If required
  status checks are ever removed, this workflow silently reverts to a rubber stamp. The header
  comment says so and gives the verification command; ADR-0016's reasoning stands as the record of
  what that state looks like.
- `enforce_admins: true` means the maintainer also cannot merge past a red gate — deliberate, since
  a gate the owner routes around is not a gate.
- If the residual risk above ever stops being acceptable, the cheapest response is to re-delete this
  file (ADR-0016's posture), not to add the semver filter.

## Alternatives considered

- **Keep ADR-0016's deletion.** Rejected: its stated precondition is now met, and the maintainer
  explicitly wants auto-merge. Leaving a decision in force after its premise expires is how a repo
  accumulates rules nobody can justify.
- **Adopt the semver-patch gate as well.** Rejected on the three grounds above. Available as a
  one-step addition if the risk appetite changes.

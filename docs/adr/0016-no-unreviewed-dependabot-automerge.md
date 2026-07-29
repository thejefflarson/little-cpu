# ADR-0016: No unreviewed auto-merge for the actions dependabot updates

**Status:** Accepted · 2026-07-28 · *Constrains the JEF-608 CI gate*

## Context

`.github/workflows/dependabot-automerge.yml` has been in the repo since before the CI gate, and it
was **inert**: it fires on `pull_request` with `if: github.actor == 'dependabot[bot]'`, but no
`.github/dependabot.yml` existed, so dependabot never opened a PR and the workflow never ran.

JEF-608 adds `.github/dependabot.yml`. That single file **arms** the auto-merge workflow, and the
combination is worse than either half:

- The workflow's only gate is *who opened the PR*. There is no update-type filter, no path filter,
  and no check on what the diff actually contains.
- It holds `contents: write`.
- `main` has **no required status checks** (JEF-608's acceptance criterion 8 is a repo-settings
  change, deferred until the workflow has run green at least once). `gh pr merge --auto` with
  nothing required to wait for merges immediately.
- `github-actions` is the *only* declared ecosystem, because it is the only one this repo has. So
  the exact class of PR this would auto-merge unreviewed is the class that **rewrites the SHA pins
  of the actions that execute in CI** — the highest-trust artifact in the repository.

That is a complete path from "compromise or simply retag an upstream action" to "arbitrary code
runs in this repo's CI", with no human in it. ADR-0013 already established that this project treats
a pin as an enforced control rather than a comment; auto-merging pin rewrites would give that up on
a weekly cron.

## Decision

**Delete `.github/workflows/dependabot-automerge.yml`. Keep `.github/dependabot.yml`.**

Dependabot still opens the PRs — the update *notification* is the part with value, and it is fully
preserved. What is removed is merging them without a human reading the diff.

Re-arming auto-merge is permitted later, and requires **both** of:

1. Branch protection on `main` with required status checks configured (JEF-608 criterion 8), so
   that `--auto` has something to wait for; **and**
2. An update-type gate — a SHA-pinned `dependabot/fetch-metadata` step restricting the merge to
   `version-update:semver-patch`.

Until both hold, the file stays out. `.github/dependabot.yml` carries a comment pointing here so
the absence reads as a decision rather than an oversight.

## Rationale

Considered and rejected: **land the `fetch-metadata` + semver-patch gate now**. It narrows the
exposure but does not close it. A patch release of a pinned action is exactly the shape a
supply-chain attack takes, and with no required checks the merge would still be immediate and
unobserved. It also answers a supply-chain finding by adding a third-party action — more trusted
surface, not less.

Considered and rejected: **hold `.github/dependabot.yml` until criterion 8 lands.** That strands a
correct, useful config behind a repo-settings change no agent in this workflow can make, and leaves
the pins to rot in the meantime.

Deleting the workflow is the smallest change that actually fails closed, adds no dependency, and is
one `git revert` away from being undone once its preconditions are real. The cost is one human
click per weekly dependabot PR, on a repo with a single ecosystem and a handful of pinned actions.

## Consequences

- Dependabot PRs require review. On this repo that is a few PRs a month.
- The two preconditions above are a **human follow-up**: branch-protection required checks cannot be
  configured from inside the worktree, and must not be attempted autonomously.
- Bumping a pinned action still goes through the same review as any other change, which is what
  ADR-0013 asks for on the riscv-formal pin and is now consistent across both pin classes.
- The composite action's cache is keyed on the release SHA-256 rather than the tag for a related
  reason: checksum verification is skipped on a cache hit, so a tag-keyed entry could be restored
  without ever meeting the checksum. Content-addressing the key means a restored entry is one that
  was verified against that exact digest when it was written.

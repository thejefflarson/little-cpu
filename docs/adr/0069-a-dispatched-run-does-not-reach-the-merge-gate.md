# ADR-0069: A dispatched run does not reach the merge gate

**Status:** Accepted · 2026-08-03 · *Amends
[ADR-0013](0013-the-riscv-formal-pin-is-an-enforced-control.md), whose pin-bump workflow opened a
pull request that could never merge. No `rtl/` change ships from this ADR.*

## Context

`formal/bump-riscv-formal-pin.sh` opened a pull request with the Actions default `GITHUB_TOKEN` and
then ran `gh workflow run ci.yml --ref "$BRANCH"`. The dispatch was there because GitHub does not
fire `pull_request` for a PR that token opens, and `workflow_dispatch` was added to `ci.yml` for
exactly that call.

It does not work, and the way it fails is the part worth keeping: **the dispatched run is green
everywhere a human looks and invisible to the only place that decides.**

## What was measured

On a scratch branch cut from `main`, so **not** conflicting with it — the earlier observations of
this were on branches that were also conflicting, which independently suppresses `pull_request` CI
and had to be ruled out first.

1. Push the branch. `ci.yml`'s `push` trigger is `branches: [main]`, so nothing runs: the commit
   has **0** check runs.
2. `gh workflow run ci.yml --ref <branch>`. Nine check runs appear on that commit.
3. Open a pull request from it. Five seconds after opening: `statusCheckRollup` is **0** while
   those nine check runs are on the head commit. Ten seconds after opening: **11**, and grouping
   the rollup entries by the workflow run in their `detailsUrl` gives three run ids, **none of them
   the dispatched one**. The commit itself carries 20 check runs across four runs; the rollup
   carries 11 across three.

The pull request was `MERGEABLE` throughout, so the conflicting-branch explanation is out.
`gh pr checks` listed the rollup's eleven and read entirely green.

A second run measured the recursion guard directly, from a real workflow using the default token: it
opened a pull request and then pushed an empty commit to the same branch. **Neither SHA has a single
check run**, and the PR's rollup is 0. So GitHub suppresses `push` from that token as well as
`pull_request` — which rules out pushing an empty commit as a fix, no secret required or not.

## Decision

**1. Open the pull request with a token that is not the Actions default.**
`formal/propose-pin-bump.sh` reads `PIN_BUMP_TOKEN` and passes it to `gh pr create`. A PR opened by
any other identity fires `pull_request` normally and needs no dispatch.

**2. With no such secret, do not open a pull request at all.** The branch is pushed either way, so
the regenerated `test/monitor.v` and the pre-graded diff — the whole value of the automation — are
on disk. The script opens an issue naming that branch, linking a compare page, and saying which
secret makes future bumps open their own PR. A human opens the PR in one click and the checks run.

**3. Remove `workflow_dispatch` from `ci.yml`.** Its only stated purpose was the call above.
Leaving a trigger whose documented reason is measurably false invites the same misreading; a manual
re-run is `gh run rerun` on any past run.

`actions: write` comes off the pin-bump workflow with the dispatch, and `issues: write` goes on.

## What a human has to do

Add a repository secret named `PIN_BUMP_TOKEN` holding a fine-grained PAT or a GitHub App
installation token with contents and pull-requests write on this repository. Until then pin bumps
arrive as issues. That is a settings action and no workflow can take it.

## Evidence

`formal/test-propose-pin-bump.sh` drives the script against a stub `gh` and pins both modes: 14
checks, hermetic, on `make test` and so in CI's required `test` job. The stub refuses
`gh workflow run` outright and refuses `pr create` unless the supplied token reached it.

The red direction was run. Making the script take the pull-request arm unconditionally and dispatch
CI alongside it turns **10 of the 14 green**.

The token path itself was demonstrated end to end rather than argued: the pin pointed at an older
SHA in a throwaway clone, `formal/bump-riscv-formal-pin.sh` run with `PIN_BUMP_TOKEN` set, and the
pull request it opened reached a populated `statusCheckRollup` and a mergeable state. The
no-secret path was demonstrated on the real scheduled workflow, dispatched from the branch: it
pushed the branch and opened an issue, and opened no pull request. Every scratch branch, pull
request and issue those runs created was deleted.

## Consequences

- A pin bump can merge, or it says plainly that it cannot and why. It could do neither before.
- There is a repository setting the automation depends on, and it degrades to a slower path rather
  than to a broken one when the setting is absent.
- **`gh workflow run` is not a way to satisfy branch protection**, and that is now written where
  someone would reach for it: in `formal/propose-pin-bump.sh`'s header and as two checks in the
  test. The failure is silent in the direction that matters — the Actions tab is green.
- The pin-bump workflow still enables auto-merge nowhere. A pin bump is reviewed.
- The check that a proposal already exists reads open issues as well as open pull requests, so the
  weekly schedule does not stack up duplicates while a bump waits for a human.

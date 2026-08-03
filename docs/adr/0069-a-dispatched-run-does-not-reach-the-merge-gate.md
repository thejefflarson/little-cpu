# ADR-0069: A dispatched run does not reach the merge gate

**Status:** Accepted · 2026-08-02 · *Amends
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

**Propose the bump by issue, never by a pull request this workflow opens.** There is one code path
and no secret.

`formal/bump-riscv-formal-pin.sh` still does the work: bump `formal/pin.mk`, regenerate
`test/monitor.v` at the new pin, commit, push the branch. `formal/propose-pin-bump.sh` then opens an
issue naming that branch, linking a compare page that opens the pull request in one click, and
saying that a human has to be the one to open it. A PR opened under a human account fires
`pull_request` normally, so its checks count.

**Remove `workflow_dispatch` from `ci.yml`.** Its only stated purpose was the call above, and its
comment stated a mechanism that is measurably false. `gh run rerun` covers a manual re-run.

The workflow's permissions become exactly what the four operations need: `contents: write` for the
push, `issues: write` for `gh issue list` and `gh issue create`, and `pull-requests: read` for the
`gh pr list` that notices a bump a human has already opened. `actions: write` goes with the
dispatch.

## Rejected: a PAT or GitHub App token

A token that is not the Actions default would make a bot-opened PR fire `pull_request` and merge
normally. **This repository is public, and that settles it.** The pin-bump workflow carries
`workflow_dispatch`, so anyone with write access can run it at an arbitrary ref — and the script at
that ref would read the secret. A token with contents and pull-requests write, reachable that way,
buys one click.

The measurement above holds either way; what changes is only who opens the pull request.

## Evidence

`formal/test-propose-pin-bump.sh` drives the script against a stub `gh`: 13 checks, hermetic, on
`make test` and so in CI's required `test` job. The stub refuses `gh pr create` and
`gh workflow run` outright, and two checks per script assert that neither string appears outside a
comment.

The red direction was run twice. Replacing the issue with a bot-opened PR **and** a dispatch turns
**9 of the 13 red**; replacing it with the bot-opened PR alone — no dispatch, the subtler
mistake — turns **8 of the 13 red**.

The path was demonstrated end to end on the real scheduled workflow, dispatched at a branch whose
pin was one commit behind: it pushed the branch, opened an issue, and opened no pull request. A
second dispatch printed `an issue already proposes …; nothing to do`. Separately, a pull request
opened from such a branch under a human account reached `statusCheckRollup = 11`,
`mergeable = MERGEABLE` and `mergeStateStatus = CLEAN`, with all seven required checks green — which
is the whole claim about who opens it. Every scratch branch, pull request and issue those runs
created was deleted.

## Consequences

- A pin bump arrives as an issue with a one-click link, and the pull request a human opens from it
  merges normally. Before this it arrived as a pull request that could never merge.
- **CI does not run on the bump until a human opens the PR.** The weekly schedule notices upstream
  moving; it no longer even appears to grade it. That is a real reduction against the intent, and
  the intent never worked.
- The commit message carries both SHAs and the compare link, so `gh pr create --fill` produces a
  pull request body with them. The diffstat and the full diff under `checks/`, `insns/` and
  `monitor/` are in the issue rather than in the PR description.
- A branch whose issue nobody acts on stays behind: next week upstream has moved again, the script
  computes a new branch name and opens a second issue, and the first branch is left. Nothing
  deletes it.
- **`gh workflow run` is not a way to satisfy branch protection**, and neither is a bot-opened PR.
  Both are written where someone would reach for them: in `formal/propose-pin-bump.sh`'s header and
  as four checks in the test. The failure is silent in the direction that matters — the Actions tab
  is green.
- The pin-bump workflow enables auto-merge nowhere. A pin bump is reviewed.
- The check that a proposal already exists reads open issues as well as open pull requests, so the
  weekly schedule does not stack up duplicates while a bump waits for a human.

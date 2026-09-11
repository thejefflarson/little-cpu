# ADR-0176: The pin bump runs upstream code without a credential

**Status:** Accepted · 2026-09-10

## What was true

`.github/workflows/riscv-formal-pin-bump.yml` ran weekly with `contents: write` and
`issues: write`, and in one step cloned `YosysHQ/riscv-formal` at whatever `HEAD`
resolved to and ran `make test/monitor.v`. That target is

```
MONITOR_GEN = cd $(RISCV_FORMAL_DIR)/monitor && python3 generate.py -i rv32imc -c 1 -a -p monitor
```

— upstream's own Python, from the freshly cloned unreviewed commit, executing in the step
where `GH_TOKEN` sat in the environment. `actions/checkout` left the same token in
`.git/config` as an auth header, because `persist-credentials: false` was not set.

So one commit reaching upstream — a maintainer account takeover, a malicious pull request
merged there, a compromise of their own CI — obtained code execution on a runner holding a
token that could push branches and rewrite unprotected refs here. Nothing verified a
signature, nothing required the commit to be settled, and nothing separated "upstream
moved" from "upstream's code runs with write credentials".

The bump also decided *whether to run at all* from metadata anyone can write:

```
ISSUE_COUNT=$(gh issue list --state open --limit 200 --json title \
  --jq "[.[] | select(.title == \"$TITLE\")] | length")
```

This repository is public and upstream's `HEAD` is public, so the title
`Bump riscv-formal pin to <first 12 hex>` is computable by anyone. Opening one issue with
that exact title made every later run print `an issue already proposes ...` and exit 0, for
as long as the issue stayed open, behind a decoy indistinguishable from the workflow's own.
An open pull request whose head branch was `riscv-formal-pin/bump-<12-hex>` did the same,
and a fork's pull request satisfies that. `--limit 200` failed in the other direction too:
past two hundred open issues a real duplicate went unseen and a fresh issue opened weekly.

## The decision

**One trusted signal replaces two untrusted ones.** The duplicate check is now
`git ls-remote --exit-code --heads origin "$BRANCH"`. A branch on this repository requires
write access; a fork's branch lives on the fork and never appears here. No title matching,
no author allowlist to maintain, and no `$TITLE` interpolated into a jq program. If the
branch was merged and deleted, the pin has advanced and the equality check stops earlier.

**The credential never shares a process with upstream code.** The job is three steps and
`GH_TOKEN` is set per step rather than per job: `decide` (token, read-only), `regenerate`
(**no token**), `publish` (token). `persist-credentials: false` keeps a credential off disk
while the middle step runs. `formal/bump-riscv-formal-pin.sh` additionally refuses to start
if it finds `GH_TOKEN` set, so the property does not depend on the workflow alone.

**Upstream commits must be settled.** `PIN_BUMP_MIN_AGE_DAYS`, default 7, refuses a commit
younger than the floor. Before this, a commit pushed at 06:00 Monday was cloned, executed
and committed here at 06:17 the same morning.

**A branch is never published without its issue.** `git push` preceded `gh issue create`,
so a failed issue left a pushed branch and no issue — and because the branch name encodes
the SHA, every later run rediscovered it, called the bump already proposed, and stopped.
The pin would have gone stale in silence. Publish now takes the branch back down when the
issue cannot be opened.

## What grades it

`test/pin_bump_token_test.py`, on `make test`'s path, reads the workflow and fails if the
step that runs `bump-riscv-formal-pin.sh` carries `GH_TOKEN`, if `actions/checkout` does not
set `persist-credentials: false`, or if no step runs that script at all — the last so a
rename cannot quietly retire the check. Five `test/PROBES_EXPECTED` labels force each
direction red, including the missing-workflow case, and four more cover the two scripts'
argument and token guards.

## What this does not do

It does not verify upstream's signatures; the age floor is a delay, not authentication, and
buys time for a compromise to be noticed rather than preventing one. It does not sandbox the
generator — upstream code still runs on the runner, with whatever the runner has. And it
does not address the OSS CAD Suite setup action, which fetches a `latest` release and
interpolates the parsed tag and digest into a curl URL; that is a separate unpinned
third-party path.

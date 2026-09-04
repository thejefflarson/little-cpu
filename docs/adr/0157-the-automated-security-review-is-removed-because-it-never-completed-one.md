# ADR-0157: The automated security review is removed, because it never completed one

**Status:** Accepted · 2026-09-02

## Context

`.github/workflows/soundcheck.yml` ran `thejefflarson/soundcheck-action` on every pull
request, every push to `main`, and weekly. It was never a required status check, so it
never gated a merge; it existed to put a second reader on every diff.

It had been red on every open pull request from #240 through #251, including one whose
whole diff was `.gitattributes` and two markdown files. Two separate defects were found,
and the second is why this record exists.

## What was measured

**Defect 1: the job pushed with a read-only token.** The failing sub-step was `Commit and
push rewrites`, ending in `remote: Permission to thejefflarson/little-cpu.git denied to
github-actions[bot]` and a 403. The workflow granted `contents: read` on the strength of a
comment asserting that the pinned action's `autofix` input defaulted to `false`. At the
pinned SHA it defaults to `"true"`, "to match v1 behavior" per that action's own
`action.yml`. A downloaded `soundcheck-audit.json` artifact confirmed it, showing
`"autofix_requested": true` and a rewrite proposed for a single Low-severity finding on a
docs-only push. This was fixed on the branch — `autofix: "false"` set explicitly, and the
job gated on the action's own `exit-code` output instead of on push plumbing.

**Defect 2: the reviewer never reaches a verdict.** With defect 1 repaired, three pull-request
runs and one `workflow_dispatch` full-repository scan, all against a clean tree, each ended
the same way in under two seconds:

```
ERROR: claude exited with code 1:
Running security review on /home/runner/work/little-cpu/little-cpu (diff vs origin/main (1 files)) model=sonnet budget=$5.0...
```

The diagnostic is empty, in diff-scoped and full-repository mode alike. The fault is upstream,
in the SHA-pinned `soundcheck` / `soundcheck-action` repositories this project does not own;
the likeliest candidate is that action's own `@anthropic-ai/claude-code` npm pin predating
flags the pinned scripts now pass it.

## Decision

**Remove the workflow.** Repairing defect 1 alone would have replaced a check that is red on
every pull request with one that is green on every pull request and has never once looked at
the code.

This project's own rule is that a grader with no demonstrated red direction is not a grader.
The same argument runs the other way and is what decides this: **a check that cannot report a
finding is decoration**, and it is the more dangerous of the two failures, because a check
that is always red gets investigated and a check that is always green does not.

## What this costs, and what it does not

There is no automated per-pull-request security review on this repository. Nothing else was
wired to it — no other workflow, script or document references `soundcheck` — and because it
was never in branch protection's required set, no protection change was needed to remove it.
Security review remains available on demand as a developer command, run by whoever is working
on a change.

**This is reversible and cheaply so.** Restoring the file from git history is the whole of the
work; the reason it is not kept in the tree behind a disabled trigger is that a workflow nobody
runs decays exactly as silently as one that runs and says nothing, and git already keeps it.

## When to revisit

When the upstream action reaches a verdict on this repository. The test is one
`workflow_dispatch` run that reports a finding count — any count, including zero — rather than
exiting 1 with an empty diagnostic. **Restoring it must also re-settle `autofix` explicitly
rather than relying on the default**, which is the fact the stale comment got wrong and the
reason defect 1 outlived the assumption that produced it.

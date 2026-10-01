# ADR-0233: The weekly compare-product refresh proposes by issue

**Status:** Accepted · 2026-10-01 · *Applies ADR-0069 to `.github/workflows/compare-product-schedule.yml`.
No `rtl/` change ships from this ADR.*

## Context

The weekly re-take opened its refresh as a pull request with the Actions default `GITHUB_TOKEN`.
ADR-0069 measured that GitHub fires no `pull_request` or `push` event for what that token does, so
the pull request had no checks and could never merge. The workflow's `if: always()` upload also
mixed a `/tmp` path with a workspace path, and nothing stopped a second refresh from stacking up
while the first waited.

## Decision

- **Propose by issue.** `.github/scripts/publish-product-refresh.sh` commits `soc/compare/product.json`
  to `compare-product/refresh-<date>-<run id>`, pushes it, and opens an issue carrying the diff and a
  `compare/main...<branch>?expand=1` link. A person opens the pull request, which gets its checks.
  No PAT or App token is stored, for the reason ADR-0069 gives. The script is tracked, the way
  `formal/publish-pin-bump.sh` is (ADR-0176), so `test/compare_product_schedule_publish_test.py` runs
  it for real against a throwaway repository instead of parsing a step out of the workflow YAML.
- **One refresh at a time.** Before pushing, the script lists open issues titled
  `Refresh the cross-core product stamp...` and open pull requests whose head branch starts with
  `compare-product/refresh-`. Either one makes the run skip, say so in the step summary, and leave the
  new measurement in the uploaded artifact. It skips rather than updating the existing branch because
  a person may already be reviewing it.
- **The artifact comes from one directory and only on success.** Every intermediate file lives in
  `$RUNNER_TEMP/compare-product` (`OUT_DIR`), the upload takes that directory, and the upload runs on
  the default `success()` condition: a measurement that failed has no stamp worth keeping.
- **The issue is created last**, and a failed `gh issue create` deletes the pushed branch so a later
  run retries rather than leaving a branch nobody knows about.

## Consequences

- The workflow's permissions are `contents: write`, `issues: write` and `pull-requests: read`.
- A refresh branch whose issue nobody acts on blocks later refreshes until the issue is closed.
  That is the intent: one open proposal, not a stack.
- The script sits under `.github/scripts/`, not `soc/compare/`, because
  `soc/compare/product_diff.py --require-news` treats a change under `soc/compare/` as news and would
  make the stamp stale against its own publisher.

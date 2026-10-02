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

## Amendment: measure and publish are separate jobs

The workflow first ran `make compare-product` and the publish script in one job holding
`contents: write` and `issues: write`. A tool the measurement ran (the RISC-V gcc, the OSS CAD
Suite, `make`'s recipes) could rewrite the publish script or the stamp before the publish step ran
with `GH_TOKEN`, on a pool shared with other jobs. The workflow is now two jobs:

- **`measure`** has `contents: read` and runs every tool. It uploads only `product.json` and
  `product-diff.md` as the `compare-product-stamp` artifact, on success, and exports whether the
  stamp moved as a job output.
- **`publish`** (`needs: measure`, only when `moved == 'true'`) is the one job with write scopes
  (`contents: write`, `issues: write`, `pull-requests: read`). It checks out `main` fresh, downloads
  the artifact into a separate directory, copies `product.json` over the checked-out stamp and runs
  the script from that checkout. It needs only `git` and `gh`, so it runs on `ubuntu-latest`, off the
  self-hosted pool. The script's open-refresh guard, the concurrency group and the issue route are
  unchanged.
- **No global credential helper.** The workflow no longer runs `gh auth setup-git`, which writes a
  helper into the runner's global git config. The script passes
  `-c credential.helper= -c 'credential.helper=!gh auth git-credential'` on each push, so the
  credential exists only for that command.

What the split does not prevent: a compromised measurement can still write a hostile `product.json`
or diff into the artifact. The publish job commits only `soc/compare/product.json` to a branch and
puts the diff in an issue, and a person reads both before opening a pull request, which then gets
its own checks.

`test/compare_product_schedule_token_test.py` grades the split: no job holding a write scope runs
`make` or a toolchain setup action, only `publish` holds one, `measure` declares `contents: read`,
`publish` runs on `ubuntu-latest` and on `measure`'s output, and `gh auth setup-git` is absent.
`test/compare_product_schedule_publish_test.py` requires the push to carry the per-command helper.
A real dispatch has not run this shape; the first scheduled or manual run on `main` is its test.

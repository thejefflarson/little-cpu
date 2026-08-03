#!/bin/bash
# scratch: measures whether a PR opened with the default GITHUB_TOKEN, and a
# push made with it, fire pull_request-triggered workflows.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

BRANCH="scratch/token-probe-${GITHUB_RUN_ID}"
git config user.name "github-actions[bot]"
git config user.email "41898282+github-actions[bot]@users.noreply.github.com"
git checkout -b "$BRANCH"
date > TOKEN_PROBE.txt
git add TOKEN_PROBE.txt
git commit --quiet -m "token probe: open"
git push --quiet origin "$BRANCH"
echo "open-sha $(git rev-parse HEAD)"

gh pr create --title "scratch: token probe (delete me)" \
  --body "scratch" --head "$BRANCH" --base main

sleep 30
git commit --quiet --allow-empty -m "token probe: empty commit"
git push --quiet origin "$BRANCH"
echo "empty-sha $(git rev-parse HEAD)"
echo "branch $BRANCH"

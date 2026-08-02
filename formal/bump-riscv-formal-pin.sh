#!/bin/bash
# Opens a PR bumping formal/pin.mk's riscv-formal SHA when upstream's `main`
# has moved past it, regenerating test/monitor.v so monitor-freshness is a
# real verdict on the bump rather than a guaranteed failure. Does nothing --
# no branch, no commit, no PR -- if the pin is already current, or if a PR
# already proposes the SHA upstream is at.
#
# The existing gates (monitor-freshness, formal/check-genchecks.py,
# formal/check-complete-exclusions.py, the ladder itself) decide whether the
# bump is safe; this script only notices and proposes.
#
# Usage: formal/bump-riscv-formal-pin.sh
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

UPSTREAM_URL="https://github.com/YosysHQ/riscv-formal.git"

PIN_SHA=$(python3 -c "
import re, pathlib
text = pathlib.Path('formal/pin.mk').read_text()
m = re.search(r'override RISCV_FORMAL_SHA := ([0-9a-f]{40})', text)
assert m, 'could not find RISCV_FORMAL_SHA in formal/pin.mk'
print(m.group(1))
")

UPSTREAM_SHA=$(git ls-remote "$UPSTREAM_URL" HEAD | cut -f1)
# Validated before use anywhere else in this script: it is about to be
# written into formal/pin.mk, a branch name, and a shell-evaluated Python
# argument, and an upstream server is who supplies it.
if ! printf '%s' "$UPSTREAM_SHA" | grep -qE '^[0-9a-f]{40}$'; then
  echo "upstream HEAD for $UPSTREAM_URL is not a 40-hex SHA: '$UPSTREAM_SHA'" >&2
  exit 1
fi

echo "pinned:   $PIN_SHA"
echo "upstream: $UPSTREAM_SHA"

if [ "$PIN_SHA" = "$UPSTREAM_SHA" ]; then
  echo "pin is current; nothing to do"
  exit 0
fi

BRANCH="riscv-formal-pin/bump-${UPSTREAM_SHA:0:12}"

OPEN_COUNT=$(gh pr list --state open --head "$BRANCH" --json number --jq 'length')
if [ "$OPEN_COUNT" -gt 0 ]; then
  echo "a PR already proposes $UPSTREAM_SHA (branch $BRANCH); nothing to do"
  exit 0
fi

CLONE_DIR=$(mktemp -d)
trap 'rm -rf "$CLONE_DIR"' EXIT
git clone --quiet --filter=blob:none "$UPSTREAM_URL" "$CLONE_DIR"

DIFFSTAT=$(git -C "$CLONE_DIR" diff --stat "$PIN_SHA..$UPSTREAM_SHA" -- checks/ insns/ monitor/)
FULLDIFF=$(git -C "$CLONE_DIR" diff "$PIN_SHA..$UPSTREAM_SHA" -- checks/ insns/ monitor/)
DIFF_LINES=$(printf '%s\n' "$FULLDIFF" | wc -l | tr -d ' ')

git checkout -b "$BRANCH"

python3 - "$UPSTREAM_SHA" <<'EOF'
import pathlib, re, sys
new_sha = sys.argv[1]
p = pathlib.Path('formal/pin.mk')
text = p.read_text()
new_text, n = re.subn(
    r'(override RISCV_FORMAL_SHA := )[0-9a-f]{40}',
    lambda m: m.group(1) + new_sha,
    text,
)
if n != 1:
    sys.exit(f'expected exactly one RISCV_FORMAL_SHA line, replaced {n}')
p.write_text(new_text)
EOF

# A clone left on disk at the old pin trips pin.mk's own fail-closed guard
# (ADR-0013) the moment the SHA above changes underneath it.
rm -rf formal/riscv-formal
make test/monitor.v

if git diff --quiet -- formal/pin.mk test/monitor.v; then
  echo "bumping the pin produced no diff in formal/pin.mk or test/monitor.v" >&2
  exit 1
fi

git config user.name "github-actions[bot]"
git config user.email "41898282+github-actions[bot]@users.noreply.github.com"
git add formal/pin.mk test/monitor.v
git commit --quiet -m "Bump riscv-formal pin to ${UPSTREAM_SHA:0:12}

$DIFFSTAT"

git push --quiet origin "$BRANCH"

COMPARE_URL="https://github.com/YosysHQ/riscv-formal/compare/${PIN_SHA}...${UPSTREAM_SHA}"
BODY_FILE=$(mktemp)
{
  echo "Upstream riscv-formal moved."
  echo
  echo "- pinned:   \`$PIN_SHA\`"
  echo "- upstream: \`$UPSTREAM_SHA\`"
  echo "- compare:  $COMPARE_URL"
  echo
  echo "\`test/monitor.v\` is regenerated against the new pin in this commit."
  echo "The existing gates decide whether the bump is safe: monitor-freshness,"
  echo "\`formal/check-genchecks.py\`, \`formal/check-complete-exclusions.py\`,"
  echo "and the riscv-formal ladder itself."
  echo
  echo "### Diff under checks/, insns/, monitor/"
  echo
  if [ -z "$DIFFSTAT" ]; then
    echo "None -- this bump only touches other directories (e.g. cores/)."
  else
    echo '```'
    printf '%s\n' "$DIFFSTAT"
    echo '```'
    if [ "$DIFF_LINES" -le 300 ]; then
      echo
      echo "<details><summary>Full diff (${DIFF_LINES} lines)</summary>"
      echo
      echo '```diff'
      printf '%s\n' "$FULLDIFF"
      echo '```'
      echo "</details>"
    else
      echo
      echo "Full diff is ${DIFF_LINES} lines; see the compare link above."
    fi
  fi
} > "$BODY_FILE"

PR_URL=$(gh pr create --title "Bump riscv-formal pin to ${UPSTREAM_SHA:0:12}" \
  --body-file "$BODY_FILE" --head "$BRANCH" --base main)
echo "opened $PR_URL"

# A PR opened with the Actions default GITHUB_TOKEN does not trigger
# pull_request-triggered workflows -- GitHub suppresses events caused by that
# token to prevent recursive runs -- so without this, ci.yml's required checks
# would never run on this PR. workflow_dispatch is exempt, and GitHub attaches
# check runs to a PR by commit SHA regardless of which event asked for them.
if [ "${GITHUB_ACTIONS:-}" = "true" ]; then
  gh workflow run ci.yml --ref "$BRANCH"
fi

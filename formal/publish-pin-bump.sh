#!/bin/bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

if [ "$#" -ne 3 ]; then
  echo "usage: publish-pin-bump.sh <branch> <title> <out-dir>" >&2
  exit 2
fi

BRANCH=$1
TITLE=$2
OUT_DIR=$3

for f in commit-trailer issue-body.md; do
  [ -r "$OUT_DIR/$f" ] || {
    echo "publish-pin-bump.sh: cannot read $OUT_DIR/$f" >&2
    exit 2
  }
done

git config user.name "github-actions[bot]"
git config user.email "41898282+github-actions[bot]@users.noreply.github.com"
git add formal/pin.mk test/monitor.v
git commit --quiet -F - <<COMMIT
$TITLE

$(cat "$OUT_DIR/commit-trailer")
COMMIT

git push --quiet origin "$BRANCH"

# A pushed branch with no issue stops every later run, silently: the branch name
# encodes the SHA, so the decide step would rediscover it and call the bump proposed.
if ! formal/propose-pin-bump.sh "$BRANCH" "$TITLE" "$OUT_DIR/issue-body.md"; then
  echo "could not open the issue; removing $BRANCH so a later run retries" >&2
  git push --quiet --delete origin "$BRANCH" || true
  exit 1
fi

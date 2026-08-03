#!/bin/bash
# Drives formal/propose-pin-bump.sh against a stub `gh` and pins that it opens
# an issue, that the issue says what a human has to do, and that it opens no
# pull request. The defect it replaces reported success on a pull request that
# could never merge, so "it opened something" is not the property worth
# checking.
#
# The stub refuses `gh pr create` and `gh workflow run`, so a script that
# reached for either goes red here rather than proposing a bump nothing can
# merge.
#
# Hermetic: no network, no toolchain, nothing but bash.
#
# Usage: formal/test-propose-pin-bump.sh
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/.." && pwd)
SCRIPT="$REPO/formal/propose-pin-bump.sh"

tmp=$(mktemp -d "${TMPDIR:-/tmp}/propose-pin-bump.XXXXXX") || {
  echo "error: could not create a temporary directory." >&2
  exit 1
}
if [ -z "$tmp" ] || [ ! -d "$tmp" ]; then
  echo "error: mktemp -d produced no usable directory." >&2
  exit 1
fi
trap 'rm -rf "$tmp"' EXIT

mkdir -p "$tmp/bin"
cat > "$tmp/bin/gh" <<'STUB'
#!/bin/bash
# Answers `issue create`, keeps the body it was handed, and rejects everything
# else -- `pr create` and `workflow run` included -- with exit 9.
printf '%s\n' "$*" >> "$GH_LOG"
body=""
prev=""
for a in "$@"; do
  if [ "$prev" = "--body-file" ]; then body=$a; fi
  prev=$a
done
if [ "$1 $2" != "issue create" ]; then
  echo "stub gh: refusing '$1 $2'" >&2
  exit 9
fi
if [ -n "${STUB_ISSUE_FAIL:-}" ]; then
  echo "stub gh: issue creation refused" >&2
  exit 1
fi
[ -n "$body" ] && cp "$body" "$GH_BODY"
echo "https://github.com/o/r/issues/1"
STUB
chmod +x "$tmp/bin/gh"

printf 'body from the caller\n' > "$tmp/body.md"

cases=0
failed=0

# check <label> <expected-exit> <expected-text> <shell-snippet>
#
# Both the status and a fragment of the output are pinned. A status alone would
# be satisfied by a script that died on a typo before doing anything.
check() {
  local label=$1 want_exit=$2 want_text=$3 snippet=$4
  cases=$((cases + 1))
  local out rc why=""
  set +e
  out=$(eval "$snippet" 2>&1)
  rc=$?
  set -e
  if [ "$rc" -ne "$want_exit" ]; then
    why="exited $rc, expected $want_exit"
  elif ! grep -qF -- "$want_text" <<< "$out"; then
    why="output does not contain $want_text"
  fi
  if [ -z "$why" ]; then
    printf '  ok   %s\n' "$label"
  else
    printf '  RED  %s\n     -> %s\n' "$label" "$why" >&2
    printf '%s\n' "$out" | sed -e 's|^|        |' >&2
    failed=1
  fi
}

run() {  # run <case-name> [env assignments...] -- runs the script under test
  local name=$1; shift
  local dir="$tmp/$name"
  mkdir -p "$dir"
  : > "$dir/log"
  : > "$dir/body"
  env PATH="$tmp/bin:$PATH" GH_LOG="$dir/log" GH_BODY="$dir/body" "$@" \
    "$SCRIPT" pin/branch "Bump riscv-formal pin to abc123abc123" "$tmp/body.md"
}

echo "== formal/propose-pin-bump.sh"

check "it opens an issue" 0 "opened issue" \
  'run open GITHUB_REPOSITORY=o/r'
check "it opens no pull request" 0 "no pr create in the log" \
  '! grep -q "pr create" "$tmp/open/log" && echo "no pr create in the log"'
check "the issue names the branch" 0 "pin/branch" \
  'cat "$tmp/open/body"'
check "the issue says a human opens the pull request" 0 \
  "Open a pull request from" \
  'cat "$tmp/open/body"'
check "the issue links a compare page" 0 \
  "https://github.com/o/r/compare/main...pin/branch" \
  'cat "$tmp/open/body"'
check "the issue carries the caller's body" 0 "body from the caller" \
  'cat "$tmp/open/body"'

check "a refused issue is not reported as opened" 1 "issue creation refused" \
  'run issuefail STUB_ISSUE_FAIL=1'
check "too few arguments" 2 "usage: propose-pin-bump.sh" \
  'PATH="$tmp/bin:$PATH" "$SCRIPT" only-a-branch'
check "an unreadable body file" 2 "cannot read body file" \
  'PATH="$tmp/bin:$PATH" "$SCRIPT" pin/branch title "$tmp/absent.md"'

# A tripwire, not a style rule. Both of these look like they work: a PR the
# workflow opens and a run it dispatches are green in the Actions tab and
# invisible to the merge gate.
#
# Comment lines are dropped first, because both scripts name these two commands
# in prose to say why they must not be called.
echo
echo "== no script opens the pull request or dispatches CI"
for f in propose-pin-bump.sh bump-riscv-formal-pin.sh; do
  check "$f does not open a pull request" 0 "no pr create call" \
    '! grep -v "^ *#" "$REPO/formal/'"$f"'" | grep -q "gh pr create" \
       && echo "no pr create call"'
  check "$f does not dispatch CI" 0 "no dispatch call" \
    '! grep -v "^ *#" "$REPO/formal/'"$f"'" | grep -q "gh workflow run" \
       && echo "no dispatch call"'
done

echo
if [ "$failed" -ne 0 ]; then
  echo "$cases checks, at least one RED." >&2
  exit 1
fi
echo "$cases checks, all green."

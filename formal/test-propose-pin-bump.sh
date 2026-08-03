#!/bin/bash
# Drives formal/propose-pin-bump.sh against a stub `gh` and pins what it does
# in each of its two modes. The defect this replaces reported success on a pull
# request that could never merge, so "it opened something" is not the property
# worth checking: which thing it opened, and with which token, is.
#
# The stub refuses `gh workflow run` outright. Asking CI for a run that way puts
# check runs on the commit and none of them in the pull request's
# statusCheckRollup, so a script that reached for it again would go red here.
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
# Records every call, answers `pr create` and `issue create`, and rejects
# anything else. STUB_EXPECT_TOKEN pins which token the caller used.
printf '%s\n' "$*" >> "$GH_LOG"
sub="$1 $2"
body=""
prev=""
for a in "$@"; do
  if [ "$prev" = "--body-file" ]; then body=$a; fi
  prev=$a
done
case "$sub" in
  "pr create")
    if [ "${GH_TOKEN:-}" != "${STUB_EXPECT_TOKEN:-}" ]; then
      echo "stub gh: pr create ran with token '${GH_TOKEN:-}'" >&2
      exit 8
    fi
    if [ -n "${STUB_PR_FAIL:-}" ]; then
      echo "stub gh: pull request creation refused" >&2
      exit 1
    fi
    [ -n "$body" ] && cp "$body" "$GH_BODY"
    echo "https://github.com/o/r/pull/1"
    ;;
  "issue create")
    if [ -n "${STUB_ISSUE_FAIL:-}" ]; then
      echo "stub gh: issue creation refused" >&2
      exit 1
    fi
    [ -n "$body" ] && cp "$body" "$GH_BODY"
    echo "https://github.com/o/r/issues/1"
    ;;
  *)
    echo "stub gh: refusing '$sub'" >&2
    exit 9
    ;;
esac
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

check "a token opens a pull request" 0 "opened pull request" \
  'run withtoken PIN_BUMP_TOKEN=sekrit STUB_EXPECT_TOKEN=sekrit'
check "the pull request carries the supplied token" 8 \
  "pr create ran with token 'sekrit'" \
  'run wrongtoken PIN_BUMP_TOKEN=sekrit STUB_EXPECT_TOKEN=the-default-one'

check "no token opens an issue" 0 "opened issue" \
  'run notoken GITHUB_REPOSITORY=o/r'
check "no token never opens a pull request" 0 "no pr create in the log" \
  '! grep -q "pr create" "$tmp/notoken/log" && echo "no pr create in the log"'
check "the issue names the branch" 0 "pin/branch" \
  'cat "$tmp/notoken/body"'
check "the issue says what a human must do" 0 "secret named" \
  'cat "$tmp/notoken/body"'
check "the issue links a compare page" 0 \
  "https://github.com/o/r/compare/main...pin/branch" \
  'cat "$tmp/notoken/body"'
check "the issue carries the caller's body" 0 "body from the caller" \
  'cat "$tmp/notoken/body"'

check "a refused pull request is not reported as opened" 1 \
  "pull request creation refused" \
  'run prfail PIN_BUMP_TOKEN=sekrit STUB_EXPECT_TOKEN=sekrit STUB_PR_FAIL=1'
check "a refused issue is not reported as opened" 1 "issue creation refused" \
  'run issuefail STUB_ISSUE_FAIL=1'

check "too few arguments" 2 "usage: propose-pin-bump.sh" \
  'PATH="$tmp/bin:$PATH" "$SCRIPT" only-a-branch'
check "an unreadable body file" 2 "cannot read body file" \
  'PATH="$tmp/bin:$PATH" "$SCRIPT" pin/branch title "$tmp/absent.md"'

# A tripwire, not a style rule. Dispatching CI is the thing that looks like it
# works: the run is green in the Actions tab and the merge gate never sees it.
echo
echo "== no script reaches for gh workflow run"
check "propose-pin-bump.sh does not dispatch CI" 0 "no dispatch call" \
  '! grep -q "^ *gh workflow run" "$REPO/formal/propose-pin-bump.sh" \
     && echo "no dispatch call"'
check "bump-riscv-formal-pin.sh does not dispatch CI" 0 "no dispatch call" \
  '! grep -q "^ *gh workflow run" "$REPO/formal/bump-riscv-formal-pin.sh" \
     && echo "no dispatch call"'

echo
if [ "$failed" -ne 0 ]; then
  echo "$cases checks, at least one RED." >&2
  exit 1
fi
echo "$cases checks, all green."

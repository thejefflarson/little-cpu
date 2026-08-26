#!/bin/bash
# Asserts that no file git tracks also matches a .gitignore rule.
#
# Usage: tracked_ignored_test.sh [repo-root]     # defaults to this script's parent
#
# WHY THIS EXISTS. A `.gitignore` rule never applies to a file git is already
# tracking -- ignore rules are consulted only when git decides whether to
# notice an UNTRACKED path, so a rule added after the file was committed does
# nothing, and `git status` stays clean either way. Five nextpnr build
# artifacts sat in this tree exactly that way: tracked, 16.5 MB between them,
# each with its own .gitignore line that had done nothing since the day it was
# written. Nobody saw it, because the query that finds the class had never been
# run:
#
#   git ls-files -z | git check-ignore --stdin -z --no-index -v
#
# `--no-index` is the whole check. Without it, check-ignore silently skips
# every tracked path -- the same blind spot that let the five files through in
# the first place -- so a check that dropped the flag would be exactly as
# inert as the .gitignore lines it exists to catch.
#
# `--no-index` ALSO consults two exclude sources this repository does not
# carry, and both were measured rather than assumed. With `core.excludesFile`
# pointed at a file naming `*.json`, `check-ignore --no-index` matched a
# tracked `.json` file against it -- a rule on one developer's machine, red on
# theirs and green everywhere else. `-c core.excludesFile=/dev/null` answers
# that. `$GIT_DIR/info/exclude` is the same kind of rule -- per clone, never
# committed -- and is a SEPARATE source: the same `.json` line written into
# `.git/info/exclude` still matched with `core.excludesFile` neutralised, so
# that config alone does not cover it. `--no-index` needs no object database,
# and reads `$GIT_DIR/info/exclude` out of whatever `--git-dir` names, so
# `--git-dir` is pointed at a throwaway repository with no `info/exclude` of
# its own instead -- `--work-tree` keeps every path resolved against $REPO,
# so a real .gitignore anywhere in it still matches exactly as before.
#
# A hit here is never a matter of taste: a tracked-and-ignored file is either a
# dead rule (the file was committed first) or a commit that should not have
# happened (the rule was there first and something bypassed it with
# `git add -f`). Either way the fix is `git rm --cached`, never an edit to this
# script.
#
# Hermetic: git only. No toolchain, no simulator, no yosys, so this runs inside
# `make test` anywhere.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=${1:-$(cd "$HERE/.." && pwd)}

if [ ! -d "$REPO" ]; then
  echo "error: '$REPO' is not a directory, so there is nothing to scan." >&2
  exit 1
fi

tmp=$(mktemp -d "${TMPDIR:-/tmp}/littlecpu-trackedignored.XXXXXX") || {
  echo "error: could not create a temporary directory under ${TMPDIR:-/tmp}." >&2
  exit 1
}
trap 'rm -rf "$tmp"' EXIT

if ! git -c core.excludesFile=/dev/null -C "$REPO" ls-files -z > "$tmp/files" 2> /dev/null \
   || [ ! -s "$tmp/files" ]; then
  echo "error: cannot enumerate any tracked files under $REPO. A tree git" >&2
  echo "cannot list is a scan of nothing reporting green." >&2
  exit 1
fi

# A bare repository with no info/exclude of its own, so the check below can be
# pointed at it in place of $REPO's real $GIT_DIR -- see the note above.
noexclude="$tmp/no-info-exclude"
if ! git init -q --bare "$noexclude" > /dev/null 2> "$tmp/init-err"; then
  echo "error: could not create a throwaway git directory under $tmp to" >&2
  echo "isolate the check from this developer's \$GIT_DIR/info/exclude:" >&2
  cat "$tmp/init-err" >&2
  exit 1
fi

set +e
git -C "$REPO" --git-dir="$noexclude" --work-tree="$REPO" -c core.excludesFile=/dev/null \
  check-ignore --stdin -z --no-index -v < "$tmp/files" > "$tmp/hits" 2> "$tmp/err"
rc=$?
set -e

# 0 = at least one tracked path also matches a rule; 1 = none do; 128 = a
# fatal error, e.g. a .gitignore this build of git cannot parse. 1 is the only
# passing status -- a run that could not ask the question is not a green one.
case "$rc" in
  1) : ;;
  0)
    echo "error: the following tracked files also match a .gitignore rule." >&2
    echo "The rule does nothing for a file git already tracks. Run" >&2
    echo "'git rm --cached <path>' for each -- or if the file was tracked on" >&2
    echo "purpose, delete the dead rule instead:" >&2
    echo >&2
    # Four NUL-delimited fields per match: source, linenum, pattern, pathname.
    while IFS= read -r -d '' source \
       && IFS= read -r -d '' linenum \
       && IFS= read -r -d '' pattern \
       && IFS= read -r -d '' path; do
      printf '  %s  (matches "%s" at %s:%s)\n' "$path" "$pattern" "$source" "$linenum" >&2
    done < "$tmp/hits"
    exit 1
    ;;
  *)
    echo "error: git check-ignore exited $rc, which is neither 0 nor 1:" >&2
    cat "$tmp/err" >&2
    exit 1
    ;;
esac

n=$(tr -cd '\0' < "$tmp/files" | wc -c | tr -d ' ')
echo "$n tracked files, none matching a .gitignore rule"

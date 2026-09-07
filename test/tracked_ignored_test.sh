#!/bin/bash
# Asserts that no file git tracks also matches a .gitignore rule.
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

# A bare repository with no info/exclude of its own, so the check below can be pointed at
# it in place of $REPO's real $GIT_DIR -- see the note above.
notemplate="$tmp/no-template"
mkdir -p "$notemplate"
noexclude="$tmp/no-info-exclude"
if ! git init -q --bare --template="$notemplate" "$noexclude" > /dev/null 2> "$tmp/init-err"; then
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

case "$rc" in
  1) : ;;
  0)
    echo "error: the following tracked files also match a .gitignore rule." >&2
    echo "The rule does nothing for a file git already tracks. Run" >&2
    echo "'git rm --cached <path>' for each -- or if the file was tracked on" >&2
    echo "purpose, delete the dead rule instead:" >&2
    echo >&2
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

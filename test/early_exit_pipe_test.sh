#!/bin/bash
# Refuses `grep -q` or `head` reading a pipe in a pipefail bash script: an early exit SIGPIPEs the
# writer and the pipeline returns 141 for a match that was there. #!/bin/sh and Makefile recipes have
# no pipefail and are not scanned; a trailing "early-exit-pipe: intended" exempts a line under test.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=${1:-$(cd "$HERE/.." && pwd)}

MARK='early-exit-pipe:'' intended'
BARE='[|][[:space:]]*(grep|egrep|fgrep)[[:space:]]+(-[A-Za-z]*q|--quiet|--silent)'
HEADED='(printf|echo)[^|;&]*[|][[:space:]]*head([[:space:]]|$)'

if [ ! -d "$REPO" ]; then
  echo "error: '$REPO' is not a directory, so there is nothing to scan." >&2
  exit 1
fi

tmp=$(mktemp -d "${TMPDIR:-/tmp}/littlecpu-earlyexit.XXXXXX") || {
  echo "error: could not create a temporary directory under ${TMPDIR:-/tmp}." >&2
  exit 1
}
trap 'rm -rf "$tmp"' EXIT

if ! git -C "$REPO" ls-files -z > "$tmp/files" 2>/dev/null || [ ! -s "$tmp/files" ]; then
  echo "error: cannot enumerate any tracked files under $REPO. This check reads" >&2
  echo "git's index; a tree git cannot list is a scan of nothing reporting green." >&2
  exit 1
fi

scanned=0
rc=0
while IFS= read -r -d '' path; do
  case $path in *.sh) ;; *) continue;; esac
  [ -f "$REPO/$path" ] || continue
  first=""
  IFS= read -r first < "$REPO/$path" || true
  case $first in
    '#!/bin/bash'*|'#!/usr/bin/env bash'*) ;;
    *) continue;;
  esac
  scanned=$((scanned + 1))

  grep -nE -e "$BARE" -e "$HEADED" -- "$REPO/$path" > "$tmp/hits" || true
  while IFS= read -r hit; do
    [ -n "$hit" ] || continue
    body=${hit#*:}
    case $body in *"$MARK"*) continue;; esac
    case $body in [[:space:]]*'#'*|'#'*) continue;; esac
    echo "$path:${hit%%:*}: a reader that quits early reads a pipe: $body" >&2
    rc=1
  done < "$tmp/hits"

  grep -nF -e "$MARK" -- "$REPO/$path" > "$tmp/marked" || true
  while IFS= read -r hit; do
    [ -n "$hit" ] || continue
    case ${hit#*:} in [[:space:]]*'#'*|'#'*) continue;; esac
    if ! grep -qE -e "$BARE" -e "$HEADED" <<< "${hit#*:}"; then
      echo "$path:${hit%%:*}: marked '$MARK' but the line no longer has the shape" >&2
      rc=1
    fi
  done < "$tmp/marked"
done < "$tmp/files"

if [ "$scanned" -eq 0 ]; then
  echo "error: no tracked bash script under $REPO, so there is nothing to scan." >&2
  exit 1
fi
if [ "$rc" -ne 0 ]; then
  echo "Read the text from a here-string instead: grep -q PATTERN <<< \"\$text\"." >&2
  exit 1
fi
echo "early-exit pipes: none in $scanned bash scripts; every grep -q and head reads a here-string or a file."

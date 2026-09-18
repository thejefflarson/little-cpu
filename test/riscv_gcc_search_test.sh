#!/bin/bash
# Refuses the two RISC-V cross-compiler names ADR-0190 retired, anywhere they are not a
# named exception below.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=${1:-$(cd "$HERE/.." && pwd)}

if [ ! -d "$REPO" ]; then
  echo "error: '$REPO' is not a directory, so there is nothing to scan." >&2
  exit 1
fi

RETIRED_NAMES='riscv64-elf-gcc riscv64-unknown-elf-gcc'

# The allow-list: paths where an old name is a measurement's own record, not a live
# search. Exact files and directory prefixes only -- no glob wide enough to also excuse a
# future soc/ or test/ script.
allow_paths() {
  sed -e 's/#.*//' -e 's/[[:space:]]*$//' -e '/^$/d' <<'PATHS'
# Dated decision records and their index. History: several of them quote the exact
# names a build used to search for, which is the value of the record. Every prose
# doc under here, including the manifests, inherits the exemption the same way.
docs/

# The weekly cross-core stamp (`make compare-product`). It records the command
# line each pair's image was actually built with, and its own current stamp
# predates the pin; the schedule workflow re-takes it under the pinned compiler
# on its own.
soc/compare/product.json

# The probe that forces this check red plants both retired names in a fixture
# and quotes them in its label. A probe that cannot name what it is planting is
# not a probe.
test/probe_gates.sh

# This file: the declaration above and this list.
test/riscv_gcc_search_test.sh
PATHS
}

tmp=$(mktemp -d "${TMPDIR:-/tmp}/littlecpu-riscvgcc.XXXXXX") || {
  echo "error: could not create a temporary directory under ${TMPDIR:-/tmp}." >&2
  exit 1
}
trap 'rm -rf "$tmp"' EXIT

allow_paths > "$tmp/allow"
if [ ! -s "$tmp/allow" ]; then
  echo "error: the allow-list is empty, so every dated ADR and the cross-core stamp" >&2
  echo "would go red for quoting history." >&2
  exit 1
fi

if ! git -C "$REPO" ls-files -z > "$tmp/files" 2>/dev/null || [ ! -s "$tmp/files" ]; then
  echo "error: cannot enumerate any tracked files under $REPO. This check reads" >&2
  echo "git's index, because what it guards is a name arriving in a commit; a" >&2
  echo "tree git cannot list is a scan of nothing reporting green." >&2
  exit 1
fi

match_entry() {
  local path=$1 entry
  while IFS= read -r entry; do
    case "$entry" in
      */) case "$path" in "$entry"*) printf '%s' "$entry"; return 0 ;; esac ;;
      *)  if [ "$path" = "$entry" ]; then printf '%s' "$entry"; return 0; fi ;;
    esac
  done < "$tmp/allow"
  return 0
}

rc=0
: > "$tmp/unexpected"
: > "$tmp/covered"

for name in $RETIRED_NAMES; do
  hits=$( (cd "$REPO" && xargs -0 grep -nIF -e "$name" -- /dev/null < "$tmp/files") || true)
  while IFS= read -r hit; do
    [ -n "$hit" ] || continue
    path=${hit%%:*}
    entry=$(match_entry "$path")
    if [ -n "$entry" ]; then
      printf '%s\n' "$entry" >> "$tmp/covered"
    else
      printf '%s\n' "$hit" >> "$tmp/unexpected"
    fi
  done <<< "$hits"
done

if [ -s "$tmp/unexpected" ]; then
  rc=1
  echo "error: a retired compiler name appears where nothing allows it:" >&2
  sed -e 's|^|  |' "$tmp/unexpected" >&2
  echo >&2
  echo "ADR-0190 pinned riscv-none-elf-gcc and rewrote every consumer from" >&2
  echo "'search riscv64-elf-gcc, then riscv64-unknown-elf-gcc' to resolving the" >&2
  echo "one pinned name and saying 'run make riscv-gcc-setup' when it is" >&2
  echo "missing. Copy that wording rather than reviving a search; if the use" >&2
  echo "above genuinely records history -- a dated ADR, the cross-core stamp --" >&2
  echo "add its path to the allow-list in test/riscv_gcc_search_test.sh." >&2
fi

while IFS= read -r entry; do
  if ! grep -qxF -- "$entry" "$tmp/covered"; then
    rc=1
    echo >&2
    echo "error: the allow-list exempts $entry, and neither retired name" >&2
    echo "appears there any more. Delete the entry -- or if that use moved, move" >&2
    echo "the entry with it. An exemption kept past its reason is how the next one" >&2
    echo "gets waved through, and it is why this comparison runs both ways." >&2
  fi
done < "$tmp/allow"

if [ "$rc" -ne 0 ]; then
  exit 1
fi

entries=$(wc -l < "$tmp/allow" | tr -d ' ')
files=$(tr -cd '\0' < "$tmp/files" | wc -c | tr -d ' ')
echo "no riscv64-elf-gcc / riscv64-unknown-elf-gcc search outside $entries named exceptions, over $files tracked files"

#!/bin/sh
set -e

REPO=${1:-$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)}
cd "$REPO"

if [ ! -f Makefile ]; then
  echo "error: no Makefile at $REPO, so there is nothing to read." >&2
  exit 1
fi

out=$(make --dry-run --no-print-directory __makefile_target_test_no_such_target__ 2>&1 || true)

if ! printf '%s\n' "$out" | grep -q 'No rule to make target.*__makefile_target_test_no_such_target__'; then
  echo "error: make did not report a missing rule for the probe target, so this" >&2
  echo "run did not parse the Makefile the way this check assumes." >&2
  printf '%s\n' "$out" >&2
  exit 1
fi

dupes=$(printf '%s\n' "$out" | grep 'overriding \(commands\|recipe\) for target' || true)
if [ -n "$dupes" ]; then
  echo "error: a target is defined more than once in $REPO/Makefile." >&2
  echo "A redefined recipe is last-wins with only this warning, while a \`?=\`" >&2
  echo "default beside it is first-wins -- so the surviving body can be paired" >&2
  echo "with the other definition's variables. Give the two routes two names." >&2
  printf '%s\n' "$dupes" | while IFS= read -r line; do echo "  $line" >&2; done
  exit 1
fi

echo "no target defined twice in $REPO/Makefile"

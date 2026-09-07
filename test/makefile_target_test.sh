#!/bin/sh
# A target defined twice in the Makefile is LAST-WINS with only a warning, while a
# `?=` default beside it is FIRST-WINS -- so two changes that each add the same
# recipe name produce one route's variables paired with the other's body, and
# nothing says so. That shape reached review once: two CoreMark board routes at
# different ROM geometries both named `coremark-rom`, differing in linker script,
# flags and whether they checked the vendored sources against PINNED.sha256 at all.
#
# make's own parser is the oracle here rather than a second one this file would
# hand-roll: make already reports the collision, at both line numbers, and reading
# its warning cannot disagree with what make actually did.
#
# Usage: makefile_target_test.sh [repo-root]
set -e

REPO=${1:-$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)}
cd "$REPO"

if [ ! -f Makefile ]; then
  echo "error: no Makefile at $REPO, so there is nothing to read." >&2
  exit 1
fi

# A target name nothing defines: make parses the whole file, emits every warning it
# has, and only then fails to find a rule. Parse warnings are what this reads; the
# missing-rule failure is expected and discarded.
out=$(make --dry-run --no-print-directory __makefile_target_test_no_such_target__ 2>&1 || true)

# The probe target BY NAME, not just "No rule to make target": a Makefile with a
# missing `include` fails with that same wording about the include file, having never
# parsed far enough to warn about anything.
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

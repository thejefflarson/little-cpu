#!/bin/sh
# Tests each NANO_SRCS entry with its own `test -e`: one `test -e $(NANO_SRCS)` fails
# with "too many arguments" for a second entry, misreading a landed import as absent.
present=""
missing=""
for f in "$@"; do
  if [ -e "$f" ]; then present="$present $f"; else missing="$missing $f"; fi
done

if [ -z "$present" ]; then
  echo "srcs_guard.sh: no$missing -- the donor import has not landed in this" >&2
  echo "tree yet. Nothing to measure; this is not a failure." >&2
  exit 2
fi

if [ -n "$missing" ]; then
  echo "srcs_guard.sh: found$present but not$missing -- a partial NANO_SRCS." >&2
  echo "Refusing to synthesise half an import." >&2
  exit 1
fi

echo "srcs_guard.sh: all of$present present, proceeding" >&2
exit 0

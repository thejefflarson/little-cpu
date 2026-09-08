#!/bin/sh
# Tests each NANO_SRCS entry with its own `test -e` rather than handing the whole
# list to one `test -e $(NANO_SRCS)`, which fails with "too many arguments" for a
# second entry whether or not the files exist -- so any NANO_SRCS beyond one word
# used to read as "the donor import has not landed" even once every file had.
#
# Exit 0: every named source exists -- proceed. Exit 1: some exist and some do
# not, named on stderr -- a partial import, never guessed past. Exit 2: none
# exist, named on stderr -- nothing has landed yet, and that is not a failure.
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

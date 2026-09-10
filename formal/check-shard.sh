#!/bin/bash
# Runs every nth generated check, starting at i, out of an already-generated checks
# directory. Shared by formal/ and nano/formal/, whose check sets differ but whose
# layout does not.
#
# IT GRADES NOTHING, and that is the design. formal/check-baseline.sh compares two
# SETS in both directions and needs every check's status at once; a shard holds a
# fraction of them and could only ever grade a fraction of the question. The job that
# collects the shards runs the baseline over the union. A shard that never ran leaves
# no status file behind, check-baseline.sh reads that as NO-STATUS, and NO-STATUS is
# not PASS -- so a lost shard fails the baseline rather than quietly shrinking it.
set -uo pipefail

if [ $# -lt 2 ] || [ $# -gt 3 ]; then
  echo "usage: $0 <checks-dir> <i>/<n> [jobs]" >&2
  exit 2
fi

CHECKS_DIR=$1
SPEC=$2
JOBS=${3:-4}

case $SPEC in
  [1-9]/[1-9] | [1-9]/[1-9][0-9] | [1-9][0-9]/[1-9][0-9]) ;;
  *) echo "error: shard spec is '$SPEC', not <i>/<n> with both in 1..99." >&2; exit 2 ;;
esac
i=${SPEC%%/*}
n=${SPEC##*/}
if [ "$i" -gt "$n" ]; then
  echo "error: shard spec '$SPEC' asks for shard $i of $n." >&2
  exit 2
fi

if [ ! -d "$CHECKS_DIR" ]; then
  echo "error: no such checks directory: $CHECKS_DIR" >&2
  echo "Generate the set first; this script runs an existing one and makes none." >&2
  exit 2
fi

names=$(cd "$CHECKS_DIR" && ls -- *.sby 2>/dev/null | sed 's/\.sby$//' | LC_ALL=C sort \
        | awk -v i="$i" -v n="$n" 'NR % n == i % n')
if [ -z "$names" ]; then
  echo "error: shard $SPEC selected no checks out of $CHECKS_DIR." >&2
  echo "Either the set was never generated, or n is larger than it is." >&2
  exit 2
fi

count=$(printf '%s\n' $names | wc -l | tr -d ' ')
total=$(cd "$CHECKS_DIR" && ls -- *.sby 2>/dev/null | wc -l | tr -d ' ')
echo "shard $SPEC: $count of $total checks, $JOBS at a time"
# sby's own status is not read: `expect pass,fail` means it exits 0 either way, and the
# verdict is the baseline the collecting job runs. What matters here is that every
# selected check got as far as writing a status.
make -C "$CHECKS_DIR" -j"$JOBS" -k $names || true

missing=""
for name in $names; do
  [ -s "$CHECKS_DIR/$name/status" ] || missing="$missing $name"
done
if [ -n "$missing" ]; then
  echo "error: shard $SPEC finished with no status for:$missing" >&2
  echo "Those would reach the baseline as NO-STATUS. Failing here names them." >&2
  exit 1
fi
echo "shard $SPEC: $count statuses written"

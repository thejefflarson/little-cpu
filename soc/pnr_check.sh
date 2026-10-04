#!/bin/sh
# Grades one nextpnr run: <target> <nextpnr-exit-status> <log> <artifact>...
# A nonzero exit is tolerated only when every ERROR line in the log is the timing
# verdict ("Max frequency for clock"), because a design under its clock is a real
# placement that the caller's own requirement grades. Any other failure, or a missing
# artifact, means nothing was measured.
set -eu

if [ "$#" -lt 4 ]; then
  echo "usage: soc/pnr_check.sh <target> <nextpnr-status> <log> <artifact>..." >&2
  exit 2
fi
target=$1 status=$2 log=$3
shift 3

fail() {
  reason=$1
  shift
  echo "*** $target: nextpnr $reason, so NOTHING was measured. That is a failed" >&2
  echo "*** placement, not a slow design." >&2
  tail -30 "$log" >&2
  rm -f "$@"
  exit 1
}

for artifact in "$@"; do
  [ -s "$artifact" ] || fail "wrote no $artifact" "$@"
done

other=$(grep '^ERROR' "$log" | grep -v 'Max frequency for clock' || true)
[ -z "$other" ] || fail "logged an ERROR" "$@"

if [ "$status" != 0 ] && ! grep -q '^ERROR: Max frequency for clock' "$log"; then
  fail "exited $status with no timing verdict behind it" "$@"
fi

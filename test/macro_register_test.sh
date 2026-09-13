#!/bin/bash
# Asserts test/asm's shared test-macro headers name only x0-x15, since nano (RV32E) traps on x16-x31.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=${1:-$(cd "$HERE/.." && pwd)}

MACROS_H="$REPO/test/asm/test_macros.h"
RISCV_TEST_H="$REPO/test/asm/riscv_test.h"

for f in "$MACROS_H" "$RISCV_TEST_H"; do
  if [ ! -f "$f" ] || [ ! -r "$f" ]; then
    echo "error: '$f' does not exist or is not readable." >&2
    exit 1
  fi
done

REGS_X16_X31='x1[6-9]|x2[0-9]|x3[01]|a6|a7|s[2-9]|s1[01]|t[3-6]'
PATTERN="(^|[^0-9A-Za-z_])($REGS_X16_X31)([^0-9A-Za-z_]|\$)"

# grep exit 1 is "no match" (fine); `|| true` alone cannot tell that apart from exit >=2.
set +e
hit=$(grep -nE "$PATTERN" "$MACROS_H" "$RISCV_TEST_H")
grep_rc=$?
set -e
if [ "$grep_rc" -ge 2 ]; then
  echo "error: grep could not scan the shared macro headers (exit $grep_rc)." >&2
  exit 1
fi
if [ -n "$hit" ]; then
  echo "error: a shared test macro names a register outside x0-x15:" >&2
  printf '%s\n' "$hit" | sed -e 's|^|  |' >&2
  exit 1
fi

echo "test_macros.h and riscv_test.h name only x0-x15."

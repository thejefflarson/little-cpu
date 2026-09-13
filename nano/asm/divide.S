// nano-local, x0-x15 only.
#include "riscv_test.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  li x1, 20
  li x2, 6
  li x5, 3
  div x4, x1, x2
  li gp, 2
  bne x4, x5, fail

  li x1, -20
  li x2, 6
  li x5, -3
  div x4, x1, x2
  li gp, 3
  bne x4, x5, fail

  li x1, -20
  li x2, 6
  li x5, -2
  rem x4, x1, x2
  li gp, 4
  bne x4, x5, fail

  li x1, 20
  li x2, 6
  li x5, 3
  divu x4, x1, x2
  li gp, 5
  bne x4, x5, fail

  li x1, 20
  li x2, 6
  li x5, 2
  remu x4, x1, x2
  li gp, 6
  bne x4, x5, fail

  RVTEST_PASS

fail:
  RVTEST_FAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN
RVTEST_DATA_END

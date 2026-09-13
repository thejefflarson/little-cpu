// nano-local: every register below is x0-x15.
#include "riscv_test.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  li x1, 6
  li x2, 7
  li x5, 42
  mul x4, x1, x2
  li gp, 2
  bne x4, x5, fail

  li x1, -1
  li x2, -1
  li x5, 0
  mulh x4, x1, x2
  li gp, 3
  bne x4, x5, fail

  li x1, -1
  li x2, 4
  li x5, -1
  mulhsu x4, x1, x2
  li gp, 4
  bne x4, x5, fail

  li x1, 0x80000000
  li x2, 2
  li x5, 1
  mulhu x4, x1, x2
  li gp, 5
  bne x4, x5, fail

  RVTEST_PASS

fail:
  RVTEST_FAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN
RVTEST_DATA_END

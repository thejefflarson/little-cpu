// nano-local: every register below is x0-x15.
#include "riscv_test.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  li x1, 5
  li x2, 5
  li gp, 2
  bne x1, x2, fail
  beq x1, x2, 1f
  j fail
1:

  li x2, 6
  li gp, 3
  beq x1, x2, fail

  li x1, -1
  li x2, 1
  li gp, 4
  bge x1, x2, fail
  bltu x1, x2, fail

  li gp, 5
  blt x1, x2, 2f
  j fail
2:

  li gp, 6
  bgeu x1, x2, 3f
  j fail
3:

  li x4, 0
  jal x5, 4f
  j fail
4:
  li gp, 7
  bne x4, x0, fail

  la x7, 5f
  jalr x8, x7, 0
  j fail
5:
  li gp, 8
  addi x4, x4, 1
  li x9, 1
  bne x4, x9, fail

  RVTEST_PASS

fail:
  RVTEST_FAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN
RVTEST_DATA_END

// nano-local: every register below is x0-x15, since nano is RV32E and traps on x16-x31.
#include "riscv_test.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  li x1, 3
  li x2, 4
  li x5, 7
  add x4, x1, x2
  li gp, 2
  bne x4, x5, fail

  li x5, -1
  sub x4, x1, x2
  li gp, 3
  bne x4, x5, fail

  li x1, 0x0f0f
  li x2, 0x00ff
  li x5, 0x000f
  and x4, x1, x2
  li gp, 4
  bne x4, x5, fail

  li x5, 0x0fff
  or x4, x1, x2
  li gp, 5
  bne x4, x5, fail

  li x5, 0x0ff0
  xor x4, x1, x2
  li gp, 6
  bne x4, x5, fail

  li x1, 1
  li x5, 8
  slli x4, x1, 3
  li gp, 7
  bne x4, x5, fail

  li x1, -8
  li x5, -1
  srai x4, x1, 3
  li gp, 8
  bne x4, x5, fail

  li x1, 0x80000000
  li x5, 0x10000000
  srli x4, x1, 3
  li gp, 9
  bne x4, x5, fail

  li x1, 1
  li x2, -1
  li x5, 1
  sltu x4, x1, x2
  li gp, 10
  bne x4, x5, fail

  li x5, 0
  slt x4, x1, x2
  li gp, 11
  bne x4, x5, fail

  li x1, 5
  li x5, 1
  slti x4, x1, 10
  li gp, 12
  bne x4, x5, fail

  li x5, 0
  addi x4, x0, 0
  li gp, 13
  bne x4, x5, fail

  RVTEST_PASS

fail:
  RVTEST_FAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN
RVTEST_DATA_END

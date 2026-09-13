// nano-local: every register below is x0-x15.
#include "riscv_test.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  la x1, scratch
  li x2, 0x12345678
  sw x2, 0(x1)
  lw x6, 0(x1)
  li gp, 2
  bne x6, x2, fail

  lb x6, 0(x1)
  li x4, 0x78
  li gp, 3
  bne x6, x4, fail

  lbu x6, 3(x1)
  li x4, 0x12
  li gp, 4
  bne x6, x4, fail

  lh x6, 0(x1)
  li x4, 0x5678
  slli x4, x4, 16
  srai x4, x4, 16
  li gp, 5
  bne x6, x4, fail

  lhu x6, 2(x1)
  li x4, 0x1234
  li gp, 6
  bne x6, x4, fail

  li x5, 0xab
  sb x5, 8(x1)
  lbu x6, 8(x1)
  li gp, 7
  bne x6, x5, fail

  li x5, 0xbeef
  sh x5, 10(x1)
  lhu x6, 10(x1)
  li gp, 8
  bne x6, x5, fail

  RVTEST_PASS

fail:
  RVTEST_FAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN
scratch:
  .word 0
  .word 0
  .word 0
  .word 0
RVTEST_DATA_END

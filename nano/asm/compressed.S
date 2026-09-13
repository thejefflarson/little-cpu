// nano-local: every register below is x0-x15, and every compressed form here uses
// registers in x8-x15 where the C encoding's 3-bit field requires it.
#include "riscv_test.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  addi x0, x0, 0

  li x8, 5
  addi x8, x8, 3
  li x5, 8
  li gp, 2
  bne x8, x5, fail

  li x9, 0
  mv x9, x8
  li gp, 3
  bne x9, x8, fail

  add x9, x9, x8
  li x5, 16
  li gp, 4
  bne x9, x5, fail

  li x8, 0x0f
  li x9, 0x33
  and x10, x8, x9
  li x5, 0x03
  li gp, 5
  bne x10, x5, fail

  or x10, x8, x9
  li x5, 0x3f
  li gp, 6
  bne x10, x5, fail

  xor x10, x8, x9
  li x5, 0x3c
  li gp, 7
  bne x10, x5, fail

  sub x10, x9, x8
  li x5, 0x24
  li gp, 8
  bne x10, x5, fail

  andi x8, x9, 0x0f
  li x5, 0x03
  li gp, 9
  bne x8, x5, fail

  li x8, -1
  srli x8, x8, 28
  li x5, 0x0f
  li gp, 10
  bne x8, x5, fail

  li x8, -8
  srai x8, x8, 2
  li x5, -2
  li gp, 11
  bne x8, x5, fail

  la x11, scratch
  li x12, 123
  sw x12, 4(x11)
  lw x13, 4(x11)
  li gp, 12
  bne x13, x12, fail

  li x14, 7
  beqz x0, 1f
  j fail
1:
  li gp, 13
  bnez x14, 2f
  j fail
2:

  slli x14, x14, 4
  li x5, 112
  li gp, 14
  bne x14, x5, fail

  la sp, __stack_top
  addi sp, sp, -16
  li x1, 99
  sw x1, 0(sp)
  lw x11, 0(sp)
  li gp, 15
  bne x11, x1, fail
  addi sp, sp, 16

  RVTEST_PASS

fail:
  RVTEST_FAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN
scratch:
  .word 0
  .word 0
RVTEST_DATA_END

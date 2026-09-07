
#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  .option push
  .option norvc

  # Straight-line: a 32-bit instruction immediately after a compressed one.
  li TESTNUM, 2
  .option push
  .option rvc
  .p2align 2
  c.nop
  .option pop
  addi a0, x0, 0x123    # straddles pc % 4 == 2
  li a1, 0x123
  bne a0, a1, fail

  li TESTNUM, 3
  li a2, 1
  beq a2, a2, 1f
  j fail
  .option push
  .option rvc
  .p2align 2
  c.nop
  .option pop
1:
  addi a3, x0, 0x456    # branch target, pc % 4 == 2
  li a4, 0x456
  bne a3, a4, fail

  .option pop

  TEST_PASSFAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

RVTEST_DATA_END

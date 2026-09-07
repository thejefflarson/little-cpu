#include "riscv_test.h"
#include "test_macros.h"

RVTEST_CODE_BEGIN

  li    a0, 0x0BADF00D
  li    a1, 0x12345678
  li    a2, 0x0000FFFF

  fence
  fence.i
  wfi
  .option push
  .option rvc
  c.nop
  .option pop

  TEST_CASE( 1, a0, 0x0BADF00D, nop )
  TEST_CASE( 2, a1, 0x12345678, nop )
  TEST_CASE( 3, a2, 0x0000FFFF, nop )

  li    a3, 0
  fence
  addi  a3, a3, 1
  wfi
  addi  a3, a3, 1
  TEST_CASE( 4, a3, 2, nop )

  TEST_PASSFAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN
  TEST_DATA
RVTEST_DATA_END

# See LICENSE for license details.

# Two CSRs the RV32 privileged specification lists unconditionally in the machine-mode
# register tables.

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  RVTEST_INSTALL_TRAP_HANDLER

  .option push
  .option norvc
  csrr  a0, 0x310
  .option pop

  TEST_CASE( 2, a4, 0, la a3, trap_count; lw a4, 0(a3); )
  TEST_CASE( 3, a0, 0, nop; )

  li    a5, 0xffffffff
  .option push
  .option norvc
  csrw  0x310, a5
  .option pop

  TEST_CASE( 4, a4, 0, la a3, trap_count; lw a4, 0(a3); )
  TEST_CASE( 5, a0, 0, csrr a0, 0x310; )

  .option push
  .option norvc
  csrr  a0, 0xF15
  .option pop

  TEST_CASE( 6, a4, 0, la a3, trap_count; lw a4, 0(a3); )
  TEST_CASE( 7, a0, 0, nop; )

  .option push
  .option norvc
  csrw  0xF15, a5
  .option pop

  TEST_CASE( 8, a4, 1, la a3, trap_count; lw a4, 0(a3); )
  TEST_CASE( 9, a4, 2, la a3, trap_cause; lw a4, 0(a3); )

  TEST_PASSFAIL

  RVTEST_TRAP_HANDLER

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  RVTEST_TRAP_DATA

RVTEST_DATA_END

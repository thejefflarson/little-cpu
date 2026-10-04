# See LICENSE for license details.

// Zkr's mseccfg (0x747) and mseccfgh (0x757): SSEED and USEED are read-only zero, so every
// write is a legal no-op and neighbouring addresses stay unimplemented.

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  RVTEST_INSTALL_TRAP_HANDLER

  li    a5, 0xffffffff
  TEST_CASE(  2, a4, 0, csrr a4, 0x747 )
  TEST_CASE(  3, a4, 0, csrr a4, 0x757 )
  csrw  0x747, a5
  TEST_CASE(  4, a4, 0, csrr a4, 0x747 )
  csrw  0x757, a5
  TEST_CASE(  5, a4, 0, csrr a4, 0x757 )
  csrs  0x747, a5
  TEST_CASE(  6, a4, 0, csrr a4, 0x747 )
  csrc  0x757, a5
  TEST_CASE(  7, a4, 0, csrr a4, 0x757 )
  TEST_CASE(  8, a4, 0, la a3, trap_count; lw a4, 0(a3); )

  .option push
  .option norvc
  csrr  a0, 0x746
  csrr  a0, 0x767
  TEST_CASE(  9, a4, 2, la a3, trap_count; lw a4, 0(a3); )
  .option pop

  TEST_PASSFAIL

  RVTEST_TRAP_HANDLER

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  RVTEST_TRAP_DATA

RVTEST_DATA_END

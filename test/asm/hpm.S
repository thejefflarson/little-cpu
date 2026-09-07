# See LICENSE for license details.

// The 87 hardware performance monitor addresses: mhpmcounter3-31 (0xB03-0xB1F),
// mhpmcounter3h-31h (0xB83-0xB9F) and mhpmevent3-31 (0x323-0x33F).

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  RVTEST_INSTALL_TRAP_HANDLER

  .option push
  .option norvc

  TEST_CASE(  2, a0, 0, csrr a0, 0xB03; )
  TEST_CASE(  3, a0, 0, csrr a0, 0xB11; )
  TEST_CASE(  4, a0, 0, csrr a0, 0xB1F; )

  TEST_CASE(  5, a0, 0, csrr a0, 0xB83; )
  TEST_CASE(  6, a0, 0, csrr a0, 0xB91; )
  TEST_CASE(  7, a0, 0, csrr a0, 0xB9F; )

  TEST_CASE(  8, a0, 0, csrr a0, 0x323; )
  TEST_CASE(  9, a0, 0, csrr a0, 0x331; )
  TEST_CASE( 10, a0, 0, csrr a0, 0x33F; )

  TEST_CASE( 11, a4, 0, la a3, trap_count; lw a4, 0(a3); )

  li    a5, 0xffffffff

  TEST_CASE( 12, a0, 0, csrrw a0, 0xB03, a5; )
  TEST_CASE( 13, a0, 0, csrrs a0, 0xB9F, a5; )
  TEST_CASE( 14, a0, 0, csrrc a0, 0x33F, a5; )
  TEST_CASE( 15, a0, 0, csrrwi a0, 0xB1F, 31; )
  TEST_CASE( 16, a0, 0, csrrsi a0, 0xB83, 31; )
  TEST_CASE( 17, a0, 0, csrrci a0, 0x323, 31; )

  TEST_CASE( 18, a0, 0, csrr a0, 0xB03; )
  TEST_CASE( 19, a0, 0, csrr a0, 0xB9F; )
  TEST_CASE( 20, a0, 0, csrr a0, 0x33F; )

  TEST_CASE( 21, a4, 0, la a3, trap_count; lw a4, 0(a3); )

  csrr  a0, 0xB20

  TEST_CASE( 22, a4, 1, la a3, trap_count; lw a4, 0(a3); )
  TEST_CASE( 23, a4, 2, la a3, trap_cause; lw a4, 0(a3); )

  .option pop

  TEST_PASSFAIL

  RVTEST_TRAP_HANDLER

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  RVTEST_TRAP_DATA

RVTEST_DATA_END

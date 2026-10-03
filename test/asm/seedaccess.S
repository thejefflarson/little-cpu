# See LICENSE for license details.

// The Zkr `seed` CSR: which access forms trap; nothing branches on a read value.

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  RVTEST_INSTALL_TRAP_HANDLER

  .option push
  .option norvc

  TEST_CASE(  2, a4, 0, la a3, trap_count; lw a4, 0(a3); )
  csrrs a0, 0x015, x0
  TEST_CASE(  3, a4, 1, la a3, trap_count; lw a4, 0(a3); )
  TEST_CASE(  4, a4, 2, la a3, trap_cause; lw a4, 0(a3); )
  TEST_CASE(  5, a4, 0x01502573, la a3, trap_tval; lw a4, 0(a3); )
  csrrc a0, 0x015, x0
  csrrsi a0, 0x015, 0
  csrrci a0, 0x015, 0
  TEST_CASE(  6, a4, 4, la a3, trap_count; lw a4, 0(a3); )
  TEST_CASE(  7, a4, 2, la a3, trap_cause; lw a4, 0(a3); )

  li    a5, 0xffffffff
  csrrw a0, 0x015, x0
  csrrw a0, 0x015, a5
  csrrs a0, 0x015, a5
  csrrc a0, 0x015, a5
  csrrwi a0, 0x015, 0
  csrrsi a0, 0x015, 1
  csrrci a0, 0x015, 1
  csrw  0x015, a5
  csrs  0x015, a5
  TEST_CASE(  8, a4, 4, la a3, trap_count; lw a4, 0(a3); )

  TEST_CASE(  9, a4, 4, la a3, trap_count; lw a4, 0(a3); )

  .option pop

  TEST_PASSFAIL

  RVTEST_TRAP_HANDLER

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  RVTEST_TRAP_DATA

RVTEST_DATA_END

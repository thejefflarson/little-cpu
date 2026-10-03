# See LICENSE for license details.

// The Zkr `seed` CSR: status handling and consumption; baselined against the model, whose source is ready at once.

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  RVTEST_INSTALL_TRAP_HANDLER

  .option push
  .option norvc

  TEST_CASE(  2, a4, 0, csrrw a0, 0x015, x0; srli a4, a0, 30; )
  TEST_CASE(  3, a4, 0, slli a4, a0, 2; srli a4, a4, 2; )

  li    a6, 1500
poll:
  csrrw a0, 0x015, x0
  srli  a3, a0, 30
  li    a2, 3
  li    TESTNUM, 4
  beq   a3, a2, fail
  li    a2, 2
  beq   a3, a2, ready
  slli  a4, a0, 16
  li    TESTNUM, 5
  bnez  a4, fail
  addi  a6, a6, -1
  li    TESTNUM, 6
  beqz  a6, fail
  j     poll
ready:

  TEST_CASE(  7, a4, 0, slli a4, a0, 2; srli a4, a4, 18; )

  TEST_CASE(  8, a4, 1, csrrw a0, 0x015, x0; srli a4, a0, 30; )
  TEST_CASE(  9, a4, 0, slli a4, a0, 2; srli a4, a4, 2; )

  TEST_CASE( 10, a4, 0, la a3, trap_count; lw a4, 0(a3); )

  .option pop

  TEST_PASSFAIL

  RVTEST_TRAP_HANDLER

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  RVTEST_TRAP_DATA

RVTEST_DATA_END

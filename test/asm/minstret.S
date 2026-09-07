# See LICENSE for license details.

# The counter rules that nothing else can check.

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  RVTEST_INSTALL_TRAP_HANDLER

  TEST_CASE( 2, a2, 4, \
    csrr a0, minstret; \
    nop; \
    nop; \
    nop; \
    csrr a1, minstret; \
    sub  a2, a1, a0; )

  # mcycle advances on its own.

  TEST_CASE( 3, a2, 1, \
    csrr a0, mcycle; \
    nop; \
    csrr a1, mcycle; \
    sltu a2, a0, a1; )

  TEST_CASE( 4, a2, 0, \
    csrr a0, mcycleh; \
    csrr a1, minstreth; \
    or   a2, a0, a1; )

  TEST_CASE( 5, a1, 0x100, \
    li   a0, 0x100; \
    csrw minstret, a0; \
    csrr a1, minstret; )

test_6:
  csrw  mcycle, x0
  csrr  a1, mcycle
  li    x29, 16
  li    TESTNUM, 6
  bgeu  a1, x29, fail

  TEST_PASSFAIL

  RVTEST_TRAP_HANDLER_FATAL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

RVTEST_DATA_END

# See LICENSE for license details.

// A hundred-iteration backward branch aligned so its fall-through crosses a fetch window.

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  RVTEST_INSTALL_TRAP_HANDLER

  TEST_CASE( 2, a1, 100, \
    li   a0, 100; \
    li   a1, 0; \
  1: \
    addi t0, t0, 999; \
    addi t0, t0, 999; \
    addi t0, t0, 999; \
    addi t0, t0, 999; \
    addi t0, t0, -999; \
    addi t0, t0, -999; \
    addi t0, t0, -999; \
    addi t0, t0, -999; \
    addi a1, a1, 1; \
    addi a0, a0, -1; \
    nop; \
    bnez a0, 1b; )

  TEST_PASSFAIL

  RVTEST_TRAP_HANDLER_FATAL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

RVTEST_DATA_END

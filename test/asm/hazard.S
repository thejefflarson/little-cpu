// Direct regression coverage for the stall-only hazard scoreboard.

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  TEST_CASE( 2, x6, 30, \
    li   x1, 10; \
    addi x1, x1, 20; \
    addi x6, x1, 0; \
  )

  // Back-to-back RAW on rs2.
  TEST_CASE( 3, x6, 30, \
    li   x2, 10; \
    addi x2, x2, 20; \
    add  x6, x0, x2; \
  )

  TEST_CASE( 4, x6, 80, \
    li   x1, 20; \
    add  x1, x1, x1; \
    add  x6, x1, x1; \
  )

  TEST_CASE( 5, x6, 0x12345678, \
    la   x1, ldat; \
    lw   x2, 0(x1); \
    addi x6, x2, 0; \
  )

  TEST_CASE( 6, x6, 8, \
    li   x1, 40; \
    li   x2, 5; \
    div  x1, x1, x2; \
    addi x6, x1, 0; \
  )

  TEST_CASE( 7, x6, 0x0ff00ff0, \
    li   x1, 40; \
    li   x2, 5; \
    div  x1, x1, x2; \
    la   x4, ldat2; \
    lw   x6, 0(x4); \
  )

  TEST_PASSFAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

ldat:  .word 0x12345678
ldat2: .word 0x0ff00ff0

RVTEST_DATA_END

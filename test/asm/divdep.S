#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  // A load reading div's own destination register, the very next instruction
  // in program order: the hazard scoreboard must hold it for the divide's
  // real completion, not for whatever in_has_result claims one cycle early.
  TEST_CASE( 2, x6, 0x1234, \
    li   x10, 0x30000; \
    li   x11, 2; \
    li   x12, 0x18000; \
    li   x7, 0x1234; \
    sw   x7, 0(x12); \
    div  x5, x10, x11; \
    lw   x6, 0(x5); \
  )

  TEST_PASSFAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN
  TEST_DATA
RVTEST_DATA_END

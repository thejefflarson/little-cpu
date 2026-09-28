#include "riscv_test.h"
#include "test_macros.h"

// Above the machine timer's four words, below the 1 MB the linker script may grow into.
#define UNMAPPED 0x00040000

RVTEST_RV64U
RVTEST_CODE_BEGIN

  RVTEST_INSTALL_TRAP_HANDLER

test_2:
  li    TESTNUM, 2
  la    x1, trap_cause
  sw    x0, 0(x1)
  li    x10, UNMAPPED * 2
  li    x11, 2
  // A load reading div's own destination, whose result lands outside every mapped window.
  div   x5, x10, x11
  .option push
  .option norvc
  lw    x6, 0(x5)
  .option pop
  la    x1, trap_cause
  lw    x13, 0(x1)
  li    x29, 5
  bne   x13, x29, fail

  TEST_PASSFAIL

  RVTEST_TRAP_HANDLER

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN
  TEST_DATA
  RVTEST_TRAP_DATA
RVTEST_DATA_END

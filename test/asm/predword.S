#include "riscv_test.h"
#include "test_macros.h"

// A compressed loop that fits in one word, reached through a target that is 4 mod 8 so the
// pair holding it is not 8-aligned. Guessing its own word as a target would queue that word
// twice: the second copy is what decode reads after the loop's not-taken exit.

RVTEST_RV64U
RVTEST_CODE_BEGIN

  .option push
  .option norelax
  c.nop
  .option norvc

  li TESTNUM, 2
  j entry
  nop
  nop
entry:
  addi a0, x0, 2
  .option push
  .option rvc
1:
  c.addi a0, -1
  c.bnez a0, 1b
  .option pop
  bne a0, x0, fail

  // The same loop with three passes, so a guess that is right once still meets the exit.
  li TESTNUM, 3
  j entry2
  nop
  nop
  nop
entry2:
  addi a1, x0, 3
  .option push
  .option rvc
1:
  c.addi a1, -1
  c.bnez a1, 1b
  .option pop
  bne a1, x0, fail

  .option pop

  TEST_PASSFAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

RVTEST_DATA_END

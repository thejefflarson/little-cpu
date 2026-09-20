#include "riscv_test.h"
#include "test_macros.h"

// A guess from a pair's first word queues that word alone; the rest of each pair stores
// TESTNUM to tohost if reached. The last guess is wrong, so its fall-through is refetched.

RVTEST_RV64U
RVTEST_CODE_BEGIN

  .option push
  .option norelax
  c.nop
  .option norvc

  lui  t0, %hi(tohost)
  li TESTNUM, 2
  li a0, 0

  jal x0, target1
  sw   TESTNUM, %lo(tohost)(t0)
target1:
  addi a1, x0, 1
  nop

  .option rvc
  c.j target2
  .option norvc
  sw   TESTNUM, %lo(tohost)(t0)
  .option rvc
  c.nop
  .option norvc
target2:
  addi a2, x0, 2
  nop

  .option rvc
  c.nop
  c.j target3
  .option norvc
  sw   TESTNUM, %lo(tohost)(t0)
target3:
  addi a3, x0, 3
  nop

  .option rvc
  c.bnez a0, target1
  .option norvc
  addi a4, x0, 4
  .option rvc
  c.nop
  .option norvc

  li t1, 1
  bne a1, t1, fail
  li t1, 2
  bne a2, t1, fail
  li t1, 3
  bne a3, t1, fail
  li t1, 4
  bne a4, t1, fail

  .option pop

  TEST_PASSFAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

RVTEST_DATA_END

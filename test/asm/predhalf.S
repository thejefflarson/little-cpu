#include "riscv_test.h"
#include "test_macros.h"

// A guess taken from a pair's first word queues that word alone. The rest of each pair
// here is a store of TESTNUM to tohost, which reports failure if the guessed path ever
// reaches it; t0 holds tohost's upper bits so the store is one word. Pairs stay 8-aligned
// because every target is. The last pair's guess is wrong -- a backward c.bnez not taken --
// so its fall-through, in the word the half push dropped, must be refetched.

RVTEST_RV64U
RVTEST_CODE_BEGIN

  .option push
  .option norelax
  c.nop
  .option norvc

  lui  t0, %hi(tohost)
  li TESTNUM, 2
  li a0, 0

  // A jal filling the first word.
  jal x0, target1
  sw   TESTNUM, %lo(tohost)(t0)
target1:
  addi a1, x0, 1
  nop

  // A c.j at lane 0.
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

  // A c.j at lane 1, behind a compressed instruction.
  .option rvc
  c.nop
  c.j target3
  .option norvc
  sw   TESTNUM, %lo(tohost)(t0)
target3:
  addi a3, x0, 3
  nop

  // A backward c.bnez at lane 0, guessed taken and not taken.
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

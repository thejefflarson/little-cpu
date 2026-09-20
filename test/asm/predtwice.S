#include "riscv_test.h"
#include "test_macros.h"

// One guess in flight: the pair at a guessed branch's target ends in a jal that arrives
// before the branch resolves. Had it replaced the record, the not-taken branch would fall
// into the target pair's words as its own successor.

RVTEST_RV64U
RVTEST_CODE_BEGIN

  .option push
  .option norelax
  c.nop
  .option norvc

  li TESTNUM, 2
  lui  t1, %hi(zero_word)
  addi t1, t1, %lo(zero_word)
  lui  t2, %hi(one_word)
  addi t2, t2, %lo(one_word)
  j entry
  nop

back1:
  nop
  jal x0, poison

entry:
  lw a0, 0(t1)
  bne a0, x0, back1
  addi a1, x0, 7
  li t0, 7
  bne a1, t0, fail
  li TESTNUM, 3

back2:
  nop
  jal x0, joined
entry2:
  lw a0, 0(t2)
  bne a0, x0, back2
  j fail
joined:
  addi a2, x0, 9
  li t0, 9
  bne a2, t0, fail
  j done

poison:
  nop
  nop
  nop
  nop
  sll  TESTNUM, TESTNUM, 1
  or   TESTNUM, TESTNUM, 1
  lui  t0, %hi(tohost)
  sw   x0, %lo(tohost)+4(t0)
  sw   TESTNUM, %lo(tohost)(t0)
1:
  j 1b

done:
  .option pop

  TEST_PASSFAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

  .align 2
zero_word:
  .word 0
one_word:
  .word 1

RVTEST_DATA_END

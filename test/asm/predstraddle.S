#include "riscv_test.h"
#include "test_macros.h"

// A 32-bit backward branch straddling two words, guessed taken, must pop both its words;
// then the lanes-1-2 shape not taken, whose fall-through the guess abandoned.

RVTEST_RV64U
RVTEST_CODE_BEGIN

  .option push
  .option norelax
  c.nop
  .option norvc

  lui  t0, %hi(tohost)
  li TESTNUM, 2
  li a0, 1
  j entry1
  nop

back1:
  addi a1, x0, 1
  j cont1
entry1:
  .option rvc
  c.nop
  .option norvc
  bne a0, x0, back1
  .option rvc
  c.nop
  .option norvc
cont1:
  li t1, 1
  bne a1, t1, fail

  li TESTNUM, 3
  j entry2
back2:
  addi a2, x0, 2
  j cont2
entry2:
  nop
  .option rvc
  c.nop
  .option norvc
  bne a0, x0, back2
  .option rvc
  c.nop
  .option norvc
  sw   TESTNUM, %lo(tohost)(t0)
cont2:
  li t1, 2
  bne a2, t1, fail

  li TESTNUM, 4
  li a0, 0
  j entry3
  nop
back3:
  j fail
  nop
entry3:
  .option rvc
  c.nop
  .option norvc
  bne a0, x0, back3
  .option rvc
  c.li a3, 3
  .option norvc
  li t1, 3
  bne a3, t1, fail

  .option pop

  TEST_PASSFAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

RVTEST_DATA_END

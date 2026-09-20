#include "riscv_test.h"
#include "test_macros.h"

// A taken branch abandons the pairs fetch already asked for. Each of those pairs ends in a
// jal to `poison`, so a guess formed off a discarded pair would steer fetch there while
// decode believes it is reading the second pair after `target`. Alignment is counted by
// hand under norvc (every instruction four bytes after the c.nop), because .balign
// leaves its padding to a linker pass.

RVTEST_RV64U
RVTEST_CODE_BEGIN

  .option push
  .option norelax
  c.nop
  .option norvc

  li TESTNUM, 2
branch1:
  beq x0, x0, target
  nop
  nop
  jal x0, poison
  nop
  jal x0, poison
  nop
  jal x0, poison
  nop
  jal x0, poison
  nop
  jal x0, poison
  nop
  jal x0, poison

target:
  addi a0, x0, 1
  addi a1, x0, 2
  addi a2, x0, 3
  addi a3, x0, 4
  addi a4, x0, 5
  addi a5, x0, 6
  li t0, 1
  bne a0, t0, fail
  li t0, 2
  bne a1, t0, fail
  li t0, 3
  bne a2, t0, fail
  li t0, 4
  bne a3, t0, fail
  li t0, 5
  bne a4, t0, fail
  li t0, 6
  bne a5, t0, fail

  // The same shape with the branch in a pair's second word.
pair2:
  li TESTNUM, 3
  beq x0, x0, target2
  nop
  jal x0, poison
  nop
  jal x0, poison
  nop
  jal x0, poison
  nop
  jal x0, poison
  nop
  jal x0, poison
  nop
  jal x0, poison

target2:
  addi a0, x0, 7
  addi a1, x0, 8
  addi a2, x0, 9
  addi a3, x0, 10
  addi a4, x0, 11
  addi a5, x0, 12
  li t0, 7
  bne a0, t0, fail
  li t0, 8
  bne a1, t0, fail
  li t0, 9
  bne a2, t0, fail
  li t0, 10
  bne a3, t0, fail
  li t0, 11
  bne a4, t0, fail
  li t0, 12
  bne a5, t0, fail
  j done

  // Reached only through a stale guess; reports the failure without a pc-relative address,
  // because whoever runs these words is not at the pc they were linked for.
poison:
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

RVTEST_DATA_END

#include "riscv_test.h"
#include "test_macros.h"

// A load from text steals the fetch port for a cycle. When that cycle is the one a guess
// commits on, the retry must re-present the guessed target, not the successor the guess
// abandoned: that successor here is a pair that reports failure if it ever executes. Each
// block is sixteen words with the load at a different word, so the steal sweeps across the
// commit; the jal is always the twelfth word, the end of a pair.

#define N0
#define N1 nop;
#define N2 nop; nop;
#define N3 nop; nop; nop;
#define N4 nop; nop; nop; nop;
#define N5 nop; nop; nop; nop; nop;
#define N6 nop; nop; nop; nop; nop; nop;
#define N7 nop; nop; nop; nop; nop; nop; nop;
#define N8 nop; nop; nop; nop; nop; nop; nop; nop;

#define BLOCK(n, pre, post)                                                  \
  li TESTNUM, n;                                                            \
  pre                                                                       \
  lw t2, 0(t1);                                                             \
  post                                                                      \
  nop;                                                                      \
  jal x0, 1f;                                                               \
  lui  t0, %hi(tohost);                                                     \
  sw   TESTNUM, %lo(tohost)(t0);                                            \
1:                                                                          \
  addi a0, a0, 1;                                                           \
  nop;

RVTEST_RV64U
RVTEST_CODE_BEGIN

  .option push
  .option norelax
  c.nop
  .option norvc

  lui  t1, %hi(_start)
  addi t1, t1, %lo(_start)
  li a0, 0
blocks:
  BLOCK(3,  N0, N8)
  BLOCK(5,  N1, N7)
  BLOCK(7,  N2, N6)
  BLOCK(9,  N3, N5)
  BLOCK(11, N4, N4)
  BLOCK(13, N5, N3)
  BLOCK(15, N6, N2)
  BLOCK(17, N7, N1)
  BLOCK(19, N8, N0)
blocks_end:


  li TESTNUM, 2
  li t0, 9
  bne a0, t0, fail

  .option pop

  TEST_PASSFAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

RVTEST_DATA_END

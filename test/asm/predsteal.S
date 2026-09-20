#include "riscv_test.h"
#include "test_macros.h"

// A load from text steals the fetch port; on a commit cycle the retry must re-present the
// target, not the abandoned successor, which here reports failure. The load sweeps
// across sixteen-word blocks; the jal is always the twelfth word.

#define N0
#define N1 nop;
#define N2 nop; nop;
#define N3 nop; nop; nop;
#define N4 nop; nop; nop; nop;
#define N5 nop; nop; nop; nop; nop;
#define N6 nop; nop; nop; nop; nop; nop;
#define N7 nop; nop; nop; nop; nop; nop; nop;
#define N8 nop; nop; nop; nop; nop; nop; nop; nop;

#define BLOCK(n, pre, load, post)                                            \
  li TESTNUM, n;                                                            \
  pre                                                                       \
  load                                                                      \
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

  lui  t1, 0x1
  addi t1, t1, 0
  li a0, 0
#define L1 lw t2, 0(t1);
#define L2 lw t2, 0(t1); lw a1, 4(t1);

blocks:
  BLOCK(3,  N0, L1, N8)
  BLOCK(5,  N1, L1, N7)
  BLOCK(7,  N2, L1, N6)
  BLOCK(9,  N3, L1, N5)
  BLOCK(11, N4, L1, N4)
  BLOCK(13, N5, L1, N3)
  BLOCK(15, N6, L1, N2)
  BLOCK(17, N7, L1, N1)
  BLOCK(19, N8, L1, N0)
  BLOCK(21, N0, L2, N7)
  BLOCK(23, N1, L2, N6)
  BLOCK(25, N2, L2, N5)
  BLOCK(27, N3, L2, N4)
  BLOCK(29, N4, L2, N3)
  BLOCK(31, N5, L2, N2)
  BLOCK(33, N6, L2, N1)
  BLOCK(35, N7, L2, N0)
blocks_end:


  li TESTNUM, 2
  li t0, 17
  bne a0, t0, fail

  .option pop

  TEST_PASSFAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

RVTEST_DATA_END

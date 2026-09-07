// The one dual smoke program: enough to prove the harness sees two harts, and
// deliberately no more.

#include "riscv_test.h"

#define ITERS 16
// How many times hart 0 polls for hart 1's completion flag before giving up.
#define POLL_LIMIT 256

RVTEST_RV64U
RVTEST_CODE_BEGIN

  csrr    t0, mhartid
  bnez    t0, hart1

  // Release hart 1, which is spinning on `mailbox` below.
  la      t1, mailbox
  li      t2, 1
  sw      t2, 0(t1)

  la      t1, count0
  li      t2, ITERS
0:
  lw      t3, 0(t1)
  addi    t3, t3, 1
  sw      t3, 0(t1)
  addi    t2, t2, -1
  bnez    t2, 0b

  la      t1, done1
  li      t2, POLL_LIMIT
1:
  lw      t3, 0(t1)
  bnez    t3, 2f
  addi    t2, t2, -1
  bnez    t2, 1b

2:
  la      t1, count0
  lw      a0, 0(t1)
  la      t1, count1
  lw      a1, 0(t1)
  add     a2, a0, a1
  la      t1, total
  sw      a2, 0(t1)

  li      TESTNUM, 2
  li      t4, 2 * ITERS
  bne     a2, t4, fail
  RVTEST_PASS

fail:
  RVTEST_FAIL

hart1:
  la      t1, mailbox
3:
  lw      t3, 0(t1)
  beqz    t3, 3b

  la      t1, count1
  li      t2, ITERS
4:
  lw      t3, 0(t1)
  addi    t3, t3, 1
  sw      t3, 0(t1)
  addi    t2, t2, -1
  bnez    t2, 4b

  la      t1, done1
  li      t2, 1
  sw      t2, 0(t1)

5:
  j       5b

RVTEST_CODE_END

RVTEST_DATA_BEGIN
  .align 2
  .global mailbox
mailbox:
  .word 0
  .global count0
count0:
  .word 0
  .global count1
count1:
  .word 0
  .global done1
done1:
  .word 0
  .global total
total:
  .word 0
RVTEST_DATA_END

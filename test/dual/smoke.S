# The one dual smoke program: enough to prove the harness sees two harts, and
# deliberately no more. The torture programs -- contended atomics, the LR/SC
# steal, the mutation pairings -- are a separate piece of work and do not belong
# in a file whose job is to make a harness falsifiable.
#
# It lives under test/dual/ and not test/asm/ because both single-hart legs glob
# that directory and take everything in it to BE the suite. A program that only
# terminates when two harts run it would be graded as a hang there.
#
# THE SHARED COUNT IS THE POINT. Each hart adds ITERS to a counter of its own and
# hart 0 sums the two into `total`, which the runner reads back with
# `--report-word`. A working dual run reports 2 * ITERS; a run in which hart 1
# never started reports ITERS. That difference is the only thing that can tell a
# harness watching two harts from a harness watching one, and test/dual_smoke.sh
# builds this program both ways to force it.
#
# The two counters are separate on purpose. One counter incremented by both
# harts with lw/addi/sw races, and a program whose answer depends on the
# interleaving cannot grade anything. Making that increment atomic is what the
# torture programs are for.

#include "riscv_test.h"

#define ITERS 16
# How many times hart 0 polls for hart 1's completion flag before giving up.
# Bounded rather than infinite so the run with hart 1 held in reset REACHES its
# verdict and stores `total` -- an unbounded poll would time out instead, and a
# timeout says nothing about how many harts were counting.
#define POLL_LIMIT 256

RVTEST_RV64U
RVTEST_CODE_BEGIN

  csrr    t0, mhartid
  bnez    t0, hart1

  # ---- hart 0 ----------------------------------------------------------
  # Release hart 1, which is spinning on `mailbox` below.
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

  # Wait for hart 1 to finish, but only for a bounded number of passes.
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

  # Stored BEFORE the comparison, so the number the runner reads back is the
  # number this program really counted whichever way the comparison goes.
  li      TESTNUM, 2
  li      t4, 2 * ITERS
  bne     a2, t4, fail
  RVTEST_PASS

fail:
  RVTEST_FAIL

  # ---- hart 1 ----------------------------------------------------------
  # Parking is polling: this core decodes no `wfi`, so a hart with nothing to do
  # spins on a word of RAM. It writes nothing until it is released, so hart 0
  # owns every location up to that point.
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

  # Park again, for good. Hart 1 never writes `tohost`: one hart reaches the
  # verdict, so a run cannot end with two answers in it.
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

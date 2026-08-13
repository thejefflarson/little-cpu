# See LICENSE for license details.

#*****************************************************************************
# lrsclock.S
#-----------------------------------------------------------------------------
#
# An LR/SC critical section with the machine timer running underneath it.
#
# THE PROPERTY. A reservation is NOT cleared on a trap or on an mret. It is
# cleared by a store-conditional and by a write of this hart's to the reserved
# word, and by nothing else. That is what lets a constrained LR/SC sequence keep
# the specification's eventual-success guarantee while interrupts are running: a
# handler that never touches the lock word cannot make the sequence fail, so a
# retry loop is not livelocked by a periodic timer. The reference model was
# measured to do the same, which is why the two agree about it wherever they can
# be compared at all.
#
# THE RETRY LOOP IS BOUNDED, AND THAT IS THE POINT OF THE WATCHDOG. An unbounded
# retry against a core that cleared the reservation on entry to the handler
# would spin until test/run_tests.sh's 5000-cycle budget ran out and be graded
# TIMEOUT -- the same verdict a hung fetch gets, at a place in the program the
# verdict does not name. The counters below turn that into a numbered failure
# instead. They are the only reason a program in this suite may loop on a
# condition the hardware is supposed to make true.
#
# WHAT PART 1 ASSERTS AND PART 2 DOES NOT. Part 1 spins between the lr.w and the
# sc.w until the handler has really run, so the trap is inside the window by
# construction and the sc.w is not retried: it must succeed the first time. Part
# 2 is the retry loop, and a core that dropped the reservation at every trap
# would still finish it -- one retry per interrupt. So the sharp assertion is
# part 1's and part 2 is the shape a lock actually has.
#
# CO-SIMULATION CANNOT REACH THIS PROGRAM, for the reason test/asm/mtimer.S and
# test/asm/mtimermask.S cannot be reached either: the Sail model's timer is at a
# different address with a different tick period, so a store that arms this
# platform's timer lands nowhere on the model and the wait below never ends
# there. test/COSIM_EXPECTED_FAIL carries it as INCONCLUSIVE SAIL-LIMIT --
# nothing was compared, which is a weaker entry than a divergence and is
# recorded as one.
#

#include "riscv_test.h"
#include "test_macros.h"

# How many acquisitions part 2 may make before giving up on the second
# interrupt, and how many times one acquisition may retry its sc.w. Both are
# far above what the hardware needs and are watchdogs rather than expectations.
#define ACQUIRE_LIMIT 200
#define RETRY_LIMIT   64

RVTEST_RV64U
RVTEST_CODE_BEGIN

  RVTEST_INSTALL_TIMER_HANDLER
  # mtimecmp resets to zero, so mtip is asserted before the enables go on.
  RVTEST_DISARM_TIMER
  RVTEST_ENABLE_MTIE
  RVTEST_ENABLE_MIE

  #-------------------------------------------------------------
  # Part 1: a reservation held across a real timer trap.
  #
  # The delay is long enough that the interrupt cannot land before the lr.w,
  # and the wait after it is unbounded on purpose -- on this machine it ends on
  # the first pass after the handler has run.
  #-------------------------------------------------------------

  la    a0, lock
  RVTEST_ARM_TIMER( 40 )
  lr.w  a1, (a0)

irq_wait:
  la    a3, irq_count
  lw    a4, 0(a3)
  beqz  a4, irq_wait

  li    a5, 0x0000a5a5
  sc.w  a2, a5, (a0)

  # The store-conditional succeeded with a whole trap entry and mret between it
  # and its lr.w.
  TEST_CASE(  2, a2, 0, nop; )
  TEST_CASE(  3, a4, 0x0000a5a5, la a3, lock; lw a4, 0(a3); )

  # ...and the trap really was a machine timer interrupt, so test 2 did not pass
  # because nothing had happened.
  TEST_CASE(  4, a4, 1, la a3, irq_count; lw a4, 0(a3); )
  TEST_CASE(  5, a4, 0x80000007, la a3, irq_cause; lw a4, 0(a3); )

  # The lr.w read the word the disarm sequence left alone.
  TEST_CASE(  6, a1, 0x0ff00ff0, nop; )

  #-------------------------------------------------------------
  # Part 2: the lock loop. Each acquisition increments the word under lr.w/sc.w
  # and retries a bounded number of times; the loop runs until a SECOND timer
  # interrupt has been taken, so at least one acquisition overlaps a live timer.
  #
  # s1 counts acquisitions completed and s3 the retries within the current one.
  # The memory word is graded against s1 afterwards rather than against a
  # constant, because how many times the loop goes round depends on when the
  # interrupt lands.
  #-------------------------------------------------------------

  la    a0, lock
  li    s1, 0
  RVTEST_ARM_TIMER( 24 )

acquire:
  li    s3, 0
retry:
  lr.w  a4, (a0)
  addi  a5, a4, 1
  sc.w  a6, a5, (a0)
  beqz  a6, acquired

  addi  s3, s3, 1
  li    a7, RETRY_LIMIT
  blt   s3, a7, retry
  # The reservation is being lost by something that is not a write to the lock
  # word. A numbered failure, not a hang.
  li    TESTNUM, 7
  j     fail

acquired:
  # The word this acquisition's lr.w read, kept for the check below.
  mv    s4, a4
  addi  s1, s1, 1
  la    a3, irq_count
  lw    a3, 0(a3)
  li    a7, 2
  bge   a3, a7, acquired_all
  li    a7, ACQUIRE_LIMIT
  blt   s1, a7, acquire
  # The second interrupt never arrived, so nothing below would be a statement
  # about a lock loop under a live timer.
  li    TESTNUM, 8
  j     fail

acquired_all:

  # The word holds its part 1 value plus one per completed acquisition. This is
  # the loop's own bookkeeping checked against memory, which is the only place
  # an acquisition that "succeeded" without storing would show. Computed at run
  # time because how many times the loop goes round depends on when the
  # interrupt lands.
test_9:
  la    a3, lock
  lw    a4, 0(a3)
  li    a5, 0x0000a5a5
  add   a5, a5, s1
  li    TESTNUM, 9
  bne   a4, a5, fail

  # The last lr.w saw the word one short of what the last sc.w left.
test_10:
  addi  a5, s4, 1
  li    TESTNUM, 10
  bne   a4, a5, fail

  # At least one acquisition really happened, so tests 9 and 10 are not both
  # true of a loop that never ran.
test_11:
  li    TESTNUM, 11
  beqz  s1, fail

  # ...and the second interrupt really was taken during the loop.
  TEST_CASE( 12, a4, 2, la a3, irq_count; lw a4, 0(a3); )

  TEST_PASSFAIL

  RVTEST_TIMER_HANDLER

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  RVTEST_TIMER_DATA

  .align 2
lock:
  .word 0x0ff00ff0

RVTEST_DATA_END

# See LICENSE for license details.

// An LR/SC critical section with the machine timer running underneath it.

#include "riscv_test.h"
#include "test_macros.h"

// How many acquisitions part 2 may make before giving up on the second interrupt, and how
// many times one acquisition may retry its sc.w.
#define ACQUIRE_LIMIT 200
#define RETRY_LIMIT   64

RVTEST_RV64U
RVTEST_CODE_BEGIN

  RVTEST_INSTALL_TIMER_HANDLER
  // mtimecmp resets to zero, so mtip is asserted before the enables go on.
  RVTEST_DISARM_TIMER
  RVTEST_ENABLE_MTIE
  RVTEST_ENABLE_MIE

  la    a0, lock
  RVTEST_ARM_TIMER( 40 )
  lr.w  a1, (a0)

irq_wait:
  la    a3, irq_count
  lw    a4, 0(a3)
  beqz  a4, irq_wait

  li    a5, 0x0000a5a5
  sc.w  a2, a5, (a0)

  TEST_CASE(  2, a2, 0, nop; )
  TEST_CASE(  3, a4, 0x0000a5a5, la a3, lock; lw a4, 0(a3); )

  TEST_CASE(  4, a4, 1, la a3, irq_count; lw a4, 0(a3); )
  TEST_CASE(  5, a4, 0x80000007, la a3, irq_cause; lw a4, 0(a3); )

  TEST_CASE(  6, a1, 0x0ff00ff0, nop; )

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
  li    TESTNUM, 7
  j     fail

acquired:
  mv    s4, a4
  addi  s1, s1, 1
  la    a3, irq_count
  lw    a3, 0(a3)
  li    a7, 2
  bge   a3, a7, acquired_all
  li    a7, ACQUIRE_LIMIT
  blt   s1, a7, acquire
  li    TESTNUM, 8
  j     fail

acquired_all:

test_9:
  la    a3, lock
  lw    a4, 0(a3)
  li    a5, 0x0000a5a5
  add   a5, a5, s1
  li    TESTNUM, 9
  bne   a4, a5, fail

test_10:
  addi  a5, s4, 1
  li    TESTNUM, 10
  bne   a4, a5, fail

test_11:
  li    TESTNUM, 11
  beqz  s1, fail

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

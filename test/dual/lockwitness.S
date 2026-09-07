#include "dual_test.h"

// An LR/SC spinlock, and a witness that says whether it excluded anything.

// Rounds per hart. The critical section is about twenty instructions, so the longest
// legitimate wait is well inside DUALTEST_SPIN_LIMIT.
#define LOCK_N     16
#define LOCK_TOTAL (2 * LOCK_N)

#define TN_OCCUPIED 3
#define TN_TORN     4
#define TN_STUCK    5
#define TN_JOIN     6

DUALTEST_CODE_BEGIN

  jal   ra, lock_burst
  DUALTEST_JOIN( TN_JOIN )

  TEST_CASE( 7, a4, LOCK_TOTAL, la t0, wcount; lw a4, 0(t0); )
  TEST_CASE( 8, a4, 0, la t0, lock; lw a4, 0(t0); )
  TEST_CASE( 9, a4, 0, la t0, occupancy; lw a4, 0(t0); )

  TEST_PASSFAIL

hart1:
  jal   ra, lock_burst
  DUALTEST_HART1_DONE

lock_burst:
  mv    s2, ra
  li    s0, LOCK_N
1:
  la    a0, lock
  jal   ra, lock_acquire
  jal   ra, critical
  la    a0, lock
  sw    x0, 0(a0)
  addi  s0, s0, -1
  bnez  s0, 1b
  mv    ra, s2
  ret

lock_acquire:
  li    t2, DUALTEST_SPIN_LIMIT
  li    t1, 1
1:
  addi  t2, t2, -1
  beqz  t2, lock_stuck
  lr.w  t0, (a0)
  bnez  t0, 1b
  sc.w  t0, t1, (a0)
  bnez  t0, 1b
  ret

lock_stuck:
  li    TESTNUM, TN_STUCK
  j     fail

critical:
  la    a1, occupancy
  lw    a2, 0(a1)
  li    TESTNUM, TN_OCCUPIED
  bnez  a2, fail
  li    a2, 1
  sw    a2, 0(a1)

  la    a3, wcount
  lw    a4, 0(a3)
  addi  a5, a4, 1
  sw    a5, 0(a3)
  lw    a6, 0(a3)
  li    TESTNUM, TN_TORN
  bne   a6, a5, fail

  sw    x0, 0(a1)
  ret

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  DUALTEST_DATA

  .align 2
lock:
  .word 0
wcount:
  .word 0
occupancy:
  .word 0

RVTEST_DATA_END

#include "dual_test.h"

// An LR/SC spinlock, and a witness that says whether it excluded anything.
//
// WHY THE WITNESS. A lock that never grants hangs, and a hang is
// indistinguishable from a harness that never released the second hart, so a
// bare lock test grades the harness and not the lock. A lock that grants BOTH
// harts at once does not hang at all -- it runs to completion and passes.
// Neither failure is visible in the lock word.
//
// So the critical section carries a shared counter incremented NON-atomically:
// load, add one, store. Two harts inside at once lose updates, and the total
// must be exactly 2*LOCK_N. That is the witness the ticket for this program
// asks for, and it is the one that survives any interleaving.
//
// It carries a second, louder one beside it: an occupancy flag set on entry and
// cleared on exit, read before it is set. A hart that finds it already set is
// standing in a critical section with the other hart, and fails there rather
// than at the total afterwards. And the non-atomic increment reads its own
// result back, so a store lost while this hart is between its load and its
// store is caught at the instruction as well as in the sum. The three witness
// the same property at three distances; the total is the one that cannot be
// dodged by luck.
//
// Every spin here is bounded, so a lock that is never released is a FAIL at a
// named number and not a TIMEOUT.

// Rounds per hart. The critical section is about twenty instructions, so the
// longest legitimate wait is well inside DUALTEST_SPIN_LIMIT.
#define LOCK_N     16
#define LOCK_TOTAL (2 * LOCK_N)

// 1 belongs to test/dual/dual_test.h.
#define TN_OCCUPIED 3
#define TN_TORN     4
#define TN_STUCK    5
#define TN_JOIN     6

DUALTEST_CODE_BEGIN

  jal   ra, lock_burst
  DUALTEST_JOIN( TN_JOIN )

  # The witness, read back into a register because nothing else compares memory.
  TEST_CASE( 7, a4, LOCK_TOTAL, la t0, wcount; lw a4, 0(t0); )
  # Both harts released what they took.
  TEST_CASE( 8, a4, 0, la t0, lock; lw a4, 0(t0); )
  TEST_CASE( 9, a4, 0, la t0, occupancy; lw a4, 0(t0); )

  TEST_PASSFAIL

hart1:
  jal   ra, lock_burst
  DUALTEST_HART1_DONE

  #-------------------------------------------------------------
  # Both harts run everything below. Control cannot fall in: hart 0 ends in
  # TEST_PASSFAIL's loop and hart 1 in DUALTEST_HART1_DONE's.
  #-------------------------------------------------------------

  # LOCK_N times: take the lock, run the critical section, release it.
  # Clobbers a0-a6, t0-t2, s0, s2 and TESTNUM.
lock_burst:
  mv    s2, ra
  li    s0, LOCK_N
1:
  la    a0, lock
  jal   ra, lock_acquire
  jal   ra, critical
  # The release is a plain store, so the other hart's `lr.w` sees zero on its
  # next attempt and this hart's own reservation is already spent.
  la    a0, lock
  sw    x0, 0(a0)
  addi  s0, s0, -1
  bnez  s0, 1b
  mv    ra, s2
  ret

  # a0 = &lock. Spins on lr.w until the word reads zero, then claims it with
  # sc.w and retries the whole thing if the store-conditional failed. The
  # sequence between the lr.w and the sc.w is a branch and nothing else, which
  # is what the specification's constrained form asks for.
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

  # The critical section. Clobbers a1-a6 and TESTNUM.
critical:
  # Nobody else may be in here. This is the loud witness: it fails at the breach
  # rather than at the total.
  la    a1, occupancy
  lw    a2, 0(a1)
  li    TESTNUM, TN_OCCUPIED
  bnez  a2, fail
  li    a2, 1
  sw    a2, 0(a1)

  # The quiet one. Deliberately not an AMO: load, add, store, and read the
  # result back. Under exclusion the read-back is this hart's own value; without
  # it, the other hart's store lands in between and one of the two increments is
  # gone.
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

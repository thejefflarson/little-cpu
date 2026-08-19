#include "dual_test.h"

// Two harts add 1 to one shared word AMO_N times each. The word must end at
// exactly 2*AMO_N, because that is what a lost update destroys: an AMO whose
// read and write are not indivisible lets the other hart's increment land in
// between, and the pair of them then count as one.
//
// TWO CHECKS, AND THE SECOND IS THE STRONGER ONE. The final count says how many
// increments survived. The sum of the values the AMOs RETURNED says which ones:
// across both harts the eleven-encoding definition makes those returns exactly
// the multiset {0, 1, ... 2N-1} in some interleaving, so their total is
// N*(2N-1) whatever order the harts ran in. A design that dropped one update
// and returned one value twice has the right count and the wrong sum. Each hart
// carries its own partial sum and hart 0 adds them after the join.
//
// Every AMO's return is read into a register on the spot and range-checked
// there as well, so a torn read is a FAIL at the instruction rather than an
// arithmetic surprise at the end.

#define AMO_N     64
// 2 * AMO_N.
#define AMO_TOTAL 128
// 0 + 1 + ... + (2*AMO_N - 1), which is AMO_N * (2*AMO_N - 1).
#define AMO_SUM   8128

// 1 and 2 belong to test/dual/dual_test.h.
#define TN_RANGE  3
#define TN_JOIN   4

DUALTEST_CODE_BEGIN

  jal   ra, amo_burst
  DUALTEST_JOIN( TN_JOIN )

  # Nothing between the join and here writes the counter, so this is the whole
  # execution's result and not a prefix of it.
  TEST_CASE( 5, a4, AMO_TOTAL, la t0, counter; lw a4, 0(t0); )

  # test/cosim.cc compares registers and never compares memory, so the sum has
  # to come back through one too.
  TEST_CASE( 6, a5, AMO_SUM, la t0, sum1; lw a5, 0(t0); add a5, a5, a3; )

  TEST_PASSFAIL

hart1:
  jal   ra, amo_burst
  la    t0, sum1
  sw    a3, 0(t0)
  DUALTEST_HART1_DONE

  #-------------------------------------------------------------
  # Both harts run this. Returns the sum of the values its AMOs returned in a3,
  # and clobbers a0, a1, a2, a4, a6 and TESTNUM.
  #
  # Control cannot fall in: hart 0 ends in TEST_PASSFAIL's loop and hart 1 in
  # DUALTEST_HART1_DONE's.
  #-------------------------------------------------------------
amo_burst:
  la    a0, counter
  li    a1, 1
  li    a3, 0
  li    a4, AMO_N
  li    a6, AMO_TOTAL
  li    TESTNUM, TN_RANGE
1:
  amoadd.w a2, a1, (a0)
  # The counter runs 0 .. 2N-1 and an AMO returns the value it replaced, so no
  # return can reach 2N. One that does is a read that did not come from this
  # word or did not come from one cycle of it.
  bgeu  a2, a6, fail
  add   a3, a3, a2
  addi  a4, a4, -1
  bnez  a4, 1b
  ret

DUALTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  DUALTEST_DATA

  .align 2
counter:
  .word 0
sum1:
  .word 0

RVTEST_DATA_END

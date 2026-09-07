#include "dual_test.h"

// Two harts add 1 to one shared word AMO_N times each.

#define AMO_N     64
#define AMO_TOTAL (2 * AMO_N)
// 0 + 1 + ... + (AMO_TOTAL - 1).
#define AMO_SUM   (AMO_N * (AMO_TOTAL - 1))

#define TN_RANGE  3
#define TN_JOIN   4

DUALTEST_CODE_BEGIN

  jal   ra, amo_burst
  DUALTEST_JOIN( TN_JOIN )

  TEST_CASE( 5, a4, AMO_TOTAL, la t0, counter; lw a4, 0(t0); )

  TEST_CASE( 6, a5, AMO_SUM, la t0, sum1; lw a5, 0(t0); add a5, a5, a3; )

  TEST_PASSFAIL

hart1:
  jal   ra, amo_burst
  la    t0, sum1
  sw    a3, 0(t0)
  DUALTEST_HART1_DONE

amo_burst:
  la    a0, counter
  li    a1, 1
  li    a3, 0
  li    a4, AMO_N
  li    a6, AMO_TOTAL
  li    TESTNUM, TN_RANGE
1:
  amoadd.w a2, a1, (a0)
  bgeu  a2, a6, fail
  add   a3, a3, a2
  addi  a4, a4, -1
  bnez  a4, 1b
  ret

RVTEST_CODE_END

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

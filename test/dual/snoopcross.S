#include "dual_test.h"

// One hart's write must end the other hart's reservation.

#define SC_SENTINEL 0x0ff00ff0
#define SC_AVAL     0x0000cafe
#define SC_BVAL     0x0000beef

// Sweep rounds, and the two ordered rounds in front of them.
#define SC_SWEEP    32
#define SC_ROUNDS   (SC_SWEEP + 2)
// The round in which hart 1 writes the other word.
#define SC_OTHER    2

// 1 belongs to test/dual/dual_test.h.
#define TN_W1       3
#define TN_W2       4
#define TN_W3       5
#define TN_LOST     6
#define TN_NO_KILL  7
#define TN_NO_WIN   8
#define TN_JOIN     9
#define TN_BWAIT    10

DUALTEST_CODE_BEGIN

  li    s0, 0
  la    s1, shared
  la    s3, go
  li    a5, SC_BVAL

  // Round 1 -- ordered. The write is long finished before the sc.w attempts.

  addi  s0, s0, 1
  li    a7, SC_SENTINEL
  sw    a7, 0(s1)
  lr.w  a1, (s1)
  sw    s0, 0(s3)
  DUALTEST_WAIT_REG( TN_W1, done, s0 )
  li    a6, SC_AVAL
  sc.w  a2, a6, (s1)

  TEST_CASE( 11, a2, 1, nop; )
  TEST_CASE( 12, a4, SC_BVAL, lw a4, 0(s1); )

  // Round 2 -- hart 1 writes somewhere else, so the reservation must survive.

  addi  s0, s0, 1
  li    a7, SC_SENTINEL
  sw    a7, 0(s1)
  lr.w  a1, (s1)
  sw    s0, 0(s3)
  DUALTEST_WAIT_REG( TN_W2, done, s0 )
  li    a6, SC_AVAL
  sc.w  a2, a6, (s1)

  TEST_CASE( 13, a2, 0, nop; )
  TEST_CASE( 14, a4, SC_AVAL, lw a4, 0(s1); )

  li    s5, 0
  li    s6, 0
  li    s7, 0
sweep:
  addi  s0, s0, 1
  li    a7, SC_SENTINEL
  sw    a7, 0(s1)
  lr.w  a1, (s1)
  li    a6, SC_AVAL
  sw    s0, 0(s3)
  slli  t0, s5, 2
  la    t1, sled_end
  sub   t1, t1, t0
  jr    t1

  .option push
  .option norvc
sled:
  .rept SC_SWEEP
  nop
  .endr
sled_end:
  sc.w  a2, a6, (s1)
  .option pop

  beqz  a2, 4f
  addi  s6, s6, 1
  j     5f
4:
  addi  s7, s7, 1
5:
  DUALTEST_WAIT_REG( TN_W3, done, s0 )
  lw    a4, 0(s1)
  li    TESTNUM, TN_LOST
  bne   a4, a5, fail

  addi  s5, s5, 1
  li    a7, SC_SWEEP
  blt   s5, a7, sweep

  li    TESTNUM, TN_NO_KILL
  beqz  s6, fail
  li    TESTNUM, TN_NO_WIN
  beqz  s7, fail

  DUALTEST_JOIN( TN_JOIN )
  TEST_CASE( 15, a4, SC_BVAL, lw a4, 0(s1); )

  TEST_PASSFAIL

hart1:
  li    s0, 0
  la    s1, shared
  la    s2, other
  la    s4, done
1:
  addi  s0, s0, 1
  DUALTEST_WAIT_REG( TN_BWAIT, go, s0 )
  li    a6, SC_BVAL
  li    a7, SC_OTHER
  beq   s0, a7, 2f
  sw    a6, 0(s1)
  j     3f
2:
  sw    a6, 0(s2)
3:
  sw    s0, 0(s4)
  li    a7, SC_ROUNDS
  blt   s0, a7, 1b
  DUALTEST_HART1_DONE

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  DUALTEST_DATA

  .align 2
shared:
  .word 0
other:
  .word 0
go:
  .word 0
done:
  .word 0

RVTEST_DATA_END

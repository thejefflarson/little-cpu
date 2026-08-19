#include "dual_test.h"

// One hart's write must end the other hart's reservation. Hart 0 reserves a
// word, hart 1 writes it, and hart 0's store-conditional has to notice.
//
// THREE SHAPES, AND THE THIRD IS THE ONE THAT MATTERS.
//
// Round 1 is ordered: hart 0 reserves, hands over, waits for hart 1's write to
// be acknowledged, and only then attempts the sc.w. It must fail, and the word
// must still hold hart 1's value. That is the whole snoop, tested with tens of
// cycles of slack, and a design with no snoop at all fails it.
//
// Round 2 is the other direction: hart 1 writes a DIFFERENT word. The sc.w must
// succeed. Without it, a design that dropped every reservation on any foreign
// write would pass round 1 and look correct.
//
// Rounds 3 upwards are a sweep, and they exist for the off-by-one. A snoop that
// arrives one cycle late passes round 1 with room to spare: by then the write
// is ancient. It goes wrong only when the sc.w is granted the bus on the cycle
// after the write landed -- the reservation has not been told yet, the sc.w
// succeeds, and it overwrites a store that was already committed. So hart 0
// hands over and then delays a computed number of cycles before its sc.w,
// walking its attempt one cycle at a time across hart 1's write.
//
// WHAT THE SWEEP ASSERTS IS NOT WHO WON. Either order is legal: if the sc.w got
// there first the reservation was still good and the store-conditional was
// entitled to store; if the write got there first the reservation was gone. The
// invariant across both is that hart 1's value is what survives -- the write
// either came second, or it came first and killed the reservation. A sc.w that
// stores AFTER the write is the only way the word ends up holding hart 0's
// value, and that is exactly what the late snoop produces.
//
// AND THE SWEEP GRADES ITSELF. A sweep whose whole range sits on one side of
// the crossing tests nothing and passes. So hart 0 counts the rounds each side
// won and requires both to be nonzero: at least one round where the sc.w failed
// (the sweep reached hart 1's write) and at least one where it succeeded (it
// started before it). If a change to the machine moves the crossing outside the
// swept range these go red, which is the correct outcome -- the program has
// stopped covering the case it exists for and the sweep needs re-measuring, not
// the assertion relaxing.
//
// SC_SWEEP AND THE TWO COUNTS ARE OWED A MEASUREMENT. Nothing has run this
// program; the range is sized from the handshake's shape -- hart 1 notices the
// handover inside one four-instruction spin and stores immediately, so the
// crossing sits some tens of cycles after the handover -- and not from a
// waveform.

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

  #-------------------------------------------------------------
  # Round 1 -- ordered. The write is long finished before the sc.w attempts.
  #-------------------------------------------------------------

  addi  s0, s0, 1
  li    a7, SC_SENTINEL
  sw    a7, 0(s1)
  lr.w  a1, (s1)
  # The handover is a store to a different word, which leaves the reservation
  # alone -- the single-hart suite is what establishes that half.
  sw    s0, 0(s3)
  DUALTEST_WAIT_REG( TN_W1, done, s0 )
  li    a6, SC_AVAL
  sc.w  a2, a6, (s1)

  TEST_CASE( 11, a2, 1, nop; )
  TEST_CASE( 12, a4, SC_BVAL, lw a4, 0(s1); )

  #-------------------------------------------------------------
  # Round 2 -- hart 1 writes somewhere else, so the reservation must survive.
  #-------------------------------------------------------------

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

  #-------------------------------------------------------------
  # Rounds 3 upwards -- the sweep.
  #-------------------------------------------------------------

  li    s5, 0
  li    s6, 0
  li    s7, 0
sweep:
  addi  s0, s0, 1
  li    a7, SC_SENTINEL
  sw    a7, 0(s1)
  lr.w  a1, (s1)
  li    a6, SC_AVAL
  # The handover. Everything from here to the sc.w is a fixed number of cycles
  # plus the sled, so incrementing s5 moves the attempt exactly one cycle later
  # against a write whose timing is anchored to this store.
  sw    s0, 0(s3)
  slli  t0, s5, 2
  la    t1, sled_end
  sub   t1, t1, t0
  jr    t1

  # Uncompressed, so the entry point is sled_end - 4*s5 and each nop is one
  # issue cycle. A nop cannot stall, which is what makes the step exactly one.
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
  # Wait for the write to be acknowledged before reading, so the round's result
  # is the whole round's and not a snapshot taken mid-handshake.
  DUALTEST_WAIT_REG( TN_W3, done, s0 )
  lw    a4, 0(s1)
  li    TESTNUM, TN_LOST
  bne   a4, a5, fail

  addi  s5, s5, 1
  li    a7, SC_SWEEP
  blt   s5, a7, sweep

  # The sweep's own coverage, asserted rather than hoped for.
  li    TESTNUM, TN_NO_KILL
  beqz  s6, fail
  li    TESTNUM, TN_NO_WIN
  beqz  s7, fail

  DUALTEST_JOIN( TN_JOIN )
  TEST_CASE( 15, a4, SC_BVAL, lw a4, 0(s1); )

  TEST_PASSFAIL

  #-------------------------------------------------------------
  # Hart 1: wait for the handover, write, acknowledge. It writes the other word
  # in exactly one round and the shared word in every other.
  #-------------------------------------------------------------
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

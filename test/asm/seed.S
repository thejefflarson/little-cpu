# See LICENSE for license details.

// The Zkr `seed` CSR (0x015): what a read returns in each status, and that a read
// consumes the word. The harness drives a fast stand-in for the board's oscillator, so
// ES16 arrives inside the runner's cycle budget. The reference model's source is ready
// from the first read, so this program is baselined as disagreeing with it.

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  RVTEST_INSTALL_TRAP_HANDLER

  .option push
  .option norvc

  // The source is still in its power-on test; a read says so and carries no entropy.
  TEST_CASE(  2, a4, 0, csrrw a0, 0x015, x0; srli a4, a0, 30; )
  TEST_CASE(  3, a4, 0, slli a4, a0, 2; srli a4, a4, 2; )

  // Poll, with a bound, until a word is ready. Never DEAD, and never entropy outside ES16.
  li    a6, 1500
poll:
  csrrw a0, 0x015, x0
  srli  a3, a0, 30
  li    a2, 3
  li    TESTNUM, 4
  beq   a3, a2, fail
  li    a2, 2
  beq   a3, a2, ready
  slli  a4, a0, 16
  li    TESTNUM, 5
  bnez  a4, fail
  addi  a6, a6, -1
  li    TESTNUM, 6
  beqz  a6, fail
  j     poll
ready:

  // ES16: reserved and custom bits are zero.
  TEST_CASE(  7, a4, 0, slli a4, a0, 2; srli a4, a4, 18; )

  // The read consumed the word: the next one reports WAIT with nothing in the field.
  TEST_CASE(  8, a4, 1, csrrw a0, 0x015, x0; srli a4, a0, 30; )
  TEST_CASE(  9, a4, 0, slli a4, a0, 2; srli a4, a4, 2; )

  // None of it trapped.
  TEST_CASE( 10, a4, 0, la a3, trap_count; lw a4, 0(a3); )

  .option pop

  TEST_PASSFAIL

  RVTEST_TRAP_HANDLER

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  RVTEST_TRAP_DATA

RVTEST_DATA_END

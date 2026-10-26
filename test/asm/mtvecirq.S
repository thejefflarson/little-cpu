# See LICENSE for license details.

// The machine timer interrupt under a vectored mtvec: it enters at BASE + 0x1C, and no
// other slot of the table runs. A direct-mode mtvec sends the same interrupt to BASE. The
// reference model's timer sits at another address, so this program is INCONCLUSIVE
// SAIL-LIMIT and mtvecmode.S carries the exception half of the comparison.

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  la    t0, vec_table
  ori   t0, t0, 1
  csrw  mtvec, t0

  RVTEST_DISARM_TIMER
  RVTEST_ENABLE_MTIE
  RVTEST_ENABLE_MIE

  RVTEST_ARM_TIMER( 40 )

  .option push
  .option norvc
  li    a5, 0
1:
  addi  a5, a5, 1
  li    a6, 60
  blt   a5, a6, 1b
  .option pop

  TEST_CASE( 2, a4, 1, la a3, irq_count; lw a4, 0(a3); )
  TEST_CASE( 3, a4, 0x80000007, la a3, irq_cause; lw a4, 0(a3); )
  TEST_CASE( 4, a5, 60, nop; )

  // Direct mode: the same interrupt enters at BASE, which here is the handler itself.
  la    t0, timer_handler
  csrw  mtvec, t0

  RVTEST_ARM_TIMER( 40 )

  .option push
  .option norvc
  li    a5, 0
1:
  addi  a5, a5, 1
  li    a6, 60
  blt   a5, a6, 1b
  .option pop

  TEST_CASE( 5, a4, 2, la a3, irq_count; lw a4, 0(a3); )
  TEST_CASE( 6, a5, 60, nop; )

  // MODE 2 and 3 are reserved; the spec lets any legal value read back, and this core
  // reads MODE 0 or 1. Only MODE's legality is graded: the reference model keeps the old MODE.
  TEST_CASE( 7, a0, 1, \
    la t1, vec_other; ori t1, t1, 2; csrw mtvec, t1; \
    csrr a1, mtvec; andi a1, a1, 2; seqz a0, a1; )

  TEST_CASE( 8, a0, 1, \
    la t1, vec_other; ori t1, t1, 3; csrw mtvec, t1; \
    csrr a1, mtvec; andi a1, a1, 2; seqz a0, a1; )

  TEST_PASSFAIL

  RVTEST_TIMER_HANDLER

  .option push
  .option norvc
  .balign 64
vec_table:
  .rept 7
  j     fail
  .endr
  j     timer_handler
  .rept 8
  j     fail
  .endr
vec_other:
  j     fail
  .option pop

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  RVTEST_TIMER_DATA

RVTEST_DATA_END

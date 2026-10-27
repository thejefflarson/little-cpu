# See LICENSE for license details.

// A vectored mtvec takes the timer at BASE + 0x1C and a direct one at BASE; reserved MODE
// writes are graded for legality only, since the reference model keeps the old MODE.

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

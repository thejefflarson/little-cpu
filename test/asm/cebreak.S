# See LICENSE for license details.

// `c.ebreak` raises a BREAKPOINT (mcause 3), not an illegal instruction.

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  // mtvec resets to 0, which is _start — install before faulting.
  RVTEST_INSTALL_TRAP_HANDLER

  // c.ebreak at pc % 4 == 0.

  .option push
  .option rvc
  .align 2
cebrk_aligned:
  c.ebreak
  c.nop
  .option pop

  TEST_CASE( 2, a4, 3, la a3, trap_cause; lw a4, 0(a3); )
  TEST_CASE( 3, a4, 1, la a3, trap_count; lw a4, 0(a3); )

  TEST_CASE( 4, a4, 0, la a3, cebrk_aligned; andi a4, a3, 3; )

test_5:
  la    a3, trap_epc
  lw    a4, 0(a3)
  la    x29, cebrk_aligned
  li    TESTNUM, 5
  bne   a4, x29, fail

  la    a3, trap_cause
  sw    x0, 0(a3)

  .option push
  .option rvc
  .align 2
  c.nop
cebrk_odd:
  c.ebreak
  c.nop
  .option pop

  TEST_CASE( 6, a4, 3, la a3, trap_cause; lw a4, 0(a3); )
  TEST_CASE( 7, a4, 2, la a3, trap_count; lw a4, 0(a3); )

  TEST_CASE( 8, a4, 2, la a3, cebrk_odd; andi a4, a3, 3; )

test_9:
  la    a3, trap_epc
  lw    a4, 0(a3)
  la    x29, cebrk_odd
  li    TESTNUM, 9
  bne   a4, x29, fail

  TEST_CASE( 10, a5, 0x5a5, \
    li a5, 0x5a5; \
    .option push; .option rvc; .align 2; \
    c.ebreak; c.nop; \
    .option pop; )

  TEST_CASE( 11, a4, 3, la a3, trap_count; lw a4, 0(a3); )

test_12:
  csrr  a0, minstret
  .option push
  .option rvc
  .align 2
  c.ebreak
  c.nop
  .option pop
  csrr  a1, minstret
  sub   a2, a1, a0

  la    a5, trap_handler
  la    a6, trap_handler_end
  sub   a5, a6, a5
  srli  a5, a5, 2
  addi  x29, a5, 1
  li    TESTNUM, 12
  bne   a2, x29, fail

  TEST_CASE( 13, a4, 4, la a3, trap_count; lw a4, 0(a3); )

  TEST_PASSFAIL

  RVTEST_TRAP_HANDLER

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  RVTEST_TRAP_DATA

RVTEST_DATA_END

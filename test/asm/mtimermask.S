# See LICENSE for license details.

// The two masks, and the two stalls an interrupt has to wait out.

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  RVTEST_INSTALL_TIMER_HANDLER

  // mstatus.MIE clear, mie.MTIE set.

  RVTEST_DISARM_TIMER
  RVTEST_ENABLE_MTIE
  RVTEST_DISABLE_MIE
  RVTEST_ARM_TIMER( 20 )

  TEST_CASE( 2, a4, 0, \
    nop; nop; nop; nop; nop; nop; nop; nop; \
    nop; nop; nop; nop; nop; nop; nop; nop; \
    nop; nop; nop; nop; nop; nop; nop; nop; \
    la a3, irq_count; lw a4, 0(a3); )

  TEST_CASE( 3, a0, 0x80, csrr a0, mip; )

  // Unmasking takes the interrupt that was already pending.
  RVTEST_ENABLE_MIE

  TEST_CASE( 4, a4, 1, \
    nop; nop; nop; nop; \
    la a3, irq_count; lw a4, 0(a3); )
  TEST_CASE( 5, a4, 0x80000007, la a3, irq_cause; lw a4, 0(a3); )

  RVTEST_DISABLE_MTIE
  RVTEST_ARM_TIMER( 20 )

  TEST_CASE( 6, a4, 1, \
    nop; nop; nop; nop; nop; nop; nop; nop; \
    nop; nop; nop; nop; nop; nop; nop; nop; \
    nop; nop; nop; nop; nop; nop; nop; nop; \
    la a3, irq_count; lw a4, 0(a3); )

  TEST_CASE( 7, a0, 0x80, csrr a0, mip; )
  TEST_CASE( 8, a0, 0,    csrr a0, mie; )

  RVTEST_ENABLE_MTIE

  TEST_CASE( 9, a4, 2, \
    nop; nop; nop; nop; \
    la a3, irq_count; lw a4, 0(a3); )

  RVTEST_ARM_TIMER( 40 )

  li    a0, 1000
  li    a1, 7
  div   a2, a0, a1
  rem   a3, a0, a1
  li    a4, -1000
  div   a5, a4, a1
  rem   a6, a4, a1

  TEST_CASE( 10, a2, 142,  nop; )
  TEST_CASE( 11, a3, 6,    nop; )
  TEST_CASE( 12, a5, -142, nop; )
  TEST_CASE( 13, a6, -6,   nop; )

  TEST_CASE( 14, a4, 3, la a3, irq_count; lw a4, 0(a3); )

  li    a0, 0x5a5a5a5a
  csrw  mscratch, a0

  RVTEST_ARM_TIMER( 30 )

  csrr  a1, mscratch
  csrr  a2, mscratch
  csrr  a3, mscratch
  csrr  a5, mscratch
  csrr  a6, mscratch
  csrr  a7, mscratch

  TEST_CASE( 15, a1, 0x5a5a5a5a, nop; )
  TEST_CASE( 16, a7, 0x5a5a5a5a, nop; )
  TEST_CASE( 17, a4, 4, la a3, irq_count; lw a4, 0(a3); )

  TEST_CASE( 18, a0, 0x5a5a5a5a, csrr a0, mscratch; )

  TEST_PASSFAIL

  RVTEST_TIMER_HANDLER

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  RVTEST_TIMER_DATA

RVTEST_DATA_END

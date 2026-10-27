# See LICENSE for license details.

// The software interrupt's two masks, and its priority over the timer when both are pending.

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  RVTEST_INSTALL_SOFT_HANDLER

  // mie.MSIE clear, mstatus.MIE set: pending and unmasked by nothing.
  RVTEST_DISARM_TIMER
  RVTEST_ENABLE_MIE
  RVTEST_RAISE_MSIP

  TEST_CASE( 2, a4, 0, \
    nop; nop; nop; nop; nop; nop; nop; nop; \
    la a3, irq_count; lw a4, 0(a3); )
  TEST_CASE( 3, a0, 0x8, csrr a0, mip; )
  TEST_CASE( 4, a0, 0,   csrr a0, mie; )

  RVTEST_ENABLE_MSIE
  TEST_CASE( 5, a4, 1, \
    nop; nop; nop; nop; \
    la a3, irq_count; lw a4, 0(a3); )
  TEST_CASE( 6, a4, 0x80000003, la a3, irq_cause; lw a4, 0(a3); )

  RVTEST_DISABLE_MIE
  RVTEST_RAISE_MSIP
  TEST_CASE( 7, a4, 1, \
    nop; nop; nop; nop; nop; nop; nop; nop; \
    la a3, irq_count; lw a4, 0(a3); )
  TEST_CASE( 8, a0, 0x8, csrr a0, mip; )
  RVTEST_DISABLE_MSIE
  RVTEST_ENABLE_MIE
  TEST_CASE( 9, a4, 1, \
    nop; nop; nop; nop; nop; nop; nop; nop; \
    la a3, irq_count; lw a4, 0(a3); )
  RVTEST_CLEAR_MSIP

  // Both pending and both enabled: the software interrupt first, then the timer.
  RVTEST_INSTALL_LOG_HANDLER
  la    a3, irq_count
  sw    x0, 0(a3)
  RVTEST_DISABLE_MIE
  RVTEST_ENABLE_MSIE
  RVTEST_ENABLE_MTIE
  RVTEST_ARM_TIMER( 10 )
  RVTEST_RAISE_MSIP
  .rept 24
  nop
  .endr
  TEST_CASE( 10, a0, 0x88, csrr a0, mip; )
  RVTEST_ENABLE_MIE
  .rept 8
  nop
  .endr
  TEST_CASE( 11, a4, 2, la a3, irq_count; lw a4, 0(a3); )
  TEST_CASE( 12, a4, 0x80000003, la a3, irq_log; lw a4, 0(a3); )
  TEST_CASE( 13, a4, 0x80000007, la a3, irq_log; lw a4, 4(a3); )

  la    a3, irq_count
  sw    x0, 0(a3)
  RVTEST_DISABLE_MIE
  RVTEST_DISABLE_MSIE
  RVTEST_ARM_TIMER( 10 )
  RVTEST_RAISE_MSIP
  .rept 24
  nop
  .endr
  RVTEST_ENABLE_MIE
  .rept 8
  nop
  .endr
  TEST_CASE( 14, a4, 1, la a3, irq_count; lw a4, 0(a3); )
  TEST_CASE( 15, a4, 0x80000007, la a3, irq_log; lw a4, 0(a3); )

  TEST_PASSFAIL

  RVTEST_SOFT_HANDLER
  RVTEST_LOG_HANDLER

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  RVTEST_TIMER_DATA
  RVTEST_SOFT_DATA

RVTEST_DATA_END

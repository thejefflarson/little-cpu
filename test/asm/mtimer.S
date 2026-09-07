# See LICENSE for license details.

# mtimer.S The machine timer interrupt, taken end to end: armed from software, asserted
# by rtl/timer.v, gated by mie.MTIE and mstatus.MIE, and committed at a decode boundary.

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  # mtvec resets to 0, which is _start.
  RVTEST_INSTALL_TIMER_HANDLER

  # Nothing fires until it is armed AND enabled.

  RVTEST_DISARM_TIMER
  RVTEST_ENABLE_MTIE
  RVTEST_ENABLE_MIE

  TEST_CASE( 2, a4, 0, \
    nop; nop; nop; nop; nop; nop; nop; nop; \
    la a3, irq_count; lw a4, 0(a3); )

  # mip.MTIP is the platform's line, and it is low while mtimecmp is out of reach.
  TEST_CASE( 3, a0, 0,    csrr a0, mip; )
  TEST_CASE( 4, a0, 0x80, csrr a0, mie; )

  # Armed: the interrupt arrives inside a spin loop.

  RVTEST_ARM_TIMER( 40 )

spin_start:
  .option push
  .option norvc
  li    a5, 0
1:
  addi  a5, a5, 1
  li    a6, 60
  blt   a5, a6, 1b
  .option pop
spin_end:

  TEST_CASE( 5, a4, 1, la a3, irq_count; lw a4, 0(a3); )

  # Interrupt, machine timer: bit 31 set, exception code 7.
  TEST_CASE( 6, a4, 0x80000007, la a3, irq_cause; lw a4, 0(a3); )

test_7:
  la    a3, irq_epc
  lw    a4, 0(a3)
  la    x29, spin_start
  li    TESTNUM, 7
  bltu  a4, x29, fail
  la    x29, spin_end
  li    TESTNUM, 7
  bgeu  a4, x29, fail

  TEST_CASE( 8, a5, 60, nop; )

  TEST_CASE( 9, a0, 0, csrr a0, mip; )

  TEST_CASE( 10, a4, 1, \
    nop; nop; nop; nop; nop; nop; nop; nop; \
    nop; nop; nop; nop; nop; nop; nop; nop; \
    la a3, irq_count; lw a4, 0(a3); )

  TEST_CASE( 11, a4, 0, la a3, irq_mstatus; lw a4, 0(a3); andi a4, a4, 0x8; )
  TEST_CASE( 12, a4, 0x80, la a3, irq_mstatus; lw a4, 0(a3); andi a4, a4, 0x80; )

  TEST_CASE( 13, a1, 0x8, csrr a0, mstatus; andi a1, a0, 0x8; )

  RVTEST_ARM_TIMER( 40 )

  .option push
  .option norvc
  li    a5, 0
1:
  addi  a5, a5, 1
  li    a6, 60
  blt   a5, a6, 1b
  .option pop

  TEST_CASE( 14, a4, 2, la a3, irq_count; lw a4, 0(a3); )
  TEST_CASE( 15, a4, 0x80000007, la a3, irq_cause; lw a4, 0(a3); )

  la    a3, irq_count
  sw    x0, 0(a3)
  la    a3, irq_limit
  li    a4, 3
  sw    a4, 0(a3)

  RVTEST_INSTALL_STICKY_TIMER_HANDLER
  RVTEST_ARM_TIMER( 20 )

  .option push
  .option norvc
  li    a5, 0
1:
  addi  a5, a5, 1
  li    a6, 60
  blt   a5, a6, 1b
  .option pop

  TEST_CASE( 16, a4, 3, la a3, irq_count; lw a4, 0(a3); )

  TEST_CASE( 17, a5, 60, nop; )

  RVTEST_INSTALL_TIMER_HANDLER

  la    a3, irq_count
  sw    x0, 0(a3)

  li    t0, MTIMER_BASE
  li    t1, 2
  sw    t1, MTIMECMPH_OFFSET(t0)    # mtimecmp = {2, 0x10}
  li    t1, 0x10
  sw    t1, MTIMECMP_OFFSET(t0)
  li    t1, 1
  sw    t1, 4(t0)                   # mtime = {1, 0x50}
  li    t1, 0x50
  sw    t1, 0(t0)

  li    t1, 1
  sw    t1, MTIMECMPH_OFFSET(t0)
  nop
  nop
  nop
  nop
  li    t1, 0xfffffff0
  sw    t1, MTIMECMP_OFFSET(t0)

  TEST_CASE( 18, a4, 1, \
    nop; nop; nop; nop; \
    la a3, irq_count; lw a4, 0(a3); )

  TEST_CASE( 19, a4, 0x80000007, la a3, irq_cause; lw a4, 0(a3); )

  la    a3, irq_count
  sw    x0, 0(a3)

  li    t0, MTIMER_BASE
  li    t1, 2
  sw    t1, MTIMECMPH_OFFSET(t0)
  li    t1, 0x10
  sw    t1, MTIMECMP_OFFSET(t0)
  li    t1, 1
  sw    t1, 4(t0)
  li    t1, 0x50
  sw    t1, 0(t0)

  li    t1, -1
  sw    t1, MTIMECMP_OFFSET(t0)     # low = all ones: no smaller than the old
  li    t1, 1
  sw    t1, MTIMECMPH_OFFSET(t0)    # high = new: no smaller than the new
  nop
  nop
  nop
  nop
  li    t1, 0xfffffff0
  sw    t1, MTIMECMP_OFFSET(t0)     # low = new

  TEST_CASE( 20, a4, 0, \
    nop; nop; nop; nop; \
    la a3, irq_count; lw a4, 0(a3); )

  TEST_CASE( 21, a0, 0, csrr a0, mip; )

  li    t0, MTIMER_BASE
  sw    x0, 4(t0)
  sw    x0, 0(t0)

  RVTEST_ARM_TIMER( 40 )

  .option push
  .option norvc
  li    a5, 0
1:
  addi  a5, a5, 1
  li    a6, 60
  blt   a5, a6, 1b
  .option pop

  TEST_CASE( 22, a4, 1, la a3, irq_count; lw a4, 0(a3); )

  TEST_PASSFAIL

  RVTEST_TIMER_HANDLER
  RVTEST_STICKY_TIMER_HANDLER

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  RVTEST_TIMER_DATA

RVTEST_DATA_END

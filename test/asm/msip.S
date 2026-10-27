# See LICENSE for license details.

// The machine software interrupt: raised and cleared by a store to the platform's msip word,
// taken with cause 3 at BASE or, under a vectored mtvec, at BASE + 0xC.

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  RVTEST_INSTALL_SOFT_HANDLER

  RVTEST_DISARM_TIMER
  RVTEST_ENABLE_MSIE
  RVTEST_ENABLE_MIE

  TEST_CASE( 2, a4, 0, \
    nop; nop; nop; nop; nop; nop; nop; nop; \
    la a3, irq_count; lw a4, 0(a3); )
  TEST_CASE( 3, a0, 0,   csrr a0, mip; )
  TEST_CASE( 4, a0, 0x8, csrr a0, mie; )
  TEST_CASE( 5, a0, 0,   li a1, MSIP_BASE; lw a0, 0(a1); )

  RVTEST_RAISE_MSIP

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

  TEST_CASE( 6, a4, 1, la a3, irq_count; lw a4, 0(a3); )
  TEST_CASE( 7, a4, 0x80000003, la a3, irq_cause; lw a4, 0(a3); )

test_8:
  la    a3, irq_epc
  lw    a4, 0(a3)
  la    x29, spin_start
  li    TESTNUM, 8
  bltu  a4, x29, fail
  la    x29, spin_end
  li    TESTNUM, 8
  bgeu  a4, x29, fail

  TEST_CASE( 9, a5, 60, nop; )
  TEST_CASE( 10, a0, 0, csrr a0, mip; )
  TEST_CASE( 11, a0, 0, li a1, MSIP_BASE; lw a0, 0(a1); )

  // Raised under a clear mstatus.MIE it reads back, only its low bit is stored, and
  // clearing it again before MIE returns leaves nothing to take.
  RVTEST_DISABLE_MIE
  TEST_CASE( 12, a0, 1, li a1, MSIP_BASE; li a2, 0xffffffff; sw a2, 0(a1); lw a0, 0(a1); )
  TEST_CASE( 13, a0, 0x8, csrr a0, mip; )
  TEST_CASE( 14, a0, 0, li a1, MSIP_BASE; li a2, 0xfffffffe; sw a2, 0(a1); lw a0, 0(a1); )
  TEST_CASE( 15, a0, 0, csrr a0, mip; )
  RVTEST_ENABLE_MIE
  TEST_CASE( 16, a4, 1, \
    nop; nop; nop; nop; nop; nop; nop; nop; \
    la a3, irq_count; lw a4, 0(a3); )

  // Raised under a clear MIE, it is taken the moment MIE returns.
  RVTEST_DISABLE_MIE
  RVTEST_RAISE_MSIP
  nop
  nop
  RVTEST_ENABLE_MIE
  TEST_CASE( 17, a4, 2, \
    nop; nop; nop; nop; nop; nop; nop; nop; \
    la a3, irq_count; lw a4, 0(a3); )
  TEST_CASE( 18, a0, 0, csrr a0, mip; )

  // Under a vectored mtvec it enters at BASE + 0xC.
  la    t0, vec_table
  ori   t0, t0, 1
  csrw  mtvec, t0
  RVTEST_RAISE_MSIP
  nop
  nop
  nop
  nop
  TEST_CASE( 19, a4, 3, la a3, irq_count; lw a4, 0(a3); )
  TEST_CASE( 20, a4, 0x80000003, la a3, irq_cause; lw a4, 0(a3); )

  TEST_PASSFAIL

  RVTEST_SOFT_HANDLER

  .option push
  .option norvc
  .balign 64
vec_table:
  .rept 3
  j     fail
  .endr
  j     soft_handler
  .rept 12
  j     fail
  .endr
  .option pop

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  RVTEST_TIMER_DATA

RVTEST_DATA_END

# See LICENSE for license details.

#*****************************************************************************
# csr.S
#-----------------------------------------------------------------------------
#
# The Zicsr access semantics and the CSR set of ADR-0005, checked in band.
# riscv-formal has no spec model for csrr*/mret at the pinned SHA, so the
# per-retire monitor reports nothing about any instruction in this file (see
# riscv_test.h) — every claim here is asserted by the test itself.
#
# No trap is expected anywhere in this file, so it installs the *fatal* handler:
# an unexpected fault reports the live test number instead of restarting the
# program from mtvec = 0 (ADR-0029).
#

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  RVTEST_INSTALL_TRAP_HANDLER

  #-------------------------------------------------------------
  # A read/write CSR round-trips.
  #-------------------------------------------------------------

  TEST_CASE( 2, a1, 0x1234, \
    li a0, 0x1234; \
    csrw mscratch, a0; \
    csrr a1, mscratch; )

  #-------------------------------------------------------------
  # CSRRW returns the old value and installs the new one.
  #-------------------------------------------------------------

  TEST_CASE( 3, a1, 0x1234, \
    li a0, 0x5678; \
    csrrw a1, mscratch, a0; )

  TEST_CASE( 4, a1, 0x5678, csrr a1, mscratch; )

  #-------------------------------------------------------------
  # CSRRS with rs1 == x0 suppresses the write entirely (ADR-0005), so it is
  # legal against a read-only CSR — if it raised illegal-instruction instead,
  # the fatal handler below would report this test number.
  #-------------------------------------------------------------

  TEST_CASE( 5, a0, 0x40001104, csrrs a0, misa, x0; )
  TEST_CASE( 6, a1, 0x5678, csrrs a0, mscratch, x0; csrr a1, mscratch; )

  #-------------------------------------------------------------
  # The immediate forms operate on a 5-bit zimm, not on rs1.
  #-------------------------------------------------------------

  TEST_CASE( 7, a1, 0x567f, csrrsi a0, mscratch, 0x1f; csrr a1, mscratch; )
  TEST_CASE( 8, a1, 0x5670, csrrci a0, mscratch, 0x0f; csrr a1, mscratch; )

  # ...and the value returned is the one from before the update: mscratch is
  # 0x5670 on entry and 0x5673 on exit, so only the pre-update read passes.
  TEST_CASE( 9, a0, 0x5670, csrrsi a0, mscratch, 0x03; )

  #-------------------------------------------------------------
  # The read-only identification CSRs (ADR-0005).
  #-------------------------------------------------------------

  TEST_CASE( 10, a0, 0x40001104, csrr a0, misa; )
  TEST_CASE( 11, a0, 0, csrr a0, mvendorid; )
  TEST_CASE( 12, a0, 0, csrr a0, marchid; )
  TEST_CASE( 13, a0, 0, csrr a0, mimpid; )
  TEST_CASE( 14, a0, 0, csrr a0, mhartid; )

  # No interrupt sources and no mtval: all three read as zero, forever.
  TEST_CASE( 15, a2, 0, \
    csrr a0, mie; \
    csrr a1, mip; \
    or   a2, a0, a1; \
    csrr a0, mtval; \
    or   a2, a2, a0; )

  # ...and a write to them does not stick.
  TEST_CASE( 16, a1, 0, \
    li a0, -1; \
    csrw mie, a0; \
    csrr a1, mie; )

  #-------------------------------------------------------------
  # mtvec reads back the handler this test installed.
  #-------------------------------------------------------------

test_17:
  csrr  a0, mtvec
  la    x29, trap_handler
  li    TESTNUM, 17
  bne   a0, x29, fail

  #-------------------------------------------------------------
  # WARL fields. MPP is hardwired to M-mode, so it survives a zero write
  # (ADR-0005), and mepc's bit 0 is always clear — only bit 0, because C makes
  # 2-byte targets legal.
  #-------------------------------------------------------------

  TEST_CASE( 18, a1, 0x1800, \
    csrw mstatus, x0; \
    csrr a0, mstatus; \
    li   a2, 0x1800; \
    and  a1, a0, a2; )

  TEST_CASE( 19, a1, 0x102, \
    li a0, 0x103; \
    csrw mepc, a0; \
    csrr a1, mepc; )

  #-------------------------------------------------------------
  # fence and wfi are NOPs here (ADR-0005): they retire and change nothing.
  #-------------------------------------------------------------

  TEST_CASE( 20, a1, 7, \
    li a1, 7; \
    fence; \
    wfi; )

  TEST_PASSFAIL

  RVTEST_TRAP_HANDLER_FATAL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

RVTEST_DATA_END

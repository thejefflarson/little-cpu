# See LICENSE for license details.

# The Zicsr access semantics and this core's CSR set, checked in band.

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  RVTEST_INSTALL_TRAP_HANDLER

  # A read/write CSR round-trips.

  TEST_CASE( 2, a1, 0x1234, \
    li a0, 0x1234; \
    csrw mscratch, a0; \
    csrr a1, mscratch; )

  TEST_CASE( 3, a1, 0x1234, \
    li a0, 0x5678; \
    csrrw a1, mscratch, a0; )

  TEST_CASE( 4, a1, 0x5678, csrr a1, mscratch; )

  TEST_CASE( 5, a0, 0x40001105, csrrs a0, misa, x0; )
  TEST_CASE( 6, a1, 0x5678, csrrs a0, mscratch, x0; csrr a1, mscratch; )

  TEST_CASE( 7, a1, 0x567f, csrrsi a0, mscratch, 0x1f; csrr a1, mscratch; )
  TEST_CASE( 8, a1, 0x5670, csrrci a0, mscratch, 0x0f; csrr a1, mscratch; )

  TEST_CASE( 9, a0, 0x5670, csrrsi a0, mscratch, 0x03; )

  TEST_CASE( 10, a0, 0x40001105, csrr a0, misa; )
  TEST_CASE( 11, a0, 0, csrr a0, mvendorid; )
  TEST_CASE( 12, a0, 0, csrr a0, marchid; )
  TEST_CASE( 13, a0, 0, csrr a0, mimpid; )
  TEST_CASE( 14, a0, 0, csrr a0, mhartid; )

  TEST_CASE( 15, a0, 0, csrr a0, mtval; )

  TEST_CASE( 16, a0, 0x40001105, \
    csrw misa, x0; \
    csrr a0, misa; )

test_17:
  csrr  a0, mtvec
  la    x29, trap_handler
  li    TESTNUM, 17
  bne   a0, x29, fail

  TEST_CASE( 18, a1, 0x1800, \
    csrw mstatus, x0; \
    csrr a0, mstatus; \
    li   a2, 0x1800; \
    and  a1, a0, a2; )

  TEST_CASE( 19, a1, 0x102, \
    li a0, 0x103; \
    csrw mepc, a0; \
    csrr a1, mepc; )

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

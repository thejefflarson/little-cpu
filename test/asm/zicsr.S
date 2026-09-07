# See LICENSE for license details.

# zicsr.S Zicsr's write- and read-suppression rules are tests on the ENCODING, never on a
# value, and whether an access to a read-only CSR is legal follows from them.

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  # mtvec resets to 0, which is _start — install before faulting.
  RVTEST_INSTALL_TRAP_HANDLER

  li    a5, 0
  .option push
  .option norvc
  csrrs a0, mvendorid, a5
  .option pop

  TEST_CASE( 2, a4, 2, la a3, trap_cause; lw a4, 0(a3); )
  TEST_CASE( 3, a4, 1, la a3, trap_count; lw a4, 0(a3); )

  TEST_CASE( 4, a0, 0, csrrs a0, mvendorid, x0; )
  TEST_CASE( 5, a0, 0, csrrc a0, mvendorid, x0; )

  TEST_CASE( 6, a4, 1, la a3, trap_count; lw a4, 0(a3); )

  TEST_CASE( 7, a0, 0, csrrsi a0, mvendorid, 0; )
  TEST_CASE( 8, a0, 0, csrrci a0, mvendorid, 0; )
  TEST_CASE( 9, a4, 1, la a3, trap_count; lw a4, 0(a3); )

  .option push
  .option norvc
  csrrsi a0, mvendorid, 1
  .option pop

  TEST_CASE( 10, a4, 2, la a3, trap_cause; lw a4, 0(a3); )
  TEST_CASE( 11, a4, 2, la a3, trap_count; lw a4, 0(a3); )

  .option push
  .option norvc
  csrrci a0, mvendorid, 1
  .option pop

  TEST_CASE( 12, a4, 3, la a3, trap_count; lw a4, 0(a3); )

  .option push
  .option norvc
  csrrwi a0, mvendorid, 0
  .option pop

  TEST_CASE( 13, a4, 2, la a3, trap_cause; lw a4, 0(a3); )
  TEST_CASE( 14, a4, 4, la a3, trap_count; lw a4, 0(a3); )

  TEST_CASE( 15, a1, 0, \
    li a0, 0x7fff; \
    csrw mscratch, a0; \
    csrwi mscratch, 0; \
    csrr a1, mscratch; )

  TEST_CASE( 16, a1, 0x1234, \
    li a0, 0x1234; \
    csrrw x0, mscratch, a0; \
    csrr a1, mscratch; )

  TEST_CASE( 17, a1, 0x123f, \
    li a5, 0xffff0000; \
    csrrsi a0, mscratch, 15; \
    csrr a1, mscratch; )

  TEST_CASE( 18, a0, 0x1234, nop; )

  li    a5, 0xffffffff
  csrw  mtval, a5
  csrw  mie, a5
  csrw  mip, a5

  TEST_CASE( 19, a4, 4, la a3, trap_count; lw a4, 0(a3); )

  TEST_CASE( 20, a0, 0x0abc, \
    li a0, 0x0abc; \
    csrw mscratch, a0; \
    li a0, 0x0def; \
    csrrw a0, mscratch, a0; )

  TEST_CASE( 21, a1, 0x0def, csrr a1, mscratch; )

  TEST_CASE( 22, a0, 0x0def, \
    li a0, 0x0f00; \
    csrrs a0, mscratch, a0; )

  TEST_CASE( 23, a1, 0x0fef, csrr a1, mscratch; )

  TEST_CASE( 24, a4, 2, la a3, trap_cause; lw a4, 0(a3); )
  TEST_CASE( 25, a4, 4, la a3, trap_count; lw a4, 0(a3); )

  TEST_PASSFAIL

  RVTEST_TRAP_HANDLER

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  RVTEST_TRAP_DATA

RVTEST_DATA_END

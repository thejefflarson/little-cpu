# See LICENSE for license details.

// Exceptions enter at BASE under both MODEs; no timer, so Sail compares every retire.

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  la    t0, vec_table
  csrw  mtvec, t0

  TEST_CASE( 2, a0, 1, \
    la t0, vec_table; ori t0, t0, 1; csrw mtvec, t0; \
    csrr a1, mtvec; sub a1, a1, t0; seqz a0, a1; )

  .option push
  .option norvc
  ebreak
  .option pop

  TEST_CASE( 3, a4, 3, la a3, trap_cause; lw a4, 0(a3); )
  TEST_CASE( 4, a4, 1, la a3, trap_count; lw a4, 0(a3); )

  .option push
  .option norvc
  ecall
  .option pop

  TEST_CASE( 5, a4, 11, la a3, trap_cause; lw a4, 0(a3); )
  TEST_CASE( 6, a4, 2, la a3, trap_count; lw a4, 0(a3); )

  TEST_CASE( 7, a0, 1, \
    la t0, vec_table; csrw mtvec, t0; \
    csrr a1, mtvec; sub a1, a1, t0; seqz a0, a1; )

  .option push
  .option norvc
  ebreak
  .option pop

  TEST_CASE( 8, a4, 3, la a3, trap_cause; lw a4, 0(a3); )
  TEST_CASE( 9, a4, 3, la a3, trap_count; lw a4, 0(a3); )

  TEST_PASSFAIL

  RVTEST_TRAP_HANDLER

  .option push
  .option norvc
  .balign 64
vec_table:
  j     trap_handler
  .rept 15
  j     fail
  .endr
  .option pop

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  RVTEST_TRAP_DATA

RVTEST_DATA_END

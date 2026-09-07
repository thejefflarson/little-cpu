# See LICENSE for license details.

# Every trap cause this core implements, taken deliberately, recorded by the opt-in
# handler in riscv_test.h, and asserted on from the test body — the cause, the saved pc,
# and what mtval says the trap happened to.

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  # mtvec resets to 0, which is _start — install before faulting.
  RVTEST_INSTALL_TRAP_HANDLER

  # Load address misaligned: cause 4.

  la    a0, tdat
  addi  a0, a0, 1
  li    a1, 0x5a5

mis_lw:
  .option push
  .option norvc
  lw    a1, 0(a0)
  .option pop

  TEST_CASE( 2, a4, 4, la a3, trap_cause; lw a4, 0(a3); )
  TEST_CASE( 3, a4, 1, la a3, trap_count; lw a4, 0(a3); )

test_21:
  la    a3, trap_tval
  lw    a4, 0(a3)
  la    x29, tdat
  addi  x29, x29, 1
  li    TESTNUM, 21
  bne   a4, x29, fail

test_4:
  la    a3, trap_epc
  lw    a4, 0(a3)
  la    x29, mis_lw
  li    TESTNUM, 4
  bne   a4, x29, fail

  # A trapping load writes no register: a1 still holds its sentinel.
  TEST_CASE( 5, a1, 0x5a5, nop; )

  # Store address misaligned: cause 6.

  la    a0, tdat

  .option push
  .option norvc
  sw    a1, 2(a0)
  .option pop

  TEST_CASE(  6, a4, 6, la a3, trap_cause; lw a4, 0(a3); )
  TEST_CASE(  7, a4, 2, la a3, trap_count; lw a4, 0(a3); )

  TEST_CASE(  8, a4, 0x0ff0, la a3, tdat; lw a4, 0(a3); )

test_22:
  la    a3, trap_tval
  lw    a4, 0(a3)
  la    x29, tdat
  addi  x29, x29, 2
  li    TESTNUM, 22
  bne   a4, x29, fail

  .option push
  .option norvc
  ecall
  .option pop

  TEST_CASE(  9, a4, 11, la a3, trap_cause; lw a4, 0(a3); )

  TEST_CASE( 23, a4, 0, la a3, trap_tval; lw a4, 0(a3); )

  .option push
  .option norvc
  ebreak
  .option pop

  TEST_CASE( 10, a4, 3, la a3, trap_cause; lw a4, 0(a3); )

  TEST_CASE( 24, a4, 0, la a3, trap_tval; lw a4, 0(a3); )

  .option push
  .option norvc
  .4byte 0
  .option pop

  TEST_CASE( 11, a4, 2, la a3, trap_cause; lw a4, 0(a3); )

  TEST_CASE( 25, a4, 0, la a3, trap_tval; lw a4, 0(a3); )

  la    a3, trap_cause
  sw    x0, 0(a3)

  .option push
  .option norvc
  csrr  a0, 0x7c0
  .option pop

  TEST_CASE( 12, a4, 2, la a3, trap_cause; lw a4, 0(a3); )

test_26:
  la    a3, trap_tval
  lw    a4, 0(a3)
  li    x29, 0x7c002573        # csrr a0, 0x7c0
  li    TESTNUM, 26
  bne   a4, x29, fail

  la    a3, trap_cause
  sw    x0, 0(a3)

  .option push
  .option norvc
  csrw  mvendorid, t0
  .option pop

  TEST_CASE( 13, a4, 2, la a3, trap_cause; lw a4, 0(a3); )

  la    a0, tdat
  addi  a0, a0, 1
  li    a1, 0x5a5

  .option push
  .option rvc
  .align 2
  c.nop
mis_clw:
  c.lw  a1, 0(a0)
  c.nop
  .option pop

  TEST_CASE( 14, a4, 4, la a3, trap_cause; lw a4, 0(a3); )

test_27:
  la    a3, trap_tval
  lw    a4, 0(a3)
  la    x29, tdat
  addi  x29, x29, 1
  li    TESTNUM, 27
  bne   a4, x29, fail

test_15:
  la    a3, trap_epc
  lw    a4, 0(a3)
  la    x29, mis_clw
  li    TESTNUM, 15
  bne   a4, x29, fail

  TEST_CASE( 16, a4, 2, la a3, mis_clw; andi a4, a3, 3; )

  TEST_CASE( 17, a1, 0x5a5, nop; )

  TEST_CASE( 18, a4, 8, la a3, trap_count; lw a4, 0(a3); )

test_28:
  li    t0, -1
  csrw  mtval, t0
  csrr  a4, mtval
  li    TESTNUM, 28
  bne   a4, t0, fail

test_19:
  la    a3, tdat
  addi  a3, a3, 1

  csrr  a0, minstret
  .option push
  .option norvc
  lw    a4, 0(a3)
  .option pop
  csrr  a1, minstret
  sub   a2, a1, a0

  la    a5, trap_handler
  la    a6, trap_handler_end
  sub   a5, a6, a5
  srli  a5, a5, 2
  addi  x29, a5, 1
  li    TESTNUM, 19
  bne   a2, x29, fail

  TEST_CASE( 20, a4, 9, la a3, trap_count; lw a4, 0(a3); )

  TEST_PASSFAIL

  RVTEST_TRAP_HANDLER

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  RVTEST_TRAP_DATA

  .align 2
tdat:
  .word 0x0ff0
  .word 0x0ff0

RVTEST_DATA_END

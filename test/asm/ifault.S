# See LICENSE for license details.

# ifault.S Instruction access fault, cause 1: a jump to an address the instruction memory
# has nothing at.

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U

.equ bad_pc, 0x00008000

RVTEST_CODE_BEGIN

  la    t0, ifault_handler
  csrw  mtvec, t0

  li    a1, 0x5a5

  li    a2, bad_pc
  .option push
  .option norvc
  jr    a2
  .option pop

  # Control returns here from the handler, not from the jump.
resume:
  TEST_CASE( 2, a4, 1, la a3, trap_cause; lw a4, 0(a3); )
  TEST_CASE( 3, a4, 1, la a3, trap_count; lw a4, 0(a3); )

test_4:
  la    a3, trap_epc
  lw    a4, 0(a3)
  li    x29, bad_pc
  li    TESTNUM, 4
  bne   a4, x29, fail

  TEST_CASE( 5, a1, 0x5a5, nop; )

  TEST_CASE( 6, a4, 7, li a4, 3; addi a4, a4, 4; )

  la    a2, back_in_text
  .option push
  .option norvc
  jr    a2
  .option pop
  j     fail

back_in_text:
  TEST_CASE( 7, a4, 1, la a3, trap_count; lw a4, 0(a3); )

  TEST_PASSFAIL

  .align 2
ifault_handler:
  .option push
  .option norvc
  la      t0, trap_count
  lw      t1, 0(t0)
  addi    t1, t1, 1
  sw      t1, 0(t0)
  csrr    t1, mcause
  la      t0, trap_cause
  sw      t1, 0(t0)
  csrr    t1, mepc
  la      t0, trap_epc
  sw      t1, 0(t0)
  la      t1, resume
  csrw    mepc, t1
  mret
  .option pop

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  RVTEST_TRAP_DATA

RVTEST_DATA_END

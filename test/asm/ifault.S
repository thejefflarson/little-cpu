# See LICENSE for license details.

#*****************************************************************************
# ifault.S
#-----------------------------------------------------------------------------
#
# Instruction access fault, cause 1: a jump to an address the instruction
# memory has nothing at.
#
# The privileged spec retired `vacant` as a region category — such an address is
# an I/O region with no read, write or execute permission — and asks that a
# platform which can do so trap the access precisely. Before this the memory
# answered a word of zeroes, which decodes as an illegal instruction, so the
# core reported cause 2 for an address it had never fetched.
#
# The handler here does NOT resume at mepc + 4, which the shared one in
# riscv_test.h does: mepc + 4 is four bytes further into the same unmapped
# window, so it would fault again forever. It redirects to a label instead,
# which is what a real handler for this cause has to do as well.
#
# The target is past the SIMULATED ROM (test/testbench.v's 16 KB) as well as
# past the part's 8 KB, so the two machines agree about it.
#

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U

# Past both machines' ROM. Not a label in any section -- there is nothing there,
# which is the point -- so it is an absolute address the code loads.
.equ bad_pc, 0x00008000

RVTEST_CODE_BEGIN

  la    t0, ifault_handler
  csrw  mtvec, t0

  # A sentinel, so a handler that returned to the wrong place shows up as a
  # value rather than as a pass.
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

  # mepc holds the address that could not be fetched, which is the jump's
  # target and not the jump.
test_4:
  la    a3, trap_epc
  lw    a4, 0(a3)
  li    x29, bad_pc
  li    TESTNUM, 4
  bne   a4, x29, fail

  # The instruction that never ran wrote nothing.
  TEST_CASE( 5, a1, 0x5a5, nop; )

  # Fetching resumes normally afterwards: this arithmetic is running from ROM
  # again, several instructions past the redirect.
  TEST_CASE( 6, a4, 7, li a4, 3; addi a4, a4, 4; )

  # The other direction. A jump INSIDE the text window is not a fault, so the
  # cases above cannot pass on a core that faults every jump.
  la    a2, back_in_text
  .option push
  .option norvc
  jr    a2
  .option pop
  j     fail

back_in_text:
  TEST_CASE( 7, a4, 1, la a3, trap_count; lw a4, 0(a3); )

  TEST_PASSFAIL

  # Records the same three words riscv_test.h's handler does, then resumes at a
  # fetchable address. `.align 2` satisfies mtvec's WARL base.
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

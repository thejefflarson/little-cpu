#*****************************************************************************
# straddle.S
#-----------------------------------------------------------------------------
#
# ADR-0003 regression: a 32-bit instruction can straddle a 4-byte boundary
# when pc % 4 == 2 (immediately after a compressed instruction). Fetch reads
# the window combinationally every cycle (no aligner FSM, no stall, no
# flush), so this costs nothing structurally; the
# point of this test is to prove it costs nothing *in practice* too, in both
# sim legs, for both a straight-line straddle and a branch redirect landing
# on one.
#
# `.option norvc` covers this whole file except two explicit c.nop windows:
# every other instruction here is deliberately kept 32-bit so the offset of
# each straddling instruction is exact and doesn't depend on which
# instructions the assembler chose to auto-compress. `.p2align 2` inside
# each `rvc` window is not a no-op the second time -- it is what forces the
# following c.nop back onto a 4-byte boundary regardless of the (odd, by
# then) alignment left over from the first straddle, so the branch target
# below lands precisely on pc % 4 == 2.

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  .option push
  .option norvc

  #-------------------------------------------------------------
  # Straight-line: a 32-bit instruction immediately after a compressed one.
  #-------------------------------------------------------------
  li TESTNUM, 2
  .option push
  .option rvc
  .p2align 2
  c.nop
  .option pop
  addi a0, x0, 0x123    # straddles pc % 4 == 2
  li a1, 0x123
  bne a0, a1, fail

  #-------------------------------------------------------------
  # A branch into a 2-byte-aligned (not 4-byte-aligned) target that is
  # itself a straddling 32-bit instruction.
  #-------------------------------------------------------------
  li TESTNUM, 3
  li a2, 1
  beq a2, a2, 1f
  j fail
  .option push
  .option rvc
  .p2align 2
  c.nop
  .option pop
1:
  addi a3, x0, 0x456    # branch target, pc % 4 == 2
  li a4, 0x456
  bne a3, a4, fail

  .option pop

  TEST_PASSFAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

RVTEST_DATA_END

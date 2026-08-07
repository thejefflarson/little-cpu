# See LICENSE for license details.

#*****************************************************************************
# cebreak.S
#-----------------------------------------------------------------------------
#
# `c.ebreak` raises a BREAKPOINT (mcause 3), not an illegal instruction.
#
# The C extension defines C.EBREAK (16'h9002) as expanding to EBREAK, so it
# raises the same exception with the same cause. It is the one trap cause on
# this core that has a compressed encoding of its own, and it sits in a corner
# of quadrant 2 that three neighbouring decodes each exclude by a different
# field: C.JALR needs rd/rs1 != 0, C.JR needs instr[12] == 0, and C.ADD/C.MV
# need rs2 != 0. Miss it and the encoding decodes to nothing at all, which on
# this core means `instr_valid` is low and the cause comes out 2 (illegal
# instruction) rather than 3 — a wrong-but-plausible answer that still traps,
# still records an mepc, and still resumes, so nothing except the cause value
# gives it away.
#
# Nothing else in the suite reaches it. test/asm/trap.S's breakpoint case is
# `.option norvc`, and so is every other trapping instruction there, precisely
# because the handler resumes at mepc+4 (riscv_test.h constraint 1). So the
# assembler never had a reason to emit C.EBREAK anywhere, and test/decoder_tb.v
# drives the 32-bit form.
#
# Both alignments are exercised. `c.ebreak` at pc % 4 == 2 is where mepc's WARL
# mask has to preserve bit 1 — the same property test/asm/trap.S
# checks with a `c.lw`, restated here against a trap cause that comes from the
# instruction itself rather than from its operand address.
#
# Layout: each faulting `c.ebreak` is followed by exactly one `c.nop`, because
# the recording handler resumes at mepc+4 unconditionally and cannot read the
# faulting instruction to learn it was 2 bytes (Harvard buses). The
# `.align 2` before each block is what makes the alignment claim true rather
# than incidental, and tests 4 and 8 assert it, so this file cannot quietly
# degrade into two copies of the same case.
#

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  # mtvec resets to 0, which is _start — install before faulting.
  RVTEST_INSTALL_TRAP_HANDLER

  #-------------------------------------------------------------
  # c.ebreak at pc % 4 == 0.
  #-------------------------------------------------------------

  .option push
  .option rvc
  .align 2
cebrk_aligned:
  c.ebreak
  c.nop
  .option pop

  TEST_CASE( 2, a4, 3, la a3, trap_cause; lw a4, 0(a3); )
  TEST_CASE( 3, a4, 1, la a3, trap_count; lw a4, 0(a3); )

  TEST_CASE( 4, a4, 0, la a3, cebrk_aligned; andi a4, a3, 3; )

  # mepc is the address of the faulting `c.ebreak` itself.
test_5:
  la    a3, trap_epc
  lw    a4, 0(a3)
  la    x29, cebrk_aligned
  li    TESTNUM, 5
  bne   a4, x29, fail

  #-------------------------------------------------------------
  # c.ebreak at pc % 4 == 2: mepc must come back with bit 1 SET.
  #-------------------------------------------------------------

  # Clear the recorded cause so the next case cannot pass on a stale value.
  la    a3, trap_cause
  sw    x0, 0(a3)

  .option push
  .option rvc
  .align 2
  c.nop
cebrk_odd:
  c.ebreak
  c.nop
  .option pop

  TEST_CASE( 6, a4, 3, la a3, trap_cause; lw a4, 0(a3); )
  TEST_CASE( 7, a4, 2, la a3, trap_count; lw a4, 0(a3); )

  TEST_CASE( 8, a4, 2, la a3, cebrk_odd; andi a4, a3, 3; )

test_9:
  la    a3, trap_epc
  lw    a4, 0(a3)
  la    x29, cebrk_odd
  li    TESTNUM, 9
  bne   a4, x29, fail

  #-------------------------------------------------------------
  # A trapping `c.ebreak` writes no register, the same commitment
  # every other trapping instruction here makes. c.ebreak's encoding has rd
  # == x0 anyway, so this is really a check that decode's trap suppression did
  # not route something into the rd field on the way past.
  #-------------------------------------------------------------

  TEST_CASE( 10, a5, 0x5a5, \
    li a5, 0x5a5; \
    .option push; .option rvc; .align 2; \
    c.ebreak; c.nop; \
    .option pop; )

  TEST_CASE( 11, a4, 3, la a3, trap_count; lw a4, 0(a3); )

  #-------------------------------------------------------------
  # A trapping instruction does not increment `minstret`, and that
  # has to hold for the compressed breakpoint too. Same shape as trap.S's
  # test 19: the difference across the pair is the first `csrr` plus one
  # handler entry, and the `c.ebreak` itself contributes nothing.
  #-------------------------------------------------------------

test_12:
  csrr  a0, minstret
  .option push
  .option rvc
  .align 2
  c.ebreak
  c.nop
  .option pop
  csrr  a1, minstret
  sub   a2, a1, a0

  la    a5, trap_handler
  la    a6, trap_handler_end
  sub   a5, a6, a5
  srli  a5, a5, 2
  addi  x29, a5, 1
  li    TESTNUM, 12
  bne   a2, x29, fail

  TEST_CASE( 13, a4, 4, la a3, trap_count; lw a4, 0(a3); )

  TEST_PASSFAIL

  RVTEST_TRAP_HANDLER

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  RVTEST_TRAP_DATA

RVTEST_DATA_END

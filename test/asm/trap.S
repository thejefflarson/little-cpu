# See LICENSE for license details.

#*****************************************************************************
# trap.S
#-----------------------------------------------------------------------------
#
# Every trap cause this core implements, taken
# deliberately, recorded by the opt-in handler in riscv_test.h, and asserted
# on from the test body.
#
# This test self-checks in band rather than leaning on the per-retire RVFI
# monitor, and that is not a stylistic choice: riscv-formal ships no spec model
# for ecall/ebreak/mret/csrr* at the pinned SHA, so `spec_valid` is 0 for every
# instruction here that matters and the monitor's whole semantic block is
# skipped. See the constraints at the top of riscv_test.h.
#
# Every trapping instruction is wrapped in `.option norvc` because the handler
# resumes at mepc+4 unconditionally — it cannot read the faulting instruction
# to find out how long it was (Harvard buses).
#
# The handler clobbers t0/t1, so nothing here holds a live value in either
# across a fault.
#

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  # mtvec resets to 0, which is _start — install before faulting.
  RVTEST_INSTALL_TRAP_HANDLER

  #-------------------------------------------------------------
  # Load address misaligned: cause 4.
  #-------------------------------------------------------------

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

  # mepc holds the address of the faulting instruction, and resuming at mepc+4
  # therefore landed on the instruction after it — which is where the code
  # asserting this is running.
test_4:
  la    a3, trap_epc
  lw    a4, 0(a3)
  la    x29, mis_lw
  li    TESTNUM, 4
  bne   a4, x29, fail

  # A trapping load writes no register: a1 still holds its sentinel.
  TEST_CASE( 5, a1, 0x5a5, nop; )

  #-------------------------------------------------------------
  # Store address misaligned: cause 6.
  #-------------------------------------------------------------

  la    a0, tdat
  addi  a0, a0, 2

  .option push
  .option norvc
  sw    a1, 0(a0)
  .option pop

  TEST_CASE(  6, a4, 6, la a3, trap_cause; lw a4, 0(a3); )
  TEST_CASE(  7, a4, 2, la a3, trap_count; lw a4, 0(a3); )

  # ...and wrote no memory: tdat is untouched.
  TEST_CASE(  8, a4, 0x0ff0, la a3, tdat; lw a4, 0(a3); )

  #-------------------------------------------------------------
  # Environment call from M-mode: cause 11.
  #-------------------------------------------------------------

  .option push
  .option norvc
  ecall
  .option pop

  TEST_CASE(  9, a4, 11, la a3, trap_cause; lw a4, 0(a3); )

  #-------------------------------------------------------------
  # Breakpoint: cause 3.
  #-------------------------------------------------------------

  .option push
  .option norvc
  ebreak
  .option pop

  TEST_CASE( 10, a4, 3, la a3, trap_cause; lw a4, 0(a3); )

  #-------------------------------------------------------------
  # Illegal instruction: cause 2. Two flavours — an encoding that decodes to
  # nothing (the all-zero word is illegal by definition), and a read of an
  # unimplemented CSR.
  #-------------------------------------------------------------

  .option push
  .option norvc
  .4byte 0
  .option pop

  TEST_CASE( 11, a4, 2, la a3, trap_cause; lw a4, 0(a3); )

  # Clear the recorded cause so the next case cannot pass on a stale value.
  la    a3, trap_cause
  sw    x0, 0(a3)

  .option push
  .option norvc
  csrr  a0, 0x7c0
  .option pop

  TEST_CASE( 12, a4, 2, la a3, trap_cause; lw a4, 0(a3); )

  #-------------------------------------------------------------
  # ...and the other illegal-CSR rule: a WRITE to a CSR that is read-only by
  # address (mvendorid is 0xF11, so addr[11:10] == 2'b11). The read side of the
  # same CSR is legal and csr.S asserts that it is, so this is specifically the
  # write that must fault. Until trap entry existed the rule was
  # architecturally unobservable -- which is why formal/checks.cfg records that
  # csrw_mscratch_ch0 cannot see the sibling suppression rule either.
  #-------------------------------------------------------------

  la    a3, trap_cause
  sw    x0, 0(a3)

  .option push
  .option norvc
  csrw  mvendorid, t0
  .option pop

  TEST_CASE( 13, a4, 2, la a3, trap_cause; lw a4, 0(a3); )

  #-------------------------------------------------------------
  # A COMPRESSED faulting instruction, so `mepc` is exercised at pc % 4 == 2 --
  # the one case the WARL mask on mepc must not break. That mask clears bit 0
  # only, because C makes 2-byte targets legal; a mask that cleared
  # bit 1 as well would resume two bytes early here and nowhere else in this
  # file, since every other trapping instruction above is `.option norvc` and
  # therefore 4-byte aligned.
  #
  # Two pieces of layout are load-bearing, both consequences of the handler
  # resuming at mepc+4 unconditionally (it cannot read the faulting instruction
  # to learn its length: Harvard buses, and riscv_test.h constraint 1). The
  # `.align 2` plus a 2-byte `c.nop` puts the faulting instruction at
  # pc % 4 == 2; the `c.nop` behind it is 2 bytes of padding so mepc+4 lands on
  # an instruction boundary rather than inside one. Test 16 asserts the
  # alignment actually came out that way, so this cannot quietly degrade into a
  # 4-aligned case that proves nothing.
  #-------------------------------------------------------------

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

test_15:
  la    a3, trap_epc
  lw    a4, 0(a3)
  la    x29, mis_clw
  li    TESTNUM, 15
  bne   a4, x29, fail

  TEST_CASE( 16, a4, 2, la a3, mis_clw; andi a4, a3, 3; )

  # A trapping compressed load writes no register either.
  TEST_CASE( 17, a1, 0x5a5, nop; )

  # Eight traps taken, eight handler entries.
  TEST_CASE( 18, a4, 8, la a3, trap_count; lw a4, 0(a3); )

  #-------------------------------------------------------------
  # A trapping instruction issues, and retires in RVFI with
  # rvfi_trap = 1, but it did NOT retire architecturally — so it
  # must not increment `minstret`. This lives here rather than in
  # test/asm/minstret.S because it needs a trap to be takeable at all; that
  # file carries the exactness rule, which does not.
  #
  # Between the two reads: the first csrr, the whole handler, and the
  # misaligned load — which must not count. `.option norvc` because the
  # handler resumes at mepc+4.
  #-------------------------------------------------------------

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

  # Expected = the first csrr, plus one entry's worth of handler. The handler
  # body is straight-line and `.option norvc`, so its instruction count is its
  # byte span / 4. Computed at run time rather than assembled as a constant:
  # `li` cannot take a label difference, and hardcoding the number would let an
  # edit to riscv_test.h silently invalidate this assertion.
  la    a5, trap_handler
  la    a6, trap_handler_end
  sub   a5, a6, a5
  srli  a5, a5, 2
  addi  x29, a5, 1
  li    TESTNUM, 19
  bne   a2, x29, fail

  # The trap really was taken — otherwise test 19 would be asserting about a
  # sequence that never entered the handler at all.
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

# See LICENSE for license details.

#*****************************************************************************
# mtimer.S
#-----------------------------------------------------------------------------
#
# The machine timer interrupt, taken end to end: armed from software, asserted
# by rtl/timer.v, gated by mie.MTIE and mstatus.MIE, and committed at a decode
# boundary.
#
# THIS FILE AND test/asm/mtimermask.S ARE THE ONLY PROGRAM-LEVEL ORACLE FOR ANY
# OF IT. riscv-formal ships no spec model for an interrupt at the pinned SHA --
# its two pc checks read `rvfi_intr` and use it only to stop expecting pc
# continuity, and nothing upstream names mie, mip, mstatus or an interrupt
# cause -- and the generated checks tie the core's timer input off, so they say
# nothing about this either way. What is checked where: rtl/timer.v's own
# behaviour in test/timer_tb.v, the three-term enable in test/csr_tb.v, the
# decode-boundary commit in test/decoder_tb.v and formal/traps.sv, and the
# whole path only here.
#
# The one property worth stating twice, because everything else follows from
# it: an interrupt is taken on a cycle that would OTHERWISE HAVE ISSUED an
# instruction. The instruction it displaces has not issued, so `mepc` is that
# instruction's own address and the handler resumes AT it -- which is why
# RVTEST_TIMER_HANDLER does not advance mepc the way RVTEST_TRAP_HANDLER does.
#

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  # mtvec resets to 0, which is _start. Install before arming anything.
  RVTEST_INSTALL_TIMER_HANDLER

  #-------------------------------------------------------------
  # Nothing fires until it is armed AND enabled. This runs before any of the
  # cases below so a stray entry cannot be mistaken for one of them.
  #
  # mtimecmp resets to zero, so mtip is already asserted here. Both enables
  # reset to zero as well, which is what makes that harmless -- and disarming
  # before enabling is the ordering any boot path owes the timer.
  #-------------------------------------------------------------

  RVTEST_DISARM_TIMER
  RVTEST_ENABLE_MTIE
  RVTEST_ENABLE_MIE

  TEST_CASE( 2, a4, 0, \
    nop; nop; nop; nop; nop; nop; nop; nop; \
    la a3, irq_count; lw a4, 0(a3); )

  # mip.MTIP is the platform's line, and it is low while mtimecmp is out of
  # reach. mie.MTIE is what this test just set, and it reads back.
  TEST_CASE( 3, a0, 0,    csrr a0, mip; )
  TEST_CASE( 4, a0, 0x80, csrr a0, mie; )

  #-------------------------------------------------------------
  # Armed: the interrupt arrives inside a spin loop.
  #
  # The loop is deliberately made of instructions that are all the same length
  # and all in one basic block, so `mepc` can be asserted to lie inside it --
  # that is the claim that the interrupt was taken between two instructions of
  # the interrupted code, rather than somewhere the core invented.
  #-------------------------------------------------------------

  RVTEST_ARM_TIMER( 40 )

spin_start:
  .option push
  .option norvc
  li    a5, 0
1:
  addi  a5, a5, 1
  li    a6, 60
  blt   a5, a6, 1b
  .option pop
spin_end:

  TEST_CASE( 5, a4, 1, la a3, irq_count; lw a4, 0(a3); )

  # Interrupt, machine timer: bit 31 set, exception code 7.
  TEST_CASE( 6, a4, 0x80000007, la a3, irq_cause; lw a4, 0(a3); )

  # mepc lands inside the spin loop. A handler that resumed at mepc+4 would
  # still pass this -- what it could not pass is test 8.
test_7:
  la    a3, irq_epc
  lw    a4, 0(a3)
  la    x29, spin_start
  li    TESTNUM, 7
  bltu  a4, x29, fail
  la    x29, spin_end
  li    TESTNUM, 7
  bgeu  a4, x29, fail

  # The loop still ran to completion, so the instruction at mepc was executed
  # exactly once rather than skipped. The counter reaching 60 is that
  # statement: mepc pointed at an `addi a5, a5, 1`, a `li` or the branch behind
  # them, and resuming one instruction late would have lost an iteration or the
  # branch that closes the loop.
  TEST_CASE( 8, a5, 60, nop; )

  #-------------------------------------------------------------
  # No nesting, and no second entry. The handler disarmed the timer by moving
  # mtimecmp out of reach, so `mtip` is low again; and even before that, entry
  # cleared mstatus.MIE, so nothing could have re-entered while the handler ran.
  #-------------------------------------------------------------

  TEST_CASE( 9, a0, 0, csrr a0, mip; )

  TEST_CASE( 10, a4, 1, \
    nop; nop; nop; nop; nop; nop; nop; nop; \
    nop; nop; nop; nop; nop; nop; nop; nop; \
    la a3, irq_count; lw a4, 0(a3); )

  # Entry cleared mstatus.MIE and pushed the old value into MPIE, observed from
  # INSIDE the handler rather than inferred: the handler recorded mstatus on
  # entry. MIE (bit 3) clear, MPIE (bit 7) set.
  TEST_CASE( 11, a4, 0, la a3, irq_mstatus; lw a4, 0(a3); andi a4, a4, 0x8; )
  TEST_CASE( 12, a4, 0x80, la a3, irq_mstatus; lw a4, 0(a3); andi a4, a4, 0x80; )

  # ...and mret restored MIE from MPIE, so the machine is armed for the next one
  # rather than having quietly turned interrupts off for good.
  TEST_CASE( 13, a1, 0x8, csrr a0, mstatus; andi a1, a0, 0x8; )

  #-------------------------------------------------------------
  # It is repeatable. A core that takes exactly one interrupt per reset would
  # pass everything above.
  #-------------------------------------------------------------

  RVTEST_ARM_TIMER( 40 )

  .option push
  .option norvc
  li    a5, 0
1:
  addi  a5, a5, 1
  li    a6, 60
  blt   a5, a6, 1b
  .option pop

  TEST_CASE( 14, a4, 2, la a3, irq_count; lw a4, 0(a3); )
  TEST_CASE( 15, a4, 0x80000007, la a3, irq_cause; lw a4, 0(a3); )

  #-------------------------------------------------------------
  # MTIP IS A LEVEL. The spec says the interrupt remains posted until mtimecmp
  # becomes greater than mtime, and that taking the trap does not clear it. So
  # a handler that returns without moving mtimecmp is re-entered immediately --
  # before the instruction at mepc has run.
  #
  # This is the single easiest thing to get wrong: a core that cleared the
  # pending bit on entry, or that latched an edge, passes every case above and
  # fails here. `irq_limit` is what stops the correct behaviour being a
  # livelock: the sticky handler disarms once the count reaches it.
  #-------------------------------------------------------------

  la    a3, irq_count
  sw    x0, 0(a3)
  la    a3, irq_limit
  li    a4, 3
  sw    a4, 0(a3)

  RVTEST_INSTALL_STICKY_TIMER_HANDLER
  RVTEST_ARM_TIMER( 20 )

  .option push
  .option norvc
  li    a5, 0
1:
  addi  a5, a5, 1
  li    a6, 60
  blt   a5, a6, 1b
  .option pop

  # Three entries from ONE arming. The handler moved mtimecmp only on the last.
  TEST_CASE( 16, a4, 3, la a3, irq_count; lw a4, 0(a3); )

  # ...and the interrupted code still finished, so the re-entries did not eat
  # the instruction at mepc.
  TEST_CASE( 17, a5, 60, nop; )

  RVTEST_INSTALL_TIMER_HANDLER

  #-------------------------------------------------------------
  # The torn 64-bit write, and its demonstrated red direction.
  #
  # A 32-bit store touches one half of mtimecmp, so an update is a sequence and
  # the ORDER decides whether the pair is ever transiently smaller than the
  # lesser of the old and new values. The spec's own sample code writes the LOW
  # half all-ones, then the high half, then the low half.
  #
  # Set up so the naive order is genuinely dangerous. mtime is parked at
  # 0x1_0000_0050, mtimecmp is {2, 0x10}, and the target is {1, 0xfffffff0}:
  #
  #   high half first  ->  {1, 0x10}, which mtime is past. FIRES.
  #   spec's order     ->  {2, all ones}, {1, all ones}, {1, 0xfffffff0}.
  #                        Every intermediate is out of reach.
  #
  # Test 18 fires it the wrong way ON PURPOSE, because a sequence whose failure
  # path has never run is not a check. Test 20 then does it the spec's way over
  # the same values and asserts nothing fires.
  #-------------------------------------------------------------

  la    a3, irq_count
  sw    x0, 0(a3)

  li    t0, MTIMER_BASE
  li    t1, 2
  sw    t1, MTIMECMPH_OFFSET(t0)    # mtimecmp = {2, 0x10}
  li    t1, 0x10
  sw    t1, MTIMECMP_OFFSET(t0)
  li    t1, 1
  sw    t1, 4(t0)                   # mtime = {1, 0x50}
  li    t1, 0x50
  sw    t1, 0(t0)

  # The naive order: high half first. The pair passes through {1, 0x10}.
  li    t1, 1
  sw    t1, MTIMECMPH_OFFSET(t0)
  nop
  nop
  nop
  nop
  li    t1, 0xfffffff0
  sw    t1, MTIMECMP_OFFSET(t0)

  TEST_CASE( 18, a4, 1, \
    nop; nop; nop; nop; \
    la a3, irq_count; lw a4, 0(a3); )

  # The handler disarmed and recorded the cause, so test 18 counted a real
  # timer interrupt rather than any other trap.
  TEST_CASE( 19, a4, 0x80000007, la a3, irq_cause; lw a4, 0(a3); )

  # Now the same update, the spec's way, from the same starting point.
  la    a3, irq_count
  sw    x0, 0(a3)

  li    t0, MTIMER_BASE
  li    t1, 2
  sw    t1, MTIMECMPH_OFFSET(t0)
  li    t1, 0x10
  sw    t1, MTIMECMP_OFFSET(t0)
  li    t1, 1
  sw    t1, 4(t0)
  li    t1, 0x50
  sw    t1, 0(t0)

  li    t1, -1
  sw    t1, MTIMECMP_OFFSET(t0)     # low = all ones: no smaller than the old
  li    t1, 1
  sw    t1, MTIMECMPH_OFFSET(t0)    # high = new: no smaller than the new
  nop
  nop
  nop
  nop
  li    t1, 0xfffffff0
  sw    t1, MTIMECMP_OFFSET(t0)     # low = new

  TEST_CASE( 20, a4, 0, \
    nop; nop; nop; nop; \
    la a3, irq_count; lw a4, 0(a3); )

  # ...and the pair really is out of reach afterwards, so test 20 passed
  # because the sequence was safe and not because the timer had stopped.
  TEST_CASE( 21, a0, 0, csrr a0, mip; )

  #-------------------------------------------------------------
  # One more end-to-end entry, so the sequences above are shown to leave the
  # timer usable rather than merely quiet. RVTEST_ARM_TIMER writes a zero high
  # half, so mtime's high half has to come back to zero first.
  #-------------------------------------------------------------

  li    t0, MTIMER_BASE
  sw    x0, 4(t0)
  sw    x0, 0(t0)

  RVTEST_ARM_TIMER( 40 )

  .option push
  .option norvc
  li    a5, 0
1:
  addi  a5, a5, 1
  li    a6, 60
  blt   a5, a6, 1b
  .option pop

  TEST_CASE( 22, a4, 1, la a3, irq_count; lw a4, 0(a3); )

  TEST_PASSFAIL

  RVTEST_TIMER_HANDLER
  RVTEST_STICKY_TIMER_HANDLER

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  RVTEST_TIMER_DATA

RVTEST_DATA_END

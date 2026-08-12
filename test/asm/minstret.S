# See LICENSE for license details.

#*****************************************************************************
# minstret.S
#-----------------------------------------------------------------------------
#
# The counter rules that nothing else can check.
#
# `minstret` is exact as read. CSR instructions serialize, and exactness — not
# hazard safety — is the whole reason that stall exists: without it, `csrr
# minstret` reads a count inflated by whatever is in flight behind it. What is
# checked here is the counter rule, that the difference between two reads
# separated by three nops is exactly 4, and the case below says why that is not
# the same claim as the wait.
#
# The *other* counter rule — a trapping instruction issues but does not
# increment `minstret`, because it did not retire architecturally — is checked
# in test/asm/trap.S, next to the trap that provokes it. No trap is taken here.
#
# The riscv-formal checks cannot see either rule — rvfi_csrw_check checks that
# a CSR *write* lands, not that a counter's increment semantics are right, and
# the csrc_* checks are not in that set (formal/checks.cfg's [csrs] note).
# So simulation is the only mechanised check of them today.
#
# No trap is expected anywhere in this file, so it installs the *fatal*
# handler: an unexpected fault reports the live test number instead of
# restarting the program from mtvec = 0.
#

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  RVTEST_INSTALL_TRAP_HANDLER

  #-------------------------------------------------------------
  # Exactness: csrr, nop, nop, nop — four instructions retire between the two
  # reads, counting the first csrr and not the second. Exactly 4, not "about 4".
  #
  # This does NOT grade the serialization wait, though it read as though it did.
  # `minstret` advances when an instruction issues and both reads are issues, so
  # the difference is the number of issues between them whether or not the CSR
  # instruction waited for the pipeline to empty — delete the CSR and mret half
  # of `serialize` in rtl/decoder.v and this stays green, and so does the rest of
  # the suite. What the wait buys is that the value read is the count that has
  # architecturally finished rather than the count in flight, and no difference
  # of two reads can see that. test/decoder_tb.v grades it, and
  # test/MUTATION_DETECTORS is what keeps that pairing honest.
  #-------------------------------------------------------------

  TEST_CASE( 2, a2, 4, \
    csrr a0, minstret; \
    nop; \
    nop; \
    nop; \
    csrr a1, minstret; \
    sub  a2, a1, a0; )

  #-------------------------------------------------------------
  # mcycle advances on its own. It counts cycles, not instructions, so the only
  # portable claim is monotonicity across a few instructions.
  #-------------------------------------------------------------

  TEST_CASE( 3, a2, 1, \
    csrr a0, mcycle; \
    nop; \
    csrr a1, mcycle; \
    sltu a2, a0, a1; )

  # The high halves are separate CSRs and read as zero this early in the run.
  TEST_CASE( 4, a2, 0, \
    csrr a0, mcycleh; \
    csrr a1, minstreth; \
    or   a2, a0, a1; )

  # An explicit write takes precedence over that cycle's increment.
  # For minstret that is exact, because nothing issues between the write and
  # the read back — the read serializes behind the write.
  TEST_CASE( 5, a1, 0x100, \
    li   a0, 0x100; \
    csrw minstret, a0; \
    csrr a1, minstret; )

  #-------------------------------------------------------------
  # The same rule for mcycle, but asserted as a BOUND rather than an exact
  # value, deliberately. mcycle counts cycles, so the number read back is the
  # serialization drain plus the pipeline depth — real numbers, but ones a
  # timing change may legitimately move. An exact assertion here would be a
  # change-detector for pipeline timing wearing a correctness test's clothes.
  # What is actually being claimed is that the write landed at all: without
  # it, mcycle is in the thousands by this point in the run.
  #-------------------------------------------------------------

test_6:
  csrw  mcycle, x0
  csrr  a1, mcycle
  li    x29, 16
  li    TESTNUM, 6
  bgeu  a1, x29, fail

  TEST_PASSFAIL

  RVTEST_TRAP_HANDLER_FATAL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

RVTEST_DATA_END

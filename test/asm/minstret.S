# See LICENSE for license details.

#*****************************************************************************
# minstret.S
#-----------------------------------------------------------------------------
#
# The counter rules that nothing else can check.
#
# `minstret` is exact as read. CSR instructions serialize, and exactness — not
# hazard safety — is the whole reason that stall exists. Without it, `csrr
# minstret` reads a count inflated by whatever happens to be in flight behind
# it, and the difference between two reads separated by three nops is a
# function of pipeline occupancy rather than of architectural state. Here it
# must be exactly 4.
#
# The *other* counter rule — a trapping instruction issues but does not
# increment `minstret`, because it did not retire architecturally — is checked
# in test/asm/trap.S, next to the trap that provokes it. No trap is taken here.
#
# The riscv-formal ladder cannot see either rule — rvfi_csrw_check checks that
# a CSR *write* lands, not that a counter's increment semantics are right, and
# the csrc_* checks are not on this ladder (formal/checks.cfg's [csrs] note).
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
  # reads, counting the first csrr and not the second. Exactly 4, not "about
  # 4": that is the deliverable the serialization stall buys, and an off-by-N
  # here means the check that holds a CSR instruction until execute, access and
  # writeback are empty is looking at the wrong thing.
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

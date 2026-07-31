# See LICENSE for license details.

#*****************************************************************************
# minstret.S
#-----------------------------------------------------------------------------
#
# The two counter rules that nothing else can check.
#
# 1. `minstret` is exact as read. CSR instructions serialize (ADR-0005), and
#    ADR-0027 records that exactness — not hazard safety — is the whole reason
#    that stall exists. Without it, `csrr minstret` reads a count inflated by
#    whatever happens to be in flight behind it, and the difference between two
#    reads separated by three nops is a function of pipeline occupancy rather
#    than of architectural state. Here it must be exactly 4.
#
# 2. A trapping instruction does not increment `minstret` (ADR-0027): it issued,
#    and it retires in RVFI with rvfi_trap = 1, but it did not retire
#    architecturally. The riscv-formal ladder cannot see this — rvfi_csrw_check
#    checks that a CSR write lands, not that a counter's increment semantics are
#    right, and the csrc_* checks that would are unreachable at the pinned SHA.
#    So this test is the simulation-side check of that rule.
#
# The expected count across the trap is derived from the handler's own size
# rather than hardcoded, so editing the handler in riscv_test.h cannot silently
# invalidate this file.
#

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  RVTEST_INSTALL_TRAP_HANDLER

  #-------------------------------------------------------------
  # Exactness: csrr, nop, nop, nop — four instructions retire between the two
  # reads, counting the first csrr and not the second.
  #-------------------------------------------------------------

  TEST_CASE( 2, a2, 4, \
    csrr a0, minstret; \
    nop; \
    nop; \
    nop; \
    csrr a1, minstret; \
    sub  a2, a1, a0; )

  #-------------------------------------------------------------
  # A trapping instruction contributes nothing. Between the two reads: the
  # first csrr, the whole handler, and the misaligned load — which must not
  # count. `.option norvc` because the handler resumes at mepc+4.
  #-------------------------------------------------------------

test_3:
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
  li    TESTNUM, 3
  bne   a2, x29, fail

  # The trap really was taken — otherwise test 3 would be asserting about a
  # sequence that never entered the handler at all.
  TEST_CASE( 4, a4, 1, la a3, trap_count; lw a4, 0(a3); )

  #-------------------------------------------------------------
  # mcycle advances on its own. It counts cycles, not instructions, so the only
  # portable claim is monotonicity across a few instructions.
  #-------------------------------------------------------------

  TEST_CASE( 5, a2, 1, \
    csrr a0, mcycle; \
    nop; \
    csrr a1, mcycle; \
    sltu a2, a0, a1; )

  # The high halves are separate CSRs and read as zero this early in the run.
  TEST_CASE( 6, a2, 0, \
    csrr a0, mcycleh; \
    csrr a1, minstreth; \
    or   a2, a0, a1; )

  # An explicit write takes precedence over that cycle's increment (ADR-0005).
  TEST_CASE( 7, a1, 0x100, \
    li   a0, 0x100; \
    csrw minstret, a0; \
    csrr a1, minstret; )

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

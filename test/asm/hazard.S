# Direct regression coverage for the stall-only hazard
# scoreboard. Every case below has zero cycles of separation between the
# producer and the consumer, so a core with no interlock (or an incomplete
# one) reads stale operands and gets the wrong answer.
#
# x3 (gp) is TESTNUM and x29 is TEST_CASE's own scratch register (see
# riscv_test.h/test_macros.h) — every case below avoids both, using x6 as
# its result register instead.

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  # Back-to-back RAW on rs1: the producer's rd is immediately consumed as
  # the very next instruction's rs1.
  TEST_CASE( 2, x6, 30, \
    li   x1, 10; \
    addi x1, x1, 20; \
    addi x6, x1, 0; \
  )

  # Back-to-back RAW on rs2.
  TEST_CASE( 3, x6, 30, \
    li   x2, 10; \
    addi x2, x2, 20; \
    add  x6, x0, x2; \
  )

  # Back-to-back RAW on both rs1 and rs2 of the same producer, same register.
  TEST_CASE( 4, x6, 80, \
    li   x1, 20; \
    add  x1, x1, x1; \
    add  x6, x1, x1; \
  )

  # Load-use: the loaded value is consumed by the very next instruction.
  TEST_CASE( 5, x6, 0x12345678, \
    la   x1, ldat; \
    lw   x2, 0(x1); \
    addi x6, x2, 0; \
  )

  # Divide immediately followed by a dependent instruction: rs1 of the
  # consumer is the divide's rd, with zero cycles of separation. Regression
  # for the accessor replay defect (rtl/executor.v's `stalled` used to be
  # wired to nothing) combined with the scoreboard needing to reach past a
  # multi-cycle divide, not just a single-cycle producer.
  TEST_CASE( 6, x6, 8, \
    li   x1, 40; \
    li   x2, 5; \
    div  x1, x1, x2; \
    addi x6, x1, 0; \
  )

  # Divide immediately followed by an independent load: must not stall
  # beyond the divide's own latency — the two are unrelated, so the load
  # should issue as soon as the divider drains, with no extra scoreboard
  # bubble on top.
  TEST_CASE( 7, x6, 0x0ff00ff0, \
    li   x1, 40; \
    li   x2, 5; \
    div  x1, x1, x2; \
    la   x4, ldat2; \
    lw   x6, 0(x4); \
  )

  TEST_PASSFAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

ldat:  .word 0x12345678
ldat2: .word 0x0ff00ff0

RVTEST_DATA_END

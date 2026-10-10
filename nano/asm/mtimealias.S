// nano-local, x0-x15. mtime and mcycle are one counter: a CSR write to mcycle/mcycleh moves what the timer window reads, and a store to either mtime word moves what mcycle/mcycleh read, each without disturbing the other half.
#include "riscv_test.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  la a5, __mtimer

  li gp, 1
  li a0, 5
  csrw mcycleh, a0              // mcycle -> mtime, high word
  lw a1, 4(a5)
  bne a1, a0, fail

  li gp, 2
  li a0, 0x10000
  csrw mcycle, a0               // mcycle -> mtime, low word; it has ticked less than 4096 cycles, a slow memory included
  lw a1, 0(a5)
  sub a1, a1, a0
  li a2, 0x1000
  bgeu a1, a2, fail
  lw a1, 4(a5)                  // the high word is untouched
  li a0, 5
  bne a1, a0, fail

  li gp, 3
  li a0, 0x20000
  sw a0, 0(a5)                  // mtime -> mcycle, low word
  csrr a1, mcycle
  sub a1, a1, a0
  li a2, 0x1000
  bgeu a1, a2, fail
  csrr a1, mcycleh              // the high word is untouched
  li a0, 5
  bne a1, a0, fail

  li gp, 4
  li a0, 9
  sw a0, 4(a5)                  // mtime -> mcycle, high word
  csrr a1, mcycleh
  bne a1, a0, fail
  csrr a1, mcycle               // the low word is untouched
  li a0, 0x20000
  sub a1, a1, a0
  li a2, 0x1000
  bgeu a1, a2, fail

  li gp, 5
  li a0, -1
  csrw mcycle, a0               // the low word's carry reaches the high word through both views
  nop
  nop
  nop
  nop
  lw a1, 4(a5)
  li a0, 10
  bne a1, a0, fail

  RVTEST_PASS

fail:
  RVTEST_FAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN
  .align 2
RVTEST_DATA_END

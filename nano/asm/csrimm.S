// nano-local: every register below is x0-x15, since nano is RV32E and traps on x16-x31.
//
// csrrsi/csrrci with a zero immediate must not write, the same suppression the spec
// gives csrrs/csrrc with rs1=x0 -- a tripwire against writing that arm's write-enable
// only over the register-operand pair.
#include "riscv_test.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  csrrs x1, minstret, x0
  csrrci x0, minstret, 0
  csrrs x2, minstret, x0
  sub x4, x2, x1
  li x5, 2
  li gp, 1
  bne x4, x5, fail

  csrrs x1, minstret, x0
  csrrsi x0, minstret, 0
  csrrs x2, minstret, x0
  sub x4, x2, x1
  li x5, 2
  li gp, 2
  bne x4, x5, fail

  RVTEST_PASS

fail:
  RVTEST_FAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN
RVTEST_DATA_END

// nano-local, x0-x15 only. M is cut, so the assembler's rv32ec target refuses the
// mnemonic itself -- the raw encoding is placed directly to prove the CORE traps it.
#include "riscv_test.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  li x1, 6
  li x2, 7
  li x4, 0                   // rd below; RVFI reads regs[rd] even on a trap
  .word 0x02208233           // mul x4, x1, x2 -- traps illegal, M is cut
  RVTEST_FAIL                // unreached unless the trap above did not fire

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN
RVTEST_DATA_END

// nano-local, x0-x15: the raw M-encoding proves the trap handler sees cause 2 at mepc.
#include "riscv_test.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  RVTEST_INSTALL_TRAP_HANDLER

  li x1, 6
  li x2, 7
  li x4, 0                   // rd below; RVFI reads regs[rd] even on a trap
  li gp, 2
mul_instr:
  .word 0x02208233           // mul x4, x1, x2 -- traps illegal, M is cut

  li gp, 3
  la t0, trap_count
  lw t1, 0(t0)
  li t2, 1
  bne t1, t2, fail

  li gp, 4
  la t0, trap_cause
  lw t1, 0(t0)
  li t2, 2
  bne t1, t2, fail

  li gp, 5
  la t0, trap_epc
  lw t1, 0(t0)
  la t2, mul_instr
  bne t1, t2, fail

  RVTEST_PASS

fail:
  RVTEST_FAIL

  RVTEST_TRAP_HANDLER

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN
RVTEST_TRAP_DATA
RVTEST_DATA_END

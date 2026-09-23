// nano-local, x0-x15. Exercises MEIP (ui_in[7]); the handler disables mie.MEIE since a real pin has no software-visible clear.
#include "riscv_test.h"

#define MIE_MEIE 0x800
#define MSTATUS_MIE 0x8
#define CAUSE_MEIP 0x8000000b

RVTEST_RV64U
RVTEST_CODE_BEGIN

  la t0, trap_handler
  csrw mtvec, t0

  la t2, irq_count
  sw x0, 0(t2)

  li t0, MIE_MEIE
  csrs mie, t0
  li t0, MSTATUS_MIE
  csrs mstatus, t0

  li gp, 2
  li s0, 0
  li a0, 100
  la a2, __irqctl
  li a3, 1
  sw a3, 0(a2)                  // arms irq_meip in the harness at this program point, not a cycle count
1:
  lw t1, 0(t2)
  bnez t1, 2f
  addi s0, s0, 1
  blt s0, a0, 1b
  j fail

2:
  li gp, 3
  li a0, 1
  bne t1, a0, fail

  li gp, 4
  csrr t1, mcause
  li a0, CAUSE_MEIP
  bne t1, a0, fail

  RVTEST_PASS

fail:
  RVTEST_FAIL

  .align 2
trap_handler:
  .option push
  .option norvc
  la t0, irq_count
  lw t1, 0(t0)
  addi t1, t1, 1
  sw t1, 0(t0)
  li t0, MIE_MEIE
  csrc mie, t0                // disable MEIE: a level pin has no other way to stop re-entry
  mret
  .option pop

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN
  .align 2
  .global irq_count
irq_count:
  .word 0
RVTEST_DATA_END

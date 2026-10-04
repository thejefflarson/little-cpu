// nano-local, x0-x15. The three-store mtimecmp update, graded both ways: written high word first it passes through a value at or below mtime and posts a spurious interrupt, written low all-ones, high, low it never does; then MEIP and MTIP pending together are taken external first.
#include "riscv_test.h"

#define MIE_MTIE 0x80
#define MIE_BOTH 0x880
#define MSTATUS_MIE 0x8
#define CAUSE_MTIP 0x80000007
#define CAUSE_MEIP 0x8000000b

// The handler's record, one word each from irq_count; a4 and a5 are its alone to read.
#define IRQ_CAUSE 4
#define IRQ_CAUSE0 8
#define IRQ_LIMIT 16

RVTEST_RV64U
RVTEST_CODE_BEGIN

  la t0, trap_handler
  csrw mtvec, t0
  la a5, __mtimer
  la a4, irq_count
  li a0, 1
  sw a0, IRQ_LIMIT(a4)

  li a1, -1
  sw a1, 8(a5)                  // out of reset mtimecmp is zero: move it out of reach, spec order
  sw a1, 12(a5)
  sw a1, 8(a5)

  li a0, 1
  sw a0, 4(a5)
  li a0, 0x50
  sw a0, 0(a5)                  // mtime = {1, 0x50}
  li a0, 2
  sw a0, 12(a5)
  li a0, 0x10
  sw a0, 8(a5)                  // mtimecmp = {2, 0x10}

  li a0, MIE_MTIE
  csrs mie, a0
  li a0, MSTATUS_MIE
  csrs mstatus, a0

  li a0, 1
  sw a0, 12(a5)                 // the wrong order: high first leaves {1, 0x10}, below mtime
  nop
  nop
  nop
  nop
  li gp, 1
  lw a0, 0(a4)
  li a1, 1
  bne a0, a1, fail

  li gp, 2
  lw a0, IRQ_CAUSE(a4)
  li a1, CAUSE_MTIP
  bne a0, a1, fail

  sw x0, 0(a4)
  li a0, 1
  sw a0, 4(a5)
  li a0, 0x50
  sw a0, 0(a5)                  // mtime = {1, 0x50}
  li a0, 2
  sw a0, 12(a5)
  li a0, 0x10
  sw a0, 8(a5)                  // mtimecmp = {2, 0x10}

  li a0, -1
  sw a0, 8(a5)                  // the spec's order: low all-ones, no smaller than the old,
  li a0, 1
  sw a0, 12(a5)                 // high, no smaller than the new,
  nop
  nop
  nop
  nop
  li a0, 0xfffffff0
  sw a0, 8(a5)                  // low
  nop
  nop
  nop
  nop
  li gp, 3
  lw a0, 0(a4)
  bnez a0, fail

  li gp, 4
  csrr a0, mip
  andi a0, a0, MIE_MTIE
  bnez a0, fail

  li a0, MSTATUS_MIE
  csrc mstatus, a0
  li a0, MIE_BOTH
  csrs mie, a0
  li a0, 2
  sw a0, IRQ_LIMIT(a4)
  sw x0, 0(a4)
  sw x0, 8(a5)                  // mtimecmp = {1, 0}: MTIP goes up
  la a0, __irqctl
  li a1, 1
  sw a1, 0(a0)                  // and so does MEIP
  nop
  nop
  nop
  nop
  nop
  nop
  li a0, MSTATUS_MIE
  csrs mstatus, a0
  li a1, 100
  li a2, 0
1:
  lw a0, 0(a4)
  li s0, 2
  bgeu a0, s0, 2f
  addi a2, a2, 1
  blt a2, a1, 1b
  li gp, 5
  j fail

2:
  li gp, 6
  lw a0, IRQ_CAUSE0(a4)         // both pending: external outranks timer
  li a1, CAUSE_MEIP
  bne a0, a1, fail

  li gp, 7
  lw a0, IRQ_CAUSE(a4)
  li a1, CAUSE_MTIP
  bne a0, a1, fail

  RVTEST_PASS

fail:
  RVTEST_FAIL

  .align 2
trap_handler:
  .option push
  .option norvc
  lw t1, 0(a4)
  addi t2, t1, 1
  sw t2, 0(a4)
  csrr t2, mcause
  bnez t1, 1f
  sw t2, IRQ_CAUSE0(a4)
1:
  sw t2, IRQ_CAUSE(a4)
  li t0, 0x800
  csrc mie, t0                  // a level pin has no other way to stop re-entry
  lw t1, 0(a4)
  lw t0, IRQ_LIMIT(a4)
  bltu t1, t0, 2f
  li t1, -1
  sw t1, 8(a5)
  sw t1, 12(a5)
  sw t1, 8(a5)
2:
  mret
  .option pop

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN
  .align 2
irq_count:
  .word 0, 0, 0, 0, 0
RVTEST_DATA_END

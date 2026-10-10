// nano-local, x0-x15. The machine timer end to end: out of reset MTIP is up and all three enable terms gate it, then software arms mtimecmp, takes cause 7 inside a loop, returns, and a handler that leaves mtimecmp alone is re-entered.
#include "riscv_test.h"

#define MIE_MTIE 0x80
#define MSTATUS_MIE 0x8
#define CAUSE_MTIP 0x80000007

// The handler's record, one word each from irq_count; a4 and a5 are its alone to read.
#define IRQ_CAUSE 4
#define IRQ_EPC 8
#define IRQ_MSTATUS 12
#define IRQ_LIMIT 16

RVTEST_RV64U
RVTEST_CODE_BEGIN

  la t0, trap_handler
  csrw mtvec, t0
  la a5, __mtimer
  la a4, irq_count

  li gp, 1
  csrr a0, mip                  // mtimecmp resets to zero, so MTIP is up already
  andi a0, a0, MIE_MTIE
  beqz a0, fail

  li gp, 2
  csrr a0, mie                  // harmless only because MTIE resets clear
  bnez a0, fail

  li a0, 1
  sw a0, IRQ_LIMIT(a4)
  li a0, MSTATUS_MIE
  csrs mstatus, a0
  nop
  nop
  li gp, 3
  lw a0, 0(a4)                  // MTIP and mstatus.MIE up, MTIE clear: nothing is taken
  bnez a0, fail
  li a0, MSTATUS_MIE
  csrc mstatus, a0
  li a0, MIE_MTIE
  csrs mie, a0
  nop
  nop
  li gp, 4
  lw a0, 0(a4)                  // MTIP and MTIE up, mstatus.MIE clear: nothing is taken
  bnez a0, fail

  li a0, MSTATUS_MIE
  csrs mstatus, a0
  li a1, 100
  li a2, 0
1:
  lw a0, 0(a4)                  // the posted level is taken as soon as the last term is set
  bnez a0, 2f
  addi a2, a2, 1
  blt a2, a1, 1b
  li gp, 5
  j fail

2:
  li gp, 6
  lw a0, IRQ_CAUSE(a4)
  li a1, CAUSE_MTIP
  bne a0, a1, fail

  li gp, 7
  lw a0, IRQ_MSTATUS(a4)        // entry cleared MIE and saved it in MPIE
  andi a0, a0, 0x88
  li a1, 0x80
  bne a0, a1, fail

  li gp, 8
  csrr a0, mstatus              // mret put MIE back
  andi a0, a0, MSTATUS_MIE
  beqz a0, fail

  li gp, 9
  csrr a0, mip                  // the handler moved mtimecmp out of reach
  andi a0, a0, MIE_MTIE
  bnez a0, fail

  sw x0, 0(a4)
  li a0, 3
  sw a0, IRQ_LIMIT(a4)          // the handler leaves mtimecmp alone twice
  lw a0, 0(a5)
  addi a0, a0, 60
  li a1, -1
  sw a1, 8(a5)                  // the spec's order for mtimecmp: low all-ones,
  sw x0, 12(a5)                 // high,
  sw a0, 8(a5)                  // then low
spin_start:
  li a2, 0
  li a1, 100
  li s0, 3
1:
  lw a0, 0(a4)
  bgeu a0, s0, spin_end
  addi a2, a2, 1
  blt a2, a1, 1b
  li gp, 10
  j fail
spin_end:

  li gp, 11
  lw a0, IRQ_EPC(a4)            // the last entry interrupted the loop, not code after it
  la a1, spin_start
  bltu a0, a1, fail
  la a1, spin_end
  bgeu a0, a1, fail

  RVTEST_PASS

fail:
  RVTEST_FAIL

  .align 2
trap_handler:
  .option push
  .option norvc
  lw t1, 0(a4)
  addi t1, t1, 1
  sw t1, 0(a4)
  csrr t2, mcause
  sw t2, IRQ_CAUSE(a4)
  csrr t2, mepc
  sw t2, IRQ_EPC(a4)
  csrr t2, mstatus
  sw t2, IRQ_MSTATUS(a4)
  lw t0, IRQ_LIMIT(a4)
  bltu t1, t0, 1f
  li t1, -1
  sw t1, 8(a5)
  sw t1, 12(a5)
  sw t1, 8(a5)
1:
  mret
  .option pop

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN
  .align 2
irq_count:
  .word 0, 0, 0, 0, 0
RVTEST_DATA_END

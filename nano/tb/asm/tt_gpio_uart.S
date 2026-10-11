// Exercises the UART, GPIO and machine timer through the top's pins, at the addresses nano_bus decodes. x0-x15 only: this core is RV32E.
#include "riscv_test.h"

#define TT_UART_BASE 0x10800000
#define TT_GPIO_BASE 0x10800008
#define TT_TIMER_BASE 0x10800010
#define TT_UART_BYTE 0xa5
#define TT_GPIO_MASK 0x7f
#define CAUSE_MTIP 0x80000007

RVTEST_RV64U
RVTEST_CODE_BEGIN

  li gp, 1
  li t0, TT_UART_BASE
  li t1, TT_UART_BYTE
  sw t1, 0(t0)
1:
  lw t2, 4(t0)
  andi t2, t2, 1
  bnez t2, 1b

  li gp, 2
  li t0, TT_GPIO_BASE
  lw t1, 4(t0)
  li t2, TT_GPIO_MASK
  and t1, t1, t2
  sw t1, 0(t0)

  li gp, 3
  la t0, trap_handler
  csrw mtvec, t0
  la a4, irq_count
  li a5, TT_TIMER_BASE
  li a0, -1
  sw a0, 8(a5)
  sw a0, 12(a5)
  sw a0, 8(a5)
  li a0, 0x80
  csrs mie, a0
  li a0, 8
  csrs mstatus, a0
  lw a0, 0(a5)
  addi a0, a0, 400
  li a1, -1
  sw a1, 8(a5)                  // mtimecmp = mtime + 400, in the spec's three-store order
  sw x0, 12(a5)
  sw a0, 8(a5)
  li a2, 0
  li a1, 200
1:
  lw a0, 0(a4)
  bnez a0, 2f
  addi a2, a2, 1
  blt a2, a1, 1b
  li gp, 4
  j fail

2:
  li gp, 5
  lw a0, 4(a4)
  li a1, CAUSE_MTIP
  bne a0, a1, fail

  li gp, 6
  lw a0, 8(a5)                  // the handler moved mtimecmp out of reach
  li a1, -1
  bne a0, a1, fail

  li gp, 7
  lw a0, 4(a5)                  // and mtime has not reached the high word
  bnez a0, fail

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
  sw t2, 4(a4)
  li t1, -1
  sw t1, 8(a5)
  sw t1, 12(a5)
  sw t1, 8(a5)
  mret
  .option pop

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN
  .align 2
irq_count:
  .word 0, 0
RVTEST_DATA_END

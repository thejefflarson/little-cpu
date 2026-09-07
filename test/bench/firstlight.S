// The first program written to be looked at rather than graded.

#include "riscv_test.h"

  .section .text
  .globl _start
_start:
  // A STACK, because print_count keeps `ra` on one.
  li      sp, 0x0001F000

  li      s0, 0                   # the blink counter, printed each time round
  li      s1, UART_BASE
  li      s2, 0x00010190          # scratch: a store here is only ever the LED

blink:
  // LED on. The value stored is what reaches the latch, so bit 0 is the LED.
  li      t0, 1
  sw      t0, 0(s2)
  jal     ra, delay

  sw      x0, 0(s2)
  jal     ra, delay

  addi    s0, s0, 1
  jal     ra, print_count
  j       blink

#ifndef DELAY_ITERS
#define DELAY_ITERS 1500000
#endif

delay:
  li      t1, DELAY_ITERS
1:
  addi    t1, t1, -1
  bnez    t1, 1b
  ret

print_count:
  addi    sp, sp, -32
  sw      ra, 28(sp)
  mv      t2, s0
  addi    t3, sp, 0               # buffer cursor
  li      t4, 0                   # digit count
  li      t5, 10

1:                                # divide by ten, remainder is the digit
  remu    t6, t2, t5
  addi    t6, t6, 48              # '0'
  sb      t6, 0(t3)
  addi    t3, t3, 1
  addi    t4, t4, 1
  divu    t2, t2, t5
  bnez    t2, 1b

2:                                # emit in reverse
  addi    t3, t3, -1
  lbu     a1, 0(t3)
  UART_WAIT_IDLE(s1)
  sw      a1, 0(s1)
  addi    t4, t4, -1
  bnez    t4, 2b

  li      a1, 13                  # CR
  UART_WAIT_IDLE(s1)
  sw      a1, 0(s1)
  li      a1, 10                  # LF
  UART_WAIT_IDLE(s1)
  sw      a1, 0(s1)

  lw      ra, 28(sp)
  addi    sp, sp, 32
  ret

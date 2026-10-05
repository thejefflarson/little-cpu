#define UART_BASE   0x00020020
#define UART_DATA   0
#define UART_STATUS 4
#define PERIOD      150000000

.section .text
.globl _start
_start:
  li    s0, UART_BASE
  li    s1, PERIOD
  csrr  s2, mcycle

tick:
  csrr  t0, mcycle
  sub   t0, t0, s2
  bltu  t0, s1, tick
  add   s2, s2, s1
  li    a0, 'T'
  jal   putc
  li    a0, '\n'
  jal   putc
  j     tick

putc:
  lw    t1, UART_STATUS(s0)
  andi  t1, t1, 1
  bnez  t1, putc
  sw    a0, UART_DATA(s0)
  ret

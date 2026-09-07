
#define UART_BASE   0x00020020
#define UART_DATA   0
#define UART_STATUS 4

.section .text
.globl _start
_start:
  li    s0, UART_BASE
  li    s2, 0                    # frame counter, printed as a heartbeat

main_loop:
  la    s1, message
1:
  lbu   a0, 0(s1)
  beqz  a0, 2f
  jal   putc
  addi  s1, s1, 1
  j     1b
2:
  // A long pause so the line is readable and the LEDs are visible.
  li    t0, 1500000
3:
  addi  t0, t0, -1
  bnez  t0, 3b
  addi  s2, s2, 1
  j     main_loop

putc:
  lw    t1, UART_STATUS(s0)
  andi  t1, t1, 1
  bnez  t1, putc
  sw    a0, UART_DATA(s0)
  ret

  .balign 4
message:
  .asciz "little-cpu on ECP5 at 25 MHz\n"

#define UART_BASE   0x00020020
#define UART_STATUS 4
#define ITERS       16

.section .text
.globl _start
_start:
  csrr  t0, mhartid
  bnez  t0, hart1

  li    s0, UART_BASE
  la    s3, shared
  sw    x0, 4(s3)
  sw    x0, 8(s3)
  sw    x0, 12(s3)
  li    t2, 1
  sw    t2, 0(s3)

  li    t2, ITERS
0:
  lw    t3, 4(s3)
  addi  t3, t3, 1
  sw    t3, 4(s3)
  addi  t2, t2, -1
  bnez  t2, 0b

  li    t2, 20000
1:
  lw    t3, 12(s3)
  bnez  t3, 2f
  addi  t2, t2, -1
  bnez  t2, 1b

2:
report:
  la    s1, label
3:
  lbu   a0, 0(s1)
  beqz  a0, 4f
  jal   putc
  addi  s1, s1, 1
  j     3b
4:
  lw    a0, 4(s3)
  jal   put2
  li    a0, ' '
  jal   putc
  li    a0, 'c'
  jal   putc
  li    a0, '1'
  jal   putc
  li    a0, '='
  jal   putc
  lw    a0, 8(s3)
  jal   put2
  li    a0, '\n'
  jal   putc
  li    t0, 1500000
5:
  addi  t0, t0, -1
  bnez  t0, 5b
  j     report

hart1:
  la    s3, shared
6:
  lw    t3, 0(s3)
  beqz  t3, 6b
  li    t2, ITERS
7:
  lw    t3, 8(s3)
  addi  t3, t3, 1
  sw    t3, 8(s3)
  addi  t2, t2, -1
  bnez  t2, 7b
  li    t2, 1
  sw    t2, 12(s3)
8:
  j     8b

putc:
  lw    t1, UART_STATUS(s0)
  andi  t1, t1, 1
  bnez  t1, putc
  sw    a0, 0(s0)
  ret

put2:
  mv    s5, ra
  li    t0, 10
  divu  s6, a0, t0
  remu  s7, a0, t0
  beqz  s6, 9f
  addi  a0, s6, '0'
  jal   putc
9:
  addi  a0, s7, '0'
  jal   putc
  jr    s5

label:
  .asciz "DUAL c0="

.section .bss
.align 2
shared:
  .space 16

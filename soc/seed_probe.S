#define UART_BASE   0x00020020
#define UART_DATA   0
#define UART_STATUS 4
#define CSR_SEED    0x015

// Reads the seed CSR in a loop and prints each status change with the cycle count it arrived at
// ("S<0-3> <mcycle>": 0 BIST, 1 WAIT, 2 ES16, 3 DEAD) and every word a read of ES16 returns
// ("W <word> <mcycle of the first ES16 read>"), so a read opened late still recovers the first.
.section .text
.globl _start
_start:
  li    s0, UART_BASE
  li    s5, 0                    # mcycle of the first ES16 read
  li    s3, -1                   # last status printed
  li    a0, 'R'
  jal   putc
  li    a0, '\n'
  jal   putc
poll:
  csrrw s4, CSR_SEED, x0
  srli  a1, s4, 30
  beq   a1, s3, same
  mv    s3, a1
  li    a0, 'S'
  jal   putc
  addi  a0, a1, '0'
  jal   putc
  li    a0, ' '
  jal   putc
  csrr  a0, mcycle
  jal   puthex
  li    t0, 2
  bne   a1, t0, same
  bnez  s5, same
  csrr  s5, mcycle
same:
  li    t0, 2
  bne   a1, t0, poll
  li    a0, 'W'
  jal   putc
  li    a0, ' '
  jal   putc
  slli  a0, s4, 16
  srli  a0, a0, 16
  jal   hex8
  li    a0, ' '
  jal   putc
  mv    a0, s5
  jal   puthex
  j     poll

// a0 as eight hex digits and a newline; clobbers a0, t1-t5.
puthex:
  mv    t5, ra
  jal   hex8
  li    a0, '\n'
  jal   putc
  mv    ra, t5
  ret

hex8:
  mv    t3, ra
  mv    t2, a0
  li    t4, 28
1:
  srl   a0, t2, t4
  andi  a0, a0, 15
  li    t1, 10
  blt   a0, t1, 2f
  addi  a0, a0, 'a' - '0' - 10
2:
  addi  a0, a0, '0'
  jal   putc
  addi  t4, t4, -4
  bgez  t4, 1b
  mv    ra, t3
  ret

putc:
  lw    t1, UART_STATUS(s0)
  andi  t1, t1, 1
  bnez  t1, putc
  sw    a0, UART_DATA(s0)
  ret

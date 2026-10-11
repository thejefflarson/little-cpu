#define UART_BASE   0x00020020
#define UART_STATUS 4
#define SPI_BASE    0x00020028
#define SPI_CONTROL 4
#define SPI_BUSY    0x100

.section .text
.globl _start
_start:
  li    s0, UART_BASE
  li    s1, SPI_BASE

main_loop:
  li    a0, 1
  sw    a0, SPI_CONTROL(s1)
  li    a0, 0x9f
  jal   xfer
  jal   xfer0
  mv    s2, a0
  jal   xfer0
  mv    s3, a0
  jal   xfer0
  mv    s4, a0
  sw    x0, SPI_CONTROL(s1)

  li    a0, 1
  sw    a0, SPI_CONTROL(s1)
  li    a0, 0x5a
  jal   xfer
  jal   xfer0
  jal   xfer0
  jal   xfer0
  jal   xfer0
  jal   xfer0
  mv    s5, a0
  jal   xfer0
  mv    s6, a0
  jal   xfer0
  mv    s7, a0
  jal   xfer0
  mv    s8, a0
  sw    x0, SPI_CONTROL(s1)

  la    a0, label
  jal   puts
  mv    a0, s2
  jal   put_hex
  mv    a0, s3
  jal   put_hex
  mv    a0, s4
  jal   put_hex
  la    a0, label2
  jal   puts
  mv    a0, s5
  jal   putc
  mv    a0, s6
  jal   putc
  mv    a0, s7
  jal   putc
  mv    a0, s8
  jal   putc
  li    a0, '\n'
  jal   putc

  li    t0, 1500000
1:
  addi  t0, t0, -1
  bnez  t0, 1b
  j     main_loop

xfer0:
  li    a0, 0
xfer:
  sw    a0, 0(s1)
2:
  lw    t1, 0(s1)
  andi  t2, t1, SPI_BUSY
  bnez  t2, 2b
  lw    t1, 0(s1)
  andi  a0, t1, 0xff
  ret

putc:
  lw    t1, UART_STATUS(s0)
  andi  t1, t1, 1
  bnez  t1, putc
  sw    a0, 0(s0)
  ret

puts:
  mv    s9, ra
  mv    s10, a0
3:
  lbu   a0, 0(s10)
  beqz  a0, 4f
  jal   putc
  addi  s10, s10, 1
  j     3b
4:
  jr    s9

put_hex:
  mv    s11, ra
  mv    t3, a0
  srli  a0, t3, 4
  jal   nibble
  andi  a0, t3, 15
  jal   nibble
  jr    s11

nibble:
  mv    t4, ra
  addi  a0, a0, '0'
  li    t5, '9'
  bleu  a0, t5, 5f
  addi  a0, a0, 'A' - '0' - 10
5:
  jal   putc
  jr    t4

label:
  .asciz "FLASH id="
label2:
  .asciz " sfdp="

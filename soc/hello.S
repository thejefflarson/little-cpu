# The smallest thing that proves a board can report a number: print a line over
# the UART, forever, with a visible LED heartbeat beside it.
#
# rtl/uart.v is transmit-only, 8N1, and a byte written while `busy` is set is
# DROPPED rather than queued -- so every byte polls the status register first.
# The status word is one bit: bit 0 is busy.
#
# The LEDs come free: rtl/littlesoc.v latches the low two bits of the last store
# to any address, so the store that sends a byte also drives them. That means a
# dark board and a silent line cannot disagree about whether this is running.
#
# Not in test/asm/: it never returns, and both simulation legs glob that
# directory and take everything in it to be the suite.
#   make icesugar-bitstream ICESUGAR_PROG=soc/hello.S && make icesugar-prog

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
  # A long pause so the line is readable and the LEDs are visible.
  li    t0, 1500000
3:
  addi  t0, t0, -1
  bnez  t0, 3b
  addi  s2, s2, 1
  j     main_loop

# a0 = byte. Polls `busy` before writing, because a byte written mid-frame is
# lost rather than buffered.
putc:
  lw    t1, UART_STATUS(s0)
  andi  t1, t1, 1
  bnez  t1, putc
  sw    a0, UART_DATA(s0)
  ret

  # The string lives in .text ON PURPOSE. The Makefile's `soc-rom` recipe
  # extracts `-j .text` only for an `.S` program, so a .rodata section is
  # silently dropped from the ROM image and `la` would point at nothing.
  .balign 4
message:
  .asciz "little-cpu on ECP5 at 25 MHz\n"

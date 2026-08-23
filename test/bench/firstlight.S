# The first program written to be looked at rather than graded.
#
# NOT IN test/asm, AND THAT IS THE POINT. Both sim legs glob that directory and
# hold every file in it to a 5000-cycle limit; this one never ends, so there it
# would be a suite failure rather than a program. It lives here with Dhrystone,
# outside the graded set, and reaches a ROM through `SOC_PROG=test/bench/firstlight.S`.
#
# It says two things at once, because on a board only one of them may be legible:
#
#   THE LED carries the clock. rtl/littlesoc.v latches bit 0 of the last store's
#   data into `store_bit` and drives `ledg_n` from it, so a store of 1 lights the
#   green LED and a store of 0 puts it out. The delay below is a fixed count of a
#   fixed loop, so the blink PERIOD is a measurement of the clock and nothing
#   nothing else -- time it and divide. That matters most under `SB_HFOSC`,
#   trimmed to about +/-10% and whose real rate nothing in this tree knows.
#   (Mind the comment column here: cpp runs over this file before the assembler
#   does, so a line beginning `# else` or `# if` is a directive and not a note.)
#
#   THE UART carries the same count in ASCII, for when the clock is exact enough
#   to read. At 115200 8N1 a wrong clock is garbage rather than silence, so a
#   readable line is itself evidence the divisor is right.
#
# A HALTED CORE AND A FINISHED ONE LOOK THE SAME ON A LED. That is why this
# blinks forever and prints an increasing number: a still LED is a stopped core,
# a steady blink is a running one, and a counter that advances says the loop is
# going round rather than the pin being stuck.

#include "riscv_test.h"

  .section .text
  .globl _start
_start:
  # A STACK, because print_count keeps `ra` on one. Nothing sets `sp` for a `.S`
  # program -- test/asm/sections.lds gives no startup -- so it comes out of reset
  # at zero, and `addi sp, sp, -32` then addresses 0xFFFFFFE0. That used to be a
  # store nobody answered and silently dropped; it is a store access fault now,
  # which is how this line came to be written.
  li      sp, 0x0001F000

  li      s0, 0                   # the blink counter, printed each time round
  li      s1, UART_BASE
  # NOT the base of RAM. riscv_test.h puts the HTIF window there, and both sim
  # legs read a store to it as this program's verdict -- so blinking through it
  # ends the simulation at the first flash instead of running. A hundred words
  # up is ordinary memory nothing interprets.
  li      s2, 0x00010190          # scratch: a store here is only ever the LED

blink:
  # LED on. The value stored is what reaches the latch, so bit 0 is the LED.
  li      t0, 1
  sw      t0, 0(s2)
  jal     ra, delay

  # LED off.
  sw      x0, 0(s2)
  jal     ra, delay

  addi    s0, s0, 1
  jal     ra, print_count
  j       blink

# ---- the delay, and the whole measurement ----------------------------------
#
# DELAY_ITERS iterations of a two-instruction loop, twice per blink, plus the
# print. What that costs was MEASURED under cxxrtl rather than reasoned about,
# by building the same source with a blink limit and reading the cycle count at
# two delay lengths:
#
#   DELAY_ITERS=2000, 10 blinks -> 223398 cycles
#   DELAY_ITERS=4000, 10 blinks -> 423437 cycles
#
# which separates the two terms: 5.0010 cycles per delay iteration and 2336
# cycles of per-blink overhead. Predicting the DELAY_ITERS=1000 point from those
# gives 123378 against 123398 measured, 0.02% out -- so the model is the loop and
# not a fit.
#
# AT THE DEFAULT BELOW, ONE BLINK IS 15,005,261 CYCLES. So the clock is
#
#     f = 15005261 / T        T = seconds for one full on-and-off
#
# which is 1.250 s at 12 MHz. Time ten and divide by ten: a stopwatch is good to
# about a percent over that, which is far tighter than SB_HFOSC's +/-10% trim and
# is the only way this tree has of learning what that oscillator actually runs at.
#ifndef DELAY_ITERS
#define DELAY_ITERS 1500000
#endif

delay:
  li      t1, DELAY_ITERS
1:
  addi    t1, t1, -1
  bnez    t1, 1b
  ret

# ---- the counter, in ASCII -------------------------------------------------
#
# Decimal, low digit first into a buffer on the stack, then emitted in reverse.
# No .data section is touched: this program is linked with test/asm/sections.lds
# like every other `.S`, and a bring-up program that depended on the startup
# having copied anything would be testing two things at once.
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

# A blink for bringing a board up, and nothing else.
#
# rtl/littlesoc.v latches the low two bits of the last store to ANY address onto
# its two LEDs, so this needs no device and no map: it counts, and publishes two
# bits of the counter. The pattern cycles off / green / red / both, which says
# the core is EXECUTING rather than that the clock is merely ticking -- a
# hardware counter would blink just as happily with a broken datapath.
#
# Not in test/asm/: both simulation legs glob that directory and this never
# returns, so it would hang the runner's 5000-cycle bound. Build it by path:
#   make soc-rom SOC_PROG=soc/blink.S
#
# Shift 21 is about half a second a step at 25 MHz and about one at 12; the
# exact rate is not the point, but it is a measurement if you time it against
# the simulated cycle count.
.section .text
.globl _start
_start:
  li   t0, 0x00010000        # any mapped RAM word; the value is what matters
  li   t1, 0
1:
  addi t1, t1, 1
  srli t2, t1, 21
  andi t2, t2, 3
  sw   t2, 0(t0)
  j    1b

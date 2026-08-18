# See LICENSE for license details.

#*****************************************************************************
# uart.S
#-----------------------------------------------------------------------------
#
# The transmit-only UART, driven the way firmware drives it: poll the status
# register, write a byte, poll again. Printing "OK\n" is the whole program.
#
# WHAT THIS PROGRAM CAN AND CANNOT SEE. The status register is all of it: the
# serial line is not wired to anything test/testbench.v looks at, and no
# consumer of this suite decodes it. So the bytes on the wire are checked in
# test/uart_tb.v, which samples the line every cycle and decodes it at the
# configured divisor with five of its own failures forced. What is checked HERE
# is the memory-mapped half, through real loads and stores against the whole
# core: that a write is accepted, that it takes the device busy, that the byte
# occupies a real frame's worth of time, and that a write arriving mid-frame is
# dropped rather than queued.
#
# IT DOES NOT AGREE WITH THE REFERENCE MODEL, AND CANNOT. Sail has plain memory
# where this device is, so the status register reads back whatever was last
# stored there and `busy` is never set: the model spins in the first wait below
# forever and `make cosim-suite` records INCONCLUSIVE SAIL-LIMIT, which is
# NOTHING COMPARED rather than a divergence. test/COSIM_EXPECTED_FAIL carries
# why.
#
# The waits are unbounded on purpose. A device that never raises `busy` and one
# that never lowers it both hang here, and the runner's cycle budget turns that
# into a red program -- which is a check, where a bounded retry that gave up and
# carried on would not be.
#

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  li    t0, UART_BASE

  # Nothing has been written, so nothing is in flight.
  TEST_CASE( 2, a0, 0, lw a0, UART_STATUS_OFFSET(t0); )

  # The data register is write-only.
  TEST_CASE( 3, a0, 0, lw a0, 0(t0); )

  #-------------------------------------------------------------
  # A write starts a frame. This is the first thing the reference model cannot
  # follow: on Sail the store lands in memory, the load reads zero and the wait
  # below never ends.
  #-------------------------------------------------------------

  li    a1, 'O'
  sw    a1, 0(t0)
  TEST_CASE( 4, a0, 1, lw a0, UART_STATUS_OFFSET(t0); )

  #-------------------------------------------------------------
  # A byte written mid-frame is dropped: there is no queue, and the device must
  # go on sending the byte it already has rather than splicing the two. The
  # program cannot see the line, so what it checks is that the second write
  # neither restarted the frame nor extended it -- the count below is the same
  # with the write and without it.
  #-------------------------------------------------------------

  li    a1, '!'
  sw    a1, 0(t0)

  # How many polls the frame takes. Any smaller number would also be reached by
  # a device that dropped the byte on the floor, so this is a floor and test 6
  # is the ceiling.
  li    a2, 0
1:
  lw    a0, UART_STATUS_OFFSET(t0)
  addi  a2, a2, 1
  bnez  a0, 1b

  # Ten bit times at 12 MHz over 115200 baud is 1040 cycles, and each pass of
  # the loop above is more than one instruction, so the count is well under
  # that and well over zero. The bound is loose on purpose: it is a statement
  # that the device held the line for a real frame rather than for a cycle,
  # and the exact cycle count belongs in the bench, where the divisor is
  # visible.
  TEST_CASE( 5, a3, 1, sltiu a3, a2, 1040; )
  TEST_CASE( 6, a3, 1, sltiu a3, a2, 64; xori a3, a3, 1; )

  #-------------------------------------------------------------
  # The second byte, through the wait macro firmware would use.
  #-------------------------------------------------------------

  li    a1, 'K'
  sw    a1, 0(t0)
  UART_WAIT_IDLE( t0 )

  TEST_CASE( 7, a0, 0, lw a0, UART_STATUS_OFFSET(t0); )

  #-------------------------------------------------------------
  # The status register is read-only and the word past the region is nobody's,
  # so neither store starts anything. Without these a device that transmitted on
  # any store to its neighbourhood would pass everything above.
  #-------------------------------------------------------------

  li    a1, 'X'
  sw    a1, UART_STATUS_OFFSET(t0)
  TEST_CASE( 8, a0, 0, lw a0, UART_STATUS_OFFSET(t0); )

  sw    a1, 8(t0)
  TEST_CASE( 9, a0, 0, lw a0, UART_STATUS_OFFSET(t0); )

  #-------------------------------------------------------------
  # The last byte of the string, which doubles as the control on the two above:
  # they passed because those stores were ignored, and not because the device
  # had stopped transmitting by then.
  #-------------------------------------------------------------

  li    a1, '\n'
  sw    a1, 0(t0)
  TEST_CASE( 10, a0, 1, lw a0, UART_STATUS_OFFSET(t0); )
  UART_WAIT_IDLE( t0 )

  TEST_PASSFAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

RVTEST_DATA_END

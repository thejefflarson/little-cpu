# See LICENSE for license details.

#*****************************************************************************
# spiflash.S
#-----------------------------------------------------------------------------
#
# The read-only SPI master, driven the way a boot loader drives it: select the
# flash, exchange a command byte, poll, exchange the bytes the answer is made
# of, release. Reading the JEDEC id is the whole program.
#
# WHY THE ID AND NOT THE DATA ARRAY. This program runs in two places. Under the
# simulators the other end of the four wires is test/spiflash_model.v, and on the
# board it is a real W25Q32JV -- and the two agree about exactly one thing, the
# three bytes `0x9f` returns. The data array does not: the part's contents are a
# bitstream and whatever was flashed beside it. So what is checked here is the
# id, which is a real multi-byte transaction, and the bus behaviour around it;
# the address advancing across a 256-byte boundary is checked in
# test/spiflash_tb.v, where the model is the thing on the other end by
# construction.
#
# IT DOES NOT AGREE WITH THE REFERENCE MODEL, AND CANNOT, for the reason
# test/asm/uart.S does not: Sail has plain memory where this device is, so the
# data register reads back whatever byte was last stored to it and `busy` is
# never set. Test 4 below is the first load whose value differs, and it is
# placed before the first unbounded poll on purpose -- everything up to it is
# compared and agrees.
#
# The waits are unbounded on purpose. A master that never raises `busy` and one
# that never lowers it both hang here, and the runner's cycle budget turns that
# into a red program, where a bounded retry that carried on would not.
#

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  RVTEST_INSTALL_TRAP_HANDLER

  li    t0, SPI_BASE

  #-------------------------------------------------------------
  # Out of reset the master holds nothing and has received nothing, which is
  # also the state that leaves the flash's wires to whatever is programming the
  # board.
  #-------------------------------------------------------------

  TEST_CASE( 2, a0, 0, lw a0, 0(t0); )
  TEST_CASE( 3, a0, 0, lw a0, SPI_CONTROL_OFFSET(t0); )

  #-------------------------------------------------------------
  # Select the flash and start the command. This is the first thing the
  # reference model cannot follow: on Sail the store lands in memory, so the
  # load below reads zero and every wait after it never ends.
  #-------------------------------------------------------------

  li    a1, 1
  sw    a1, SPI_CONTROL_OFFSET(t0)

  # a0 is put somewhere neither answer is, so the load below WRITES it on both
  # sides. Left holding zero it would change nothing on Sail, and the traces
  # would then differ in length rather than in a value -- which is a divergence
  # the model reports by running out of budget in the wait after it instead of
  # by disagreeing at the load. Loaded over -1 it disagrees where it should.
  li    a0, -1

  li    a1, 0x9f
  sw    a1, 0(t0)
  TEST_CASE( 4, a0, SPI_BUSY_BIT, lw a0, 0(t0); )

  # An exchange is sixteen cycles, which is not many instructions, so the load
  # above is the one that has to land inside it and it is the instruction after
  # the store. That a write arriving mid-exchange is DROPPED rather than queued
  # is graded in test/spiflash_tb.v instead, where the cycle the second write
  # arrives on is chosen rather than inferred from a CPI.

  SPI_WAIT_IDLE( t0 )
  TEST_CASE( 5, a0, 0, lw a0, 0(t0); andi a0, a0, SPI_BUSY_BIT; )

  #-------------------------------------------------------------
  # The three id bytes. `0xEF 0x70 0x16` is a Winbond W25Q32JV, which is what
  # the board carries and what `iceprog` prints; no shift order, bit count or
  # sampling edge other than the right one produces them.
  #-------------------------------------------------------------

  TEST_CASE(  6, a0, 0xef, SPI_XFER( t0, 0 ); mv a0, t2; )
  TEST_CASE(  7, a0, 0x70, SPI_XFER( t0, 0 ); mv a0, t2; )
  TEST_CASE(  8, a0, 0x16, SPI_XFER( t0, 0 ); mv a0, t2; )

  #-------------------------------------------------------------
  # Releasing the chip select ends the transaction, so the same command asked
  # again answers from the start. Without this, a device that ignored the chip
  # select entirely and simply replayed three bytes would pass everything above.
  #-------------------------------------------------------------

  SPI_SELECT( t0, 0 )
  SPI_SELECT( t0, 1 )
  SPI_XFER( t0, 0x9f )
  TEST_CASE(  9, a0, 0xef, SPI_XFER( t0, 0 ); mv a0, t2; )
  TEST_CASE( 10, a0, 0x70, SPI_XFER( t0, 0 ); mv a0, t2; )
  TEST_CASE( 11, a0, 0x16, SPI_XFER( t0, 0 ); mv a0, t2; )

  SPI_SELECT( t0, 0 )

  #-------------------------------------------------------------
  # The word past the region is nobody's, and the core refuses the store rather
  # than dropping it: cause 7, committed in decode, so nothing reaches the bus.
  # That is what says the region decode grew to cover THIS window and stopped
  # there -- a decode that had swallowed the neighbourhood would answer here.
  #
  # The handler clobbers t0 and t1, so the base is reloaded behind it.
  #-------------------------------------------------------------

  li    TESTNUM, 12
  la    x1, trap_cause
  sw    x0, 0(x1)
  la    x1, trap_count
  lw    x14, 0(x1)
  .option push
  .option norvc
  sw    a1, 8(t0)
  .option pop
  li    t0, SPI_BASE
  la    x1, trap_cause
  lw    x13, 0(x1)
  li    x29, 7
  bne   x13, x29, fail
  la    x1, trap_count
  lw    x13, 0(x1)
  addi  x29, x14, 1
  bne   x13, x29, fail
  TEST_CASE( 12, a0, 0, lw a0, SPI_CONTROL_OFFSET(t0); )

  #-------------------------------------------------------------
  # ...and the word below it is the UART's status register, which the same
  # decode does answer. The pair is the whole claim: the window is where the
  # map says it is, on both sides.
  #-------------------------------------------------------------

  li    a1, UART_BASE
  TEST_CASE( 13, a0, 0, lw a0, UART_STATUS_OFFSET(a1); )

  TEST_PASSFAIL

  RVTEST_TRAP_HANDLER

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  RVTEST_TRAP_DATA

RVTEST_DATA_END

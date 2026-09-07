# See LICENSE for license details.

# The read-only SPI controller, driven the way a boot loader drives it: select the flash,
# exchange a command byte, poll, exchange the bytes the answer is made of, release.

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  RVTEST_INSTALL_TRAP_HANDLER

  li    t0, SPI_BASE

  TEST_CASE( 2, a0, 0, lw a0, 0(t0); )
  TEST_CASE( 3, a0, 0, lw a0, SPI_CONTROL_OFFSET(t0); )

  li    a1, 1
  sw    a1, SPI_CONTROL_OFFSET(t0)

  li    a0, -1

  li    a1, 0x9f
  sw    a1, 0(t0)
  TEST_CASE( 4, a0, SPI_BUSY_BIT, lw a0, 0(t0); )

  SPI_WAIT_IDLE( t0 )
  TEST_CASE( 5, a0, 0, lw a0, 0(t0); andi a0, a0, SPI_BUSY_BIT; )

  TEST_CASE(  6, a0, 0xef, SPI_XFER( t0, 0 ); mv a0, t2; )
  TEST_CASE(  7, a0, 0x70, SPI_XFER( t0, 0 ); mv a0, t2; )
  TEST_CASE(  8, a0, 0x16, SPI_XFER( t0, 0 ); mv a0, t2; )

  SPI_SELECT( t0, 0 )
  SPI_SELECT( t0, 1 )
  SPI_XFER( t0, 0x9f )
  TEST_CASE(  9, a0, 0xef, SPI_XFER( t0, 0 ); mv a0, t2; )
  TEST_CASE( 10, a0, 0x70, SPI_XFER( t0, 0 ); mv a0, t2; )
  TEST_CASE( 11, a0, 0x16, SPI_XFER( t0, 0 ); mv a0, t2; )

  SPI_SELECT( t0, 0 )

  li    TESTNUM, 12
  li    t0, MAP_TOP
  la    x1, trap_cause
  sw    x0, 0(x1)
  la    x1, trap_count
  lw    x14, 0(x1)
  .option push
  .option norvc
  sw    a1, 0(t0)
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

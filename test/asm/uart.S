# See LICENSE for license details.

# The transmit-only UART, driven the way firmware drives it: poll the status register,
# write a byte, poll again.

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  RVTEST_INSTALL_TRAP_HANDLER

  li    t0, UART_BASE

  # Nothing has been written, so nothing is in flight.
  TEST_CASE( 2, a0, 0, lw a0, UART_STATUS_OFFSET(t0); )

  TEST_CASE( 3, a0, 0, lw a0, 0(t0); )

  li    a1, 'O'
  sw    a1, 0(t0)
  TEST_CASE( 4, a0, 1, lw a0, UART_STATUS_OFFSET(t0); )

  li    a1, '!'
  sw    a1, 0(t0)

  li    a2, 0
1:
  lw    a0, UART_STATUS_OFFSET(t0)
  addi  a2, a2, 1
  bnez  a0, 1b

  TEST_CASE( 5, a3, 1, sltiu a3, a2, 1040; )
  TEST_CASE( 6, a3, 1, sltiu a3, a2, 64; xori a3, a3, 1; )

  li    a1, 'K'
  sw    a1, 0(t0)
  UART_WAIT_IDLE( t0 )

  TEST_CASE( 7, a0, 0, lw a0, UART_STATUS_OFFSET(t0); )

  li    a1, 'X'
  sw    a1, UART_STATUS_OFFSET(t0)
  TEST_CASE( 8, a0, 0, lw a0, UART_STATUS_OFFSET(t0); )

  li    TESTNUM, 9
  li    t0, MAP_TOP
  la    x1, trap_cause
  sw    x0, 0(x1)
  la    x1, trap_count
  lw    x14, 0(x1)
  .option push
  .option norvc
  sw    a1, 0(t0)
  .option pop
  li    t0, UART_BASE
  la    x1, trap_cause
  lw    x13, 0(x1)
  li    x29, 7
  bne   x13, x29, fail
  la    x1, trap_count
  lw    x13, 0(x1)
  addi  x29, x14, 1
  bne   x13, x29, fail
  TEST_CASE( 9, a0, 0, lw a0, UART_STATUS_OFFSET(t0); )

  li    a1, '\n'
  sw    a1, 0(t0)
  TEST_CASE( 10, a0, 1, lw a0, UART_STATUS_OFFSET(t0); )
  UART_WAIT_IDLE( t0 )

  TEST_PASSFAIL

  RVTEST_TRAP_HANDLER

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  RVTEST_TRAP_DATA

RVTEST_DATA_END

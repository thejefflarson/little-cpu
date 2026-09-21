// Exercises the UART and GPIO through the tt_um top's own pins -- nano_tt_tb.v drives
// ui_in and decodes uo_out, never peeking the core's internal bus -- so a wiring mistake
// in nano/bus.v, nano/uart.v or nano/gpio.v fails here even where the core-only suite
// cannot see it. x0-x15 only: this core is RV32E.
#include "riscv_test.h"

#define TT_UART_BASE 0x10800000
#define TT_GPIO_BASE 0x10800008
#define TT_UART_BYTE 0xa5
#define TT_GPIO_MASK 0x7f

RVTEST_RV64U
RVTEST_CODE_BEGIN

  li gp, 1
  li t0, TT_UART_BASE
  li t1, TT_UART_BYTE
  sw t1, 0(t0)
1:
  lw t2, 4(t0)
  andi t2, t2, 1
  bnez t2, 1b

  li gp, 2
  li t0, TT_GPIO_BASE
  lw t1, 4(t0)
  li t2, TT_GPIO_MASK
  and t1, t1, t2
  sw t1, 0(t0)

  RVTEST_PASS

fail:
  RVTEST_FAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN
RVTEST_DATA_END

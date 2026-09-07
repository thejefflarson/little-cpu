// Runs a batch of `.S` suite programs on a board and says how each one came out, over the
// UART, because a part has no HTIF and nothing reads `tohost`.

#include "riscv_test.h"

  .section .text.init
  .globl _board_start
_board_start:
  // ZERO `.driver` BEFORE ANYTHING READS IT. It is NOLOAD and sits outside the range
  // init_memory clears, so nothing else ever touches it -- and SPRAM comes out of
  // configuration holding whatever the last program to run left there.
  la    t0, __driver_start
  la    t1, __driver_end
0:
  bgeu  t0, t1, 1f
  sw    x0, 0(t0)
  addi  t0, t0, 4
  j     0b
1:
  j     run_next

// BEFORE EVERY PROGRAM, NOT ONCE PER BATCH, and hardware is what taught this.
init_memory:
  la    t0, __data_load
  la    t1, __data_start
  la    t2, __data_end
1:
  bgeu  t1, t2, 2f
  lw    t3, 0(t0)
  sw    t3, 0(t1)
  addi  t0, t0, 4
  addi  t1, t1, 4
  j     1b
2:
  la    t0, __bss_start
  la    t1, __bss_end
3:
  bgeu  t0, t1, 4f
  sw    x0, 0(t0)
  addi  t0, t0, 4
  j     3b
4:
  ret

// THE DRIVER'S STATE LIVES IN MEMORY, NOT IN REGISTERS, and that is not a stylistic
// choice.
run_next:
  la    sp, __stack_top            /* the program before it may have moved this */
  jal   ra, init_memory            /* a fresh machine, as under simulation */
  la    sp, __stack_top

  la    t0, board_index
  lw    t1, 0(t0)
  la    t2, board_count
  lw    t2, 0(t2)
  bgeu  t1, t2, all_done

  li    t0, UART_BASE
11:
  lw    t1, UART_STATUS_OFFSET(t0)
  bnez  t1, 11b

  // PUT THE INTERRUPT STATE BACK, and the machine timer is why.
  csrw  mie, zero
  csrw  mstatus, zero
  csrw  mtvec, zero

  li    TESTNUM, 0

  la    t0, board_table
  slli  t1, t1, 2
  add   t0, t0, t1
  lw    t0, 0(t0)
  jr    t0                         /* into the program's own _start */

  .globl board_next
board_next:
  la    sp, __stack_top
  la    t0, board_verdict
  sw    TESTNUM, 0(t0)

  la    t0, board_index
  lw    a0, 0(t0)
  jal   ra, put_dec                /* index */
  li    a0, 32
  jal   ra, put_char
  la    t0, board_verdict
  lw    a0, 0(t0)
  jal   ra, put_dec                /* verdict */
  li    a0, 10
  jal   ra, put_char

  la    t0, board_verdict
  lw    t1, 0(t0)
  li    t2, 1
  beq   t1, t2, 8f
  la    t0, board_failed
  li    t1, 1
  sw    t1, 0(t0)
8:
  la    t0, board_index
  lw    t1, 0(t0)
  addi  t1, t1, 1
  sw    t1, 0(t0)
  j     run_next

all_done:
all_done_loop:
  la    s0, board_console
  la    s1, board_console_len
  lw    s1, 0(s1)
  li    s2, 0
6:
  bgeu  s2, s1, 7f
  add   a0, s0, s2
  lbu   a0, 0(a0)
  jal   ra, put_wire               /* wire only: re-buffering would grow it */
  addi  s2, s2, 1
  j     6b
7:
  li    a0, 46                     /* '.', end of one replay */
  jal   ra, put_wire
  li    a0, 10
  jal   ra, put_wire
  la    s4, board_failed
  lw    s4, 0(s4)
  li    s5, 1                      /* green */
  beqz  s4, 10f
  li    s5, 2                      /* red */
10:
  la    s3, board_scratch
  sw    s5, 0(s3)

  li    t0, 3000000
5:
  addi  t0, t0, -1
  bnez  t0, 5b
  j     all_done_loop

  .globl board_console
put_wire:
  li    t2, UART_BASE
1:
  lw    t1, UART_STATUS_OFFSET(t2)
  bnez  t1, 1b
  sw    a0, 0(t2)
  ret

put_char:
  li    t2, UART_BASE
1:
  lw    t1, UART_STATUS_OFFSET(t2)
  bnez  t1, 1b
  sw    a0, 0(t2)

  la    t2, board_console_len
  lw    t1, 0(t2)
  li    t3, 2000
  bgeu  t1, t3, 2f
  la    t3, board_console
  add   t3, t3, t1
  sb    a0, 0(t3)
  addi  t1, t1, 1
  sw    t1, 0(t2)
2:
  ret

put_dec:
  addi  sp, sp, -32
  sw    ra, 28(sp)
  mv    t4, a0
  addi  t5, sp, 0
  li    t6, 0
  li    a1, 10
1:
  remu  a2, t4, a1
  addi  a2, a2, 48
  sb    a2, 0(t5)
  addi  t5, t5, 1
  addi  t6, t6, 1
  divu  t4, t4, a1
  bnez  t4, 1b
2:
  addi  t5, t5, -1
  lbu   a0, 0(t5)
  jal   ra, put_char
  addi  t6, t6, -1
  bnez  t6, 2b
  lw    ra, 28(sp)
  addi  sp, sp, 32
  ret

  .section .driver,"aw",@nobits
  .align 2
  .globl board_console
board_console:
  .space 2048
board_console_len:
  .word 0
board_index:
  .word 0
board_verdict:
  .word 0
board_failed:
  .word 0
board_led:
  .word 0
board_scratch:
  .word 0

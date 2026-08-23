/*
 * Runs a batch of `.S` suite programs on a board and says how each one came
 * out, over the UART, because a part has no HTIF and nothing reads `tohost`.
 *
 * WHY A DRIVER AT ALL. One program per bitstream would be 72 flashes at about
 * thirty seconds each; the ROM holds several programs at once, so this runs a
 * batch and the flash count drops with it. What it costs is that the programs
 * no longer end -- `RVTEST_PASS` and `RVTEST_FAIL` jump here instead of
 * spinning, under BOARD_SUITE, so the next one can start.
 *
 * WHAT IT DOES NOT ISOLATE, and this is the honest limit of the arrangement:
 * the programs share a machine. `mtvec`, the CSR file, the reservation and
 * every register survive from one to the next, and nothing here puts them back.
 * A program that passes after another has run is not quite the same statement as
 * a program that passes alone, and where the two disagree the single-program
 * build is the one to believe. Registers and the stack ARE reset below, because
 * those are cheap; the CSRs are not.
 *
 * THE VERDICT ENCODING IS THE SUITE'S OWN. 1 is a pass and `(testnum << 1) | 1`
 * is a failure at that test number, so an odd number always arrived and an even
 * one never can -- which is what makes a missing report distinguishable from a
 * failing one.
 */

#include "riscv_test.h"

  .section .text.init
  .globl _board_start
_board_start:
  /* .data lives in ROM and has to reach RAM before any program reads it. */
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
  /* .bss, likewise: the SPRAM comes up holding whatever it held. */
  la    t0, __bss_start
  la    t1, __bss_end
3:
  bgeu  t0, t1, 4f
  sw    x0, 0(t0)
  addi  t0, t0, 4
  j     3b
4:
  la    t0, board_index
  sw    x0, 0(t0)

/* THE DRIVER'S STATE LIVES IN MEMORY, NOT IN REGISTERS, and that is not a
 * stylistic choice. A suite program owns every register while it runs -- these
 * are instruction tests, and several deliberately walk the whole file -- so an
 * index kept in s0 comes back as whatever the program left there, and a stack
 * pointer kept in sp comes back pointing anywhere. The first version of this
 * held both in registers and trapped to mtvec == 0 partway through the second
 * program. */
run_next:
  la    t0, board_index
  lw    t1, 0(t0)
  la    t2, board_count
  lw    t2, 0(t2)
  bgeu  t1, t2, all_done

  la    sp, __stack_top            /* the program before it may have moved this */
  li    TESTNUM, 0

  la    t0, board_table
  slli  t1, t1, 2
  add   t0, t0, t1
  lw    t0, 0(t0)
  jr    t0                         /* into the program's own _start */

/* RVTEST_PASS and RVTEST_FAIL land here with the verdict in TESTNUM. Nothing
 * else survived the program, so the verdict is stored before anything runs. */
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

  la    t0, board_index
  lw    t1, 0(t0)
  addi  t1, t1, 1
  sw    t1, 0(t0)
  j     run_next

all_done:
  /* A marker the host can stop on, then repeat the whole batch: the report is
   * printed seconds after configuration and nothing is listening yet. */
  li    a0, 46                     /* '.' */
  jal   ra, put_char
  li    a0, 10
  jal   ra, put_char
  li    t0, 3000000
5:
  addi  t0, t0, -1
  bnez  t0, 5b
  la    t0, board_index
  sw    x0, 0(t0)
  j     run_next

/* ---- the wire ------------------------------------------------------------ */

/* Out the UART, and into a RAM buffer as well. The buffer is what makes a batch
 * verifiable BEFORE it is flashed: the cxxrtl runner's `--console` reads a
 * NUL-terminated string out of RAM, so the same report the wire carries can be
 * read from a simulation. Same bytes, one function, so the two cannot differ. */
  .globl board_console
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

  .section .bss
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

// What a C suite program needs to run as a member of a board batch: an entry the driver can jump to,
// the startup copy it calls, and a way to hand its verdict (HTIF encoding, 1 for pass) to the driver.

  .text
  .globl _start
_start:
  call  main
1:
  j     1b

  .globl board_report
board_report:
  mv    gp, a0
  j     board_next

  .globl runtime_init
runtime_init:
  j     init_memory

// The startup for the aggregate measurement: two independently linked Dhrystone copies,
// one per hart, joined by test/dual/bench/dhry_dual.lds.

  .section .text.init,"ax",@progbits
  .globl _start
_start:
  csrr    t0, mhartid
  bnez    t0, hart1

  .option push
  .option norelax
  la      gp, __global_pointer$
  .option pop
  la      sp, __stack_top0
  call    data_init

  .option push
  .option norelax
  la      t1, mailbox
  .option pop
  li      t2, 1
  sw      t2, 0(t1)

  call    h0_main
1:
  j       1b

hart1:
  .option push
  .option norelax
  la      t1, mailbox
  .option pop
2:
  lw      t2, 0(t1)
  beqz    t2, 2b

  .option push
  .option norelax
  la      gp, __global_pointer$
  .option pop
  la      sp, __stack_top1
  call    h1_main
3:
  j       3b

  .section .bss
  .align 2
  .globl mailbox
mailbox:
  .word 0

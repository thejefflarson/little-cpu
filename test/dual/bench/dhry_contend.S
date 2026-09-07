#define CONTEND_WORDS 64

  .section .text.init,"ax",@progbits
  .globl _start
_start:
  csrr    t0, mhartid
  bnez    t0, hart1

  // hart 0: the Dhrystone port, set up exactly as test/crt0.S does --
  .option push
  .option norelax
  la      gp, __global_pointer$
  .option pop
  la      sp, __stack_top
  call    data_init
  call    main
1:
  j       1b

hart1:
  .option push
  .option norelax
  la      t0, contend_buf
  .option pop
  mv      t3, t0
  li      t1, CONTEND_WORDS*4
  add     t1, t0, t1
2:
  lw      t2, 0(t0)
  addi    t2, t2, 1
  sw      t2, 0(t0)
  addi    t0, t0, 4
  bne     t0, t1, 2b
  mv      t0, t3
  j       2b

  .section .bss
  .align 2
contend_buf:
  .skip CONTEND_WORDS*4

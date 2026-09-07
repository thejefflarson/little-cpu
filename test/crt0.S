// Startup for the C programs in test/asm.

  .section .text.init,"ax",@progbits
  .globl _start
_start:
  // One hart runs the program.
  csrr    t0, mhartid
  beqz    t0, 1f
  .option push
  .option norelax
  la      t1, __hart_park
  .option pop
2:
  lw      t2, 0(t1)
  j       2b
1:
  .option push
  .option norelax
  la      gp, __global_pointer$
  .option pop
  la      sp, __stack_top
  call    runtime_init
  call    main
1:
  j       1b

  .globl runtime_init
runtime_init:
  la      a0, __data_load_start
  la      a1, __data_start
  la      a2, __data_end
1:
  beq     a1, a2, 2f
  lw      a3, 0(a0)
  sw      a3, 0(a1)
  addi    a0, a0, 4
  addi    a1, a1, 4
  j       1b
2:
  la      a1, __bss_start
  la      a2, __bss_end
3:
  beq     a1, a2, 4f
  sw      x0, 0(a1)
  addi    a1, a1, 4
  j       3b
4:
  ret

  .section .bss
  .align 2
  .globl __hart_park
__hart_park:
  .word 0

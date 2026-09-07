// Startup for the CoreMark image this directory's cross-core harness runs.

  .section .text.init,"ax",@progbits
  .globl _start
_start:
  .option push
  .option norelax
  la      gp, __global_pointer$
  .option pop
  la      sp, __stack_top

  la      a1, __bss_start
  la      a2, __bss_end
1:
  beq     a1, a2, 2f
  sw      x0, 0(a1)
  addi    a1, a1, 4
  j       1b
2:
  call    main
3:
  j       3b

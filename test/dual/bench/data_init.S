// The copy-then-zero pass test/crt0.S's `runtime_init` does, shared by both
// dual Dhrystone builds in this directory rather than typed out twice more.
// Not `runtime_init` itself: linking test/crt0.S in would collide on its own
// `_start`, which both dhry_contend.S and dhry_dual_boot.S need to define
// their own way (a real workload for hart 1, or a second Dhrystone copy,
// where test/crt0.S's `_start` would park either).
  .section .text.init,"ax",@progbits
  .globl data_init
data_init:
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

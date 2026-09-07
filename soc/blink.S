.section .text
.globl _start
_start:
  li   t0, 0x00010000        # any mapped RAM word; the value is what matters
  li   t1, 0
1:
  addi t1, t1, 1
  srli t2, t1, 21
  andi t2, t2, 3
  sw   t2, 0(t0)
  j    1b

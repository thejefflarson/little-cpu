# A jal and a branch whose targets differ from their pcs in bits 15:14; stores 12 to RAM when done.
  .option norvc
  .text
  .globl _start
_start:
  li    s0, 0
  li    s1, 4
  lui   s2, 0x10
  j     far

  .org  0x3ff8
loop:
  addi  s0, s0, 3
  addi  s1, s1, -1
  bnez  s1, loop
  sw    s0, 0(s2)
done:
  j     done

  .org  0xc010
far:
  j     loop

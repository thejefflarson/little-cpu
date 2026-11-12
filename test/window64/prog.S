# Text spread across a 64 KB window, with a jal and a conditional branch whose targets differ
# from their own pcs in bits 15:14: the core's predicted target must carry all sixteen low bits.
# Stores 12 to the first word of RAM once the loop has added three four times.
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

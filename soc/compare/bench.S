// The one program all three harnesses in this directory run.
    .section .text.init
    .globl _start
_start:
    li      t0, 0x00010000          /* soc/compare/bench.lds' ram ORIGIN */
    li      s0, 0                   /* the value every iteration mixes into */

outer:
    li      t1, 0                   /* word index */
    li      t2, 128                 /* half the 512-word RAM, so the byte and
                                       halfword slots below stay clear of it */
inner:
    slli    t3, t1, 2
    add     t3, t0, t3              /* &ram[i] */
    xor     t4, t1, s0
    slli    t5, t4, 3
    add     t4, t4, t5
    sw      t4, 0(t3)
    lw      t6, 0(t3)               /* read back what was just written, so the
                                       memory is in the dependence chain */
    add     s0, s0, t6
    sub     s0, s0, t1
    srli    t5, s0, 7
    xor     s0, s0, t5              /* xor rather than or: an `or` here fills s0
                                       with ones and every publication below
                                       comes out the same, which would let a
                                       stuck core satisfy the bench's
                                       cross-core comparison */
    addi    t1, t1, 1
    blt     t1, t2, inner

    // Deliberately NOT at offset 0 of their words.
    sb      s0, 513(t0)
    lbu     a0, 513(t0)
    sh      s0, 518(t0)
    lhu     a1, 518(t0)
    add     s0, s0, a0
    add     s0, s0, a1

    sw      s0, 520(t0)
    lw      a2, 520(t0)
    add     s0, s0, a2
    j       outer

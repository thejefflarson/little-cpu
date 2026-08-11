/*
 * The one program both harnesses in this directory run.
 *
 * Its job is not to compute anything. Its job is to make the datapath
 * OBSERVABLE, because the first version of this measurement filled the ROM with
 * NOPs and yosys proved the outputs constant and folded the core away: 449
 * placed logic cells against the 1711 the same core synthesises to on its own,
 * and a critical path through the fragment that was left. soc/compare/placed_vs_synth.py
 * is the check that now catches that; this file is what stops it triggering.
 *
 * So every result reaches a store, every store's data comes from a load and an
 * ALU chain fed by an earlier store, and the loop never ends. Byte and halfword
 * traffic is here because the two cores lane their accesses differently -- ours
 * strobes bytes, VexRiscv replicates the store data across all four lanes and
 * shifts the load result itself -- and a harness that only ever moved whole
 * words would leave both spellings unexercised.
 *
 * RV32I ONLY. VexRiscv's checked-in configuration has no M extension, so
 * anything using one could not be one image. That means this core's multiplier
 * and divider are present in the placed netlist but never issued, which does not
 * move a static timing number and is recorded with the result anyway. C is on
 * both sides -- that core has a compressed decoder -- and is left out here only
 * because this program is hand-written and gains nothing from it.
 */
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

    /* Deliberately NOT at offset 0 of their words. Every access this program
       made before these was word-aligned, so VexRiscv's byte strobe -- the size
       mask shifted by the low address bits -- came out the same whether the
       shift was there or not, and deleting it did not fail this bench. Byte
       lane 1 and halfword lane 1 are what make the shift observable. */
    sb      s0, 513(t0)
    lbu     a0, 513(t0)
    sh      s0, 518(t0)
    lhu     a1, 518(t0)
    add     s0, s0, a0
    add     s0, s0, a1

    /* The harnesses' LEDs watch the data bus, so this store is what makes the
       whole chain above reach a pin. */
    sw      s0, 520(t0)
    lw      a2, 520(t0)
    add     s0, s0, a2
    j       outer

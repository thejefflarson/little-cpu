// The startup for the contention measurement: hart 0 runs the unmodified
// Dhrystone port (test/bench/dhry_1.c, dhry_2.c, dhry_port.c) exactly as
// test/crt0.S would set it up, and hart 1 streams real loads and stores
// across a buffer instead of test/crt0.S's park loop.
//
// NOT test/crt0.S -- that file's non-zero-hart path polls a location nothing
// ever writes, so it is real bus traffic but read-only and single-address.
// This measurement's whole point is the price of a SECOND workload, so hart 1
// here does both a load and a store, over more than one address, every pass.
//
// Linked with test/bench/bench.lds unmodified: this is one Dhrystone copy,
// the ROM and RAM budgets are the SoC's real 8 KB / 64 KB, and the buffer
// below is a few hundred bytes of `.bss` the linker packs beside Dhrystone's
// own globals with no script changes. `data_init` is
// test/dual/bench/data_init.S, linked alongside this file.
#define CONTEND_WORDS 64

  .section .text.init,"ax",@progbits
  .globl _start
_start:
  csrr    t0, mhartid
  bnez    t0, hart1

  // ---- hart 0: the Dhrystone port, set up exactly as test/crt0.S does ----
  .option push
  .option norelax
  la      gp, __global_pointer$
  .option pop
  la      sp, __stack_top
  call    data_init
  call    main
1:
  j       1b

  // ---- hart 1: real bus traffic, forever -------------------------------
  // A tight lw/sw stream across CONTEND_WORDS words, wrapping around by
  // resetting to the base rather than re-deriving the address each pass --
  // one load and one store to data RAM per iteration, which is what contends
  // for rtl/busarbiter.v's grant against hart 0's own accesses. `t3` holds
  // the base across the loop instead of a second `la`, which would need its
  // own `.option norelax`: gp is never set on this hart, so a relaxed `la`
  // after the first would compute an address off whatever gp powered up
  // holding.
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

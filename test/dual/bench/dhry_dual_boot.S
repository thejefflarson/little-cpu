// The startup for the aggregate measurement: two independently linked
// Dhrystone copies, one per hart, joined by test/dual/bench/dhry_dual.lds.
// `main`, `tohost`, `dhry_console` and everything else Dhrystone defines are
// reached here as `h0_*`/`h1_*` -- the names `objcopy --prefix-symbols` gives
// each copy so the two can share one link.
//
// Hart 0 does the one data-copy-and-bss-zero pass for BOTH copies (their
// combined `.data`/`.bss` is one region, and `.data` is empty for Dhrystone
// today regardless) and releases hart 1 through `mailbox`, the same shape
// test/dual/smoke.S uses to release its own hart 1. Hart 1 has to wait: its
// own `.bss` -- Arr_2_Glob and the rest -- is inside the range hart 0 is about
// to zero, and starting before that pass finishes would run Dhrystone against
// globals that are not yet in their reset state. `data_init` is
// test/dual/bench/data_init.S, linked alongside this file -- one copy of the
// combined `.data`/`.bss` pass, since `.data` is one region here, not two.

  .section .text.init,"ax",@progbits
  .globl _start
_start:
  csrr    t0, mhartid
  bnez    t0, hart1

  // ---- hart 0: shared init, then its own Dhrystone copy ------------------
  .option push
  .option norelax
  la      gp, __global_pointer$
  .option pop
  la      sp, __stack_top0
  call    data_init

  .option push
  .option norelax
  la      t1, mailbox
  .option pop
  li      t2, 1
  sw      t2, 0(t1)

  call    h0_main
1:
  j       1b

  // ---- hart 1: wait for hart 0's init, then its own Dhrystone copy -------
hart1:
  .option push
  .option norelax
  la      t1, mailbox
  .option pop
2:
  lw      t2, 0(t1)
  beqz    t2, 2b

  .option push
  .option norelax
  la      gp, __global_pointer$
  .option pop
  la      sp, __stack_top1
  call    h1_main
3:
  j       3b

  .section .bss
  .align 2
  .globl mailbox
mailbox:
  .word 0

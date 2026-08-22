// Startup for the C programs in test/asm. It exists because the SoC's data RAM
// is SPRAM, which the bitstream cannot initialise: the only memory that comes
// up holding anything is the block RAM behind `.text`. So test/asm/boot.lds
// links `.data`'s LOAD address into ROM and its virtual address into RAM, and
// the copy below is what moves it before `main` runs.
//
// The assembly programs do not use any of this. Their `.data` is poked straight
// into the simulated RAM by the runner, which is exactly the thing the hardware
// cannot do, and they keep test/asm/sections.lds unchanged.
//
// This lives in test/ rather than test/asm/ because both sim legs take
// test/asm/*.S to BE the suite: a startup file there would be run as a program
// and graded as one.
//
// `runtime_init` is separately callable on purpose. Simulated RAM comes up
// zeroed, so a `.bss` that was never cleared reads the same as one that was,
// and the boot-time pass alone cannot fail. test/asm/datainit.c scribbles over
// both regions and calls this again, which is the same code the reset path ran
// and does have a red direction.

  .section .text.init,"ax",@progbits
  .globl _start
_start:
  // One hart runs the program. Every other one parks, before touching a
  // register the C ABI cares about or a word of memory: `.data` has not been
  // copied out of ROM yet, `.bss` has not been cleared, and there is exactly one
  // stack below. On the single-hart machines this reads zero and falls straight
  // through.
  //
  // PARKING IS POLLING -- this core decodes no `wfi`. It polls a mailbox that
  // nothing in this tree ever writes, and that is deliberate rather than
  // unfinished: a release protocol needs somewhere to release a hart TO, and
  // the second hart's stack, its `.data` and its entry point are all decisions
  // no program here has made. Giving it a word to watch is what makes adding
  // them a change to the program rather than a change to the startup.
  //
  // `norelax` for the reason the `gp` sequence below needs it, arrived at from
  // the other side: the linker relaxes a nearby `la` to gp-relative, and `gp`
  // is not set until AFTER this branch. Relaxed, the parked hart polls an
  // address computed from whatever `gp` powered up holding -- and an address in
  // the text window takes the shared read port from the hart that is trying to
  // run.
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
  // gp first: the linker relaxes nearby data accesses to gp-relative, including
  // ones in runtime_init below. `norelax` stops it relaxing this very sequence
  // into `addi gp, gp, 0`.
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

  // The parked hart's mailbox. In `.bss` so it costs no ROM, and never read by
  // the hart that clears `.bss`, so the clearing race below it does not matter:
  // the parked hart is discarding the word it loads.
  .section .bss
  .align 2
  .globl __hart_park
__hart_park:
  .word 0

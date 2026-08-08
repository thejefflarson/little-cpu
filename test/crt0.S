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

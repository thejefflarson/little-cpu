// Startup for the CoreMark image this directory's cross-core harness runs.
//
// It is test/crt0.S with the `.data` copy removed, for the same reason
// soc/compare/dhry_start.S removes it: soc/compare/bench_vexriscv.v gives its
// core NO DATA PATH TO THE ROM, so a startup that copied `.data` out of ROM
// would copy the real bytes on one core and zeros on another. Instead
// soc/compare/coremark_tb.v pokes the initialised data into every core's RAM
// before the run, the way soc/compare/dhry_tb.v already does for Dhrystone,
// and soc/compare/coremark.lds carries `.rodata` into that same poked region
// rather than leaving it in ROM -- CoreMark's own core_state.c reads string
// tables (`intpat`/`floatpat`/`scipat`/`errpat`) as part of the benchmark
// itself, not only for printing, so those bytes have to be reachable the same
// way `.data` is.

  .section .text.init,"ax",@progbits
  .globl _start
_start:
  // gp first, and `norelax` so the linker does not relax this very sequence
  // into `addi gp, gp, 0`.
  .option push
  .option norelax
  la      gp, __global_pointer$
  .option pop
  la      sp, __stack_top

  // `.bss` is zeroed here even though the poked image already has it zero: on
  // hardware the bitstream is what would have filled the ROM, nothing fills
  // the RAM, and a benchmark whose startup does not clear `.bss` is not the
  // program anybody publishes a number for.
  la      a1, __bss_start
  la      a2, __bss_end
1:
  beq     a1, a2, 2f
  sw      x0, 0(a1)
  addi    a1, a1, 4
  j       1b
2:
  call    main
3:
  j       3b

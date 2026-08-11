// Startup for the Dhrystone image both cores in this directory run.
//
// It is test/crt0.S with the `.data` copy removed, and the reason is a property
// of the harness rather than of either core: soc/compare/bench_vexriscv.v gives
// its core NO DATA PATH TO THE ROM. The instruction memory there is a read port
// for fetch and nothing else, and a load from a ROM address reads back zero, so
// a startup that copied `.data` out of ROM would copy zeros on one core and the
// real bytes on the other -- one image producing two different machines, which
// is the one thing this directory exists to avoid.
//
// So soc/compare/dhry_tb.v pokes the initialised data into BOTH cores' RAM
// before the run, the way test/run_tests.sh pokes an assembly program's `.data`
// into the simulated RAM, and there is nothing to copy here. It happens before
// the first instruction and outside the measured window on both sides.
//
// soc/compare/bench.S needs none of this: it has no `.data` and no `.rodata`
// and reads only what it stored itself, which is why the harness went this long
// without the gap showing.

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
  // hardware the bitstream is what would have filled the ROM, nothing fills the
  // RAM, and a benchmark whose startup does not clear `.bss` is not the program
  // anybody publishes a number for.
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

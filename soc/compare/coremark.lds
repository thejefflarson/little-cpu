/*
 * The memory map the CoreMark run in this directory uses, and the reason
 * that run is a simulation and not a placement -- the same shape
 * soc/compare/dhry.lds already states for Dhrystone.
 *
 * soc/compare/bench.lds is the harness's placed map: 4 KB of ROM and 2 KB of
 * RAM, as much as both cores can be given at once on an hx8k. CoreMark does
 * not fit it and cannot be made to -- its own linked image is roughly four
 * times Dhrystone's 1528-byte `.text` even at RV32IMA with no compressed
 * encodings, and shrinking `TOTAL_DATA_SIZE` to force a fit would stop this
 * from being CoreMark, which is the one thing running it is for. So this is
 * a simulation, exactly the same trade soc/compare/dhry.lds already makes
 * and states: soc/compare/coremark_fit.py prints the block-RAM arithmetic
 * every run, and it is expected to say the image does not fit.
 *
 * `.rodata` IS IN RAM HERE, WITH NO LOAD COPY IN ROM, and for a stronger
 * reason than Dhrystone's own: soc/compare/bench_vexriscv.v gives its core no
 * data path to the ROM at all, and CoreMark's core_state.c reads its own
 * `intpat`/`floatpat`/`scipat`/`errpat` string tables as part of the
 * state-machine benchmark itself, not only to print them. Left in ROM, those
 * reads would return the real bytes on littlecpu and Hazard3 and zero on
 * VexRiscv -- one image computing two different answers, which is exactly
 * what this harness exists to make impossible. So this section has no
 * `AT>rom`, and soc/compare/coremark_tb.v pokes it into every core's RAM
 * before the run the way soc/compare/dhry_tb.v already does for `.data`.
 * soc/compare/coremark_start.S is test/crt0.S with the copy loop removed for
 * the same reason soc/compare/dhry_start.S removes it.
 *
 * Both regions are round numbers comfortably above what the image and its
 * runtime data need (soc/compare/coremark_fit.py checks the image actually
 * fits both what this script states and what soc/compare/coremark_tb.v
 * instantiates), not the smallest pair CoreMark happens to link into --
 * there is no placed budget here to economise against. `rom` measures 10768
 * bytes of `.text` at the RV32IMA build `COMPARE_COREMARK_CFLAGS` compiles this
 * with; `ram` measures 2120 bytes of
 * `.coremarkctl` + `.data` + `.bss` before `TOTAL_DATA_SIZE`'s 2000-byte
 * on-stack buffer and the rest of the call stack.
 */
MEMORY {
    rom(rx)  : ORIGIN = 0x00000000, LENGTH = 16K
    ram(!rx) : ORIGIN = 0x00010000, LENGTH = 16K
}

SECTIONS {
    .text : {
        *(.text.init);
        *(.text);
        *(.text.*);
        . = ALIGN(4);
    } >rom

    /* First in RAM, so the control window is at the region's ORIGIN and
     * soc/compare/coremark_tb.v can watch one constant address on every
     * core's data bus without reading a symbol table. */
    .coremarkctl : {
        *(.coremarkctl);
        . = ALIGN(4);
    } >ram

    .data : {
        __data_start = .;
        __global_pointer$ = . + 0x800;
        *(.rodata);
        *(.rodata.*);
        *(.srodata);
        *(.srodata.*);
        *(.sdata);
        *(.sdata.*);
        *(.data);
        *(.data.*);
        . = ALIGN(4);
        __data_end = .;
    } >ram

    .bss (NOLOAD) : {
        __bss_start = .;
        *(.sbss);
        *(.sbss.*);
        *(.bss);
        *(.bss.*);
        *(COMMON);
        . = ALIGN(4);
        __bss_end = .;
    } >ram

    __stack_top = ORIGIN(ram) + LENGTH(ram);
}

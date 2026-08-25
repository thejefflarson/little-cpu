/*
 * The linker script for `make coremark`. Same Harvard split and the same
 * `.data` load address in ROM that test/bench/bench.lds and test/asm/boot.lds
 * use -- test/crt0.S does not care which program it is copying for -- with
 * ONE region length different from bench.lds: `rom` is 16 KB here, not 8.
 *
 * THAT 16 KB IS test/testbench.v's ROM_WORDS, not this part's. The part's
 * shipping instruction memory is rtl/imemory.v's 2048 words -- 8 KB, what
 * bench.lds enforces for Dhrystone -- and CoreMark does not fit it: several
 * times Dhrystone's 3568 bytes, the same wall test/asm/rvc.S hits at 12256.
 * Linking against the simulator's double-sized ROM instead is what makes a
 * CoreMark figure possible before this part's deferred SPI-flash boot path
 * lands, and it is also why that figure is simulated only: a link that
 * succeeds here is not one rtl/littlesoc.v can boot.
 *
 *   rom  16 KB, test/testbench.v's ROM_WORDS = 4096.
 *   ram  64 KB, the same two SB_SPRAM256KA bench.lds gives Dhrystone.
 */
MEMORY {
    rom(rx)  : ORIGIN = 0x00000000, LENGTH = 16K
    ram(!rx) : ORIGIN = 0x00010000, LENGTH = 64K
}

SECTIONS {
    .text : {
        *(.text.init);
        *(.text);
        *(.text.*);
        *(.rodata);
        *(.rodata.*);
        *(.srodata);
        *(.srodata.*);
        . = ALIGN(4);
    } >rom

    .tohost : {
        *(.tohost);
        . = ALIGN(4);
    } >ram

    .data : {
        __data_start = .;
        __global_pointer$ = . + 0x800;
        *(.sdata);
        *(.sdata.*);
        *(.data);
        *(.data.*);
        . = ALIGN(4);
        __data_end = .;
    } >ram AT>rom
    __data_load_start = LOADADDR(.data);

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

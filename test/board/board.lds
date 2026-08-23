/*
 * The `.S` suite, linked for a board instead of a harness.
 *
 * ONE DIFFERENCE FROM sections.lds AND IT IS THE WHOLE POINT. There, `.data`
 * sits at its virtual address in RAM and test/run_tests.sh pokes it in -- a
 * thing a harness can do and this SoC cannot, because SPRAM comes out of
 * configuration uninitialised. Here `.data` is loaded in `rom` and copied at
 * startup, which is what test/asm/boot.lds already does for the C programs.
 *
 * The other difference is the ROM: 8K, the part's real 2048 words, where
 * sections.lds grants 16K because a simulator's memory is whatever the
 * testbench declares. That is not a formality -- test/asm/rvc.S is 12256 bytes
 * and does not fit a real ROM at all, which nothing said until something tried.
 */
MEMORY {
    rom(rx)  : ORIGIN = 0x00000000, LENGTH = 8K
    ram(!rx) : ORIGIN = 0x00010000, LENGTH = 64K
}

SECTIONS {
    .text : {
        *(.text.init);      /* the driver, first, so reset lands on it */
        *(.text);
        *(.text.*);
        . = ALIGN(4);
    } >rom

    /* Load in rom, run in ram. The three symbols are what the driver copies
     * between; ALIGN(4) on all of them so the copy can move words. */
    . = ALIGN(4);
    __data_load = LOADADDR(.data);
    .data : {
        __data_start = .;
        *(.tohost);
        *(.data);
        *(.data.*);
        *(.rodata);
        *(.rodata.*);
        . = ALIGN(4);
        __data_end = .;
    } >ram AT>rom

    .bss (NOLOAD) : {
        __bss_start = .;
        *(.bss);
        *(.bss.*);
        *(COMMON);
        . = ALIGN(4);
        __bss_end = .;
    } >ram

    /* OUTSIDE [__bss_start, __bss_end) ON PURPOSE. The driver re-zeroes .bss
     * before every program, so anything of its own kept in there -- the program
     * index above all -- would be wiped mid-batch and the batch would restart
     * at program 0 forever. */
    .driver (NOLOAD) : {
        . = ALIGN(4);
        *(.driver);
        . = ALIGN(4);
    } >ram

    __stack_top = ORIGIN(ram) + LENGTH(ram);
}

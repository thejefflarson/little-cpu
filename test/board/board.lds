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
 *
 * The 2 KB inset on `.data` and on `__stack_top` is the layout convention
 * test/asm/boot.lds' header states and the ASSERTs at the bottom grade: a load
 * or store whose base register sits in a mapped window's first or last 2 KB
 * block bubbles a cycle in rtl/decoder.v, because a 12-bit offset could cross
 * the edge and the answer has to come from a flip-flop.
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
     * between; the run address is a 2 KB multiple and both ends are ALIGN(4),
     * so the copy can move words. */
    __data_load = LOADADDR(.data);
    .data ORIGIN(ram) + 2048 : {
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
        __driver_start = .;
        *(.driver);
        . = ALIGN(4);
        __driver_end = .;
    } >ram

    __stack_top = ORIGIN(ram) + LENGTH(ram) - 2048;

    /* The grader. `.data`'s address, and the highest byte the stack can reach --
     * the stack descends and sp is decremented before anything is stored
     * through it, so nothing is ever accessed at __stack_top itself. */
    ASSERT(ADDR(.data) >= ORIGIN(ram) + 2048 &&
           ADDR(.data) < ORIGIN(ram) + LENGTH(ram) - 2048,
           "layout: .data starts in ram's first or last 2 KB block, so every access through a pointer into it pays rtl/decoder.v's one-cycle region wait. Start it one 2 KB block above ORIGIN(ram).")
    ASSERT(__stack_top - 1 >= ORIGIN(ram) + 2048 &&
           __stack_top - 1 < ORIGIN(ram) + LENGTH(ram) - 2048,
           "layout: __stack_top puts the top of the stack in ram's first or last 2 KB block, so every lw/sw off sp there pays rtl/decoder.v's one-cycle region wait. Put it one 2 KB block below ORIGIN(ram) + LENGTH(ram).")
}

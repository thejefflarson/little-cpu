/*
 * The linker script for the benchmark. It is test/asm/boot.lds' map -- same
 * Harvard split, same `.data` load address in ROM for test/crt0.S to copy --
 * with the two region lengths changed to THE SHIPPING SoC'S, so a link that
 * succeeds here is one rtl/littlesoc.v could actually boot:
 *
 *   rom  8 KB, which is rtl/imemory.v's 2048 words of block RAM. The suite's
 *        scripts link against 16 KB because test/asm/rvc.S pads past 8 KB on
 *        purpose; a benchmark has no such excuse, and the point of running one
 *        is to find out whether it fits the part.
 *   ram  64 KB, the two SB_SPRAM256KA rtl/littlesoc.v instantiates.
 *
 * SO THE 8 KB BUDGET IS ENFORCED BY THE LINKER, not by reading a size report
 * afterwards. `.text`, `.rodata` and `.data`'s load image all come out of that
 * one region, and ld reports the overflow in bytes. A Dhrystone that had to be
 * trimmed to fit would produce a number nothing could be compared against, so
 * overflowing here is the correct outcome to report rather than to fix.
 *
 * THE 2 KB INSET ON `.data` AND ON `__stack_top` TRAVELS WITH THE DMIPS FIGURE.
 * It is the layout convention test/asm/boot.lds' header states and the ASSERTs
 * at the bottom grade: a load or store whose base register sits in a mapped
 * window's first or last 2 KB block bubbles a cycle in rtl/decoder.v, because a
 * 12-bit offset could cross the window edge and the answer has to come out of a
 * flip-flop. Dhrystone is stack-heavy, so a script without the inset measures a
 * different number on the same core, and the layout is part of what the DMIPS
 * figure is quoted with.
 */
MEMORY {
    rom(rx)  : ORIGIN = 0x00000000, LENGTH = 8K
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

    /* __data_start is the first thing in this section so LOADADDR(.data) below
     * is that symbol's load address and not some padding in front of it. */
    .data ORIGIN(ram) + 2048 : {
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

    __stack_top = ORIGIN(ram) + LENGTH(ram) - 2048;

    /* The grader. `.data`'s address, and the highest byte the stack can reach --
     * the stack descends and sp is decremented before anything is stored
     * through it, so nothing is ever accessed at __stack_top itself. Both must
     * land in a 2 KB block that is neither `ram`'s first nor its last. */
    ASSERT(ADDR(.data) >= ORIGIN(ram) + 2048 &&
           ADDR(.data) < ORIGIN(ram) + LENGTH(ram) - 2048,
           "layout: .data starts in ram's first or last 2 KB block, so every access through a pointer into it pays rtl/decoder.v's one-cycle region wait. Start it one 2 KB block above ORIGIN(ram).")
    ASSERT(__stack_top - 1 >= ORIGIN(ram) + 2048 &&
           __stack_top - 1 < ORIGIN(ram) + LENGTH(ram) - 2048,
           "layout: __stack_top puts the top of the stack in ram's first or last 2 KB block, so every lw/sw off sp there pays rtl/decoder.v's one-cycle region wait. Put it one 2 KB block below ORIGIN(ram) + LENGTH(ram).")
}

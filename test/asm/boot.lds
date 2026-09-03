/*
 * The linker script for the C programs in this directory. The assembly
 * programs use sections.lds instead, and the difference between the two files
 * is one line: `.data` here has its LOAD address in `rom` and its virtual
 * address in `ram`.
 *
 * That is the whole bootstrap problem. The SoC's data RAM is SPRAM, and no
 * bitstream can initialise SPRAM; block RAM behind `.text` is the only memory
 * that comes up holding anything. A program whose `.data` exists only at its
 * virtual address therefore has no initialised globals on real hardware, which
 * is why the SoC could not run one. test/crt0.S copies
 * [__data_load_start, __data_load_start + (__data_end - __data_start)) into
 * __data_start before main, and zeroes .bss after it.
 *
 * The memory map is sections.lds' -- same Harvard split, same reason `ram` sits
 * past `rom`'s ceiling to keep GNU ld's single flat address space happy, and
 * the same two region sizes. Several files state parts of that map in their own
 * syntax and cannot include one another: test/cxxrtl.cc and test/cosim.cc each
 * carry `ram`'s ORIGIN as `kRamBase`, and test/asm/riscv_test.h carries the
 * timer's base. test/memmap_test.sh compares all of them against the RTL and is
 * what makes this comment a check rather than a request.
 *
 * `.tohost` is the one output section that stays loadable at its own address in
 * `ram`. It is an IO window, not program data: the runners watch the first word
 * of RAM for the verdict from the first cycle, so it must read zero before the
 * copy has run, and re-running the copy mid-program must not clear a verdict
 * already written.
 *
 * `.rodata` stays in `rom` and is read over the data bus, which this core
 * allows (test/asm/textload.S is the program that says so). Copying it would
 * cost the same ROM plus the RAM to land in.
 *
 * THE 2 KB INSET IS THIS CORE'S LAYOUT CONVENTION and the ASSERTs at the bottom
 * are what grade it. rtl/decoder.v answers a load or store's region question off
 * raw register bits only when the base register sits deep inside a mapped
 * window -- in it, and in neither its first 2 KB block nor its last. A 12-bit
 * signed offset reaches 2 KB either way, so a base register in an edge block
 * might cross the edge, the answer has to come from a flip-flop instead, and the
 * access bubbles one cycle. So `.data` starts one 2 KB block above `ram`'s
 * ORIGIN and the stack starts one block below its top, and both kinds of access
 * then issue with no region term in them. `.tohost` keeps the first block: the
 * runners watch the first word of RAM, and a handful of accesses to one word is
 * not what the convention is for.
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

    /*
     * __data_start is the FIRST thing in this section, so LOADADDR(.data)
     * below is the load address of that symbol and not of some padding in
     * front of it. Put anything -- an ALIGN, a symbol, a literal -- above it
     * and the copy reads from the wrong place by however many bytes it added.
     */
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

    /*
     * The grader for the convention the header describes. `.data`'s own address,
     * and the highest byte the stack can reach -- the stack descends and sp is
     * decremented before anything is stored through it, so nothing is ever
     * accessed at __stack_top itself. Both must land in a 2 KB block that is
     * neither `ram`'s first nor its last.
     */
    ASSERT(ADDR(.data) >= ORIGIN(ram) + 2048 &&
           ADDR(.data) < ORIGIN(ram) + LENGTH(ram) - 2048,
           "layout: .data starts in ram's first or last 2 KB block, so every access through a pointer into it pays rtl/decoder.v's one-cycle region wait. Start it one 2 KB block above ORIGIN(ram).")
    ASSERT(__stack_top - 1 >= ORIGIN(ram) + 2048 &&
           __stack_top - 1 < ORIGIN(ram) + LENGTH(ram) - 2048,
           "layout: __stack_top puts the top of the stack in ram's first or last 2 KB block, so every lw/sw off sp there pays rtl/decoder.v's one-cycle region wait. Put it one 2 KB block below ORIGIN(ram) + LENGTH(ram).")
}

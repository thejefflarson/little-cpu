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
 * past `rom`'s ceiling to keep GNU ld's single flat address space happy. The
 * two files have to agree about it; test/testbench.v's RAM_BASE and
 * test/cxxrtl.cc's kRamBase are the third and fourth copies.
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
 */
MEMORY {
    rom(rx)  : ORIGIN = 0x00000000, LENGTH = 16K
    ram(!rx) : ORIGIN = 0x00010000, LENGTH = 4K
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

/* The linker script for the benchmark. */
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

    ASSERT(ADDR(.data) >= ORIGIN(ram) + 2048 &&
           ADDR(.data) < ORIGIN(ram) + LENGTH(ram) - 2048,
           "layout: .data starts in ram's first or last 2 KB block, so every access through a pointer into it pays rtl/decoder.v's one-cycle region wait. Start it one 2 KB block above ORIGIN(ram).")
    ASSERT(__stack_top - 1 >= ORIGIN(ram) + 2048 &&
           __stack_top - 1 < ORIGIN(ram) + LENGTH(ram) - 2048,
           "layout: __stack_top puts the top of the stack in ram's first or last 2 KB block, so every lw/sw off sp there pays rtl/decoder.v's one-cycle region wait. Put it one 2 KB block below ORIGIN(ram) + LENGTH(ram).")
}

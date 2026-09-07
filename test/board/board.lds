/* The `.S` suite, linked for a board instead of a harness. */
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

    .driver (NOLOAD) : {
        . = ALIGN(4);
        __driver_start = .;
        *(.driver);
        . = ALIGN(4);
        __driver_end = .;
    } >ram

    __stack_top = ORIGIN(ram) + LENGTH(ram) - 2048;

    ASSERT(ADDR(.data) >= ORIGIN(ram) + 2048 &&
           ADDR(.data) < ORIGIN(ram) + LENGTH(ram) - 2048,
           "layout: .data starts in ram's first or last 2 KB block, so every access through a pointer into it pays rtl/decoder.v's one-cycle region wait. Start it one 2 KB block above ORIGIN(ram).")
    ASSERT(__stack_top - 1 >= ORIGIN(ram) + 2048 &&
           __stack_top - 1 < ORIGIN(ram) + LENGTH(ram) - 2048,
           "layout: __stack_top puts the top of the stack in ram's first or last 2 KB block, so every lw/sw off sp there pays rtl/decoder.v's one-cycle region wait. Put it one 2 KB block below ORIGIN(ram) + LENGTH(ram).")
}

/* The linker script for the aggregate measurement: two independently linked Dhrystone
   copies -- one per hart -- in one image. */
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

    .guard ORIGIN(ram) : {
        LONG(0);
    } >ram

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

    __stack_top1 = ORIGIN(ram) + LENGTH(ram) - 2048;
    __stack_top0 = __stack_top1 - 6144;

    ASSERT(ADDR(.data) >= ORIGIN(ram) + 2048 &&
           ADDR(.data) < ORIGIN(ram) + LENGTH(ram) - 2048,
           "layout: .data starts in ram's first or last 2 KB block, so every access through a pointer into it pays rtl/decoder.v's one-cycle region wait. Start it one 2 KB block above ORIGIN(ram).")
    ASSERT(__stack_top1 - 1 >= ORIGIN(ram) + 2048 &&
           __stack_top1 - 1 < ORIGIN(ram) + LENGTH(ram) - 2048,
           "layout: __stack_top1 puts hart 1's stack in ram's first or last 2 KB block, so every lw/sw off sp there pays rtl/decoder.v's one-cycle region wait.")
    ASSERT(__stack_top0 - 1 >= ORIGIN(ram) + 2048 &&
           __stack_top0 - 1 < ORIGIN(ram) + LENGTH(ram) - 2048,
           "layout: __stack_top0 puts hart 0's stack in ram's first or last 2 KB block, so every lw/sw off sp there pays rtl/decoder.v's one-cycle region wait.")
    ASSERT(__stack_top0 > __bss_end,
           "layout: hart 0's stack top overlaps the combined .bss -- the two Dhrystone copies plus the mailbox no longer fit below it.")

    ASSERT(ADDR(.guard) == ORIGIN(ram),
           "layout: .guard moved off ram's first word, so the runner's tohost poll is no longer reading a word this build keeps at zero.")
    ASSERT(ADDR(.tohost) != ORIGIN(ram),
           "layout: .tohost landed on ram's first word -- a hart's own verdict would end the run before the other hart finishes.")
}

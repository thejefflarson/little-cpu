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

/* The memory map the Dhrystone run in this directory uses, and the reason that run is
   a simulation and not a placement. */
MEMORY {
    rom(rx)  : ORIGIN = 0x00000000, LENGTH = 8K
    ram(!rx) : ORIGIN = 0x00010000, LENGTH = 16K
}

SECTIONS {
    .text : {
        *(.text.init);
        *(.text);
        *(.text.*);
        . = ALIGN(4);
    } >rom

    .dhryctl : {
        *(.dhryctl);
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

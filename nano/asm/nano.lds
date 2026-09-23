/* One address space: nano's bus is Von Neumann, not littlecpu's Harvard rom/ram pair. */
MEMORY {
    rom(rx)  : ORIGIN = 0x00000000, LENGTH = 16K
    ram(!rx) : ORIGIN = 0x00010000, LENGTH = 16K
}

SECTIONS {
    .text : {
        *(.text.init);
        *(.text);
        . = ALIGN(4);
    } >rom

    .tohost : {
        *(.tohost);
        . = ALIGN(4);
    } >ram

    /* One word, always reserved here so nano_testbench.v can hardcode its address. */
    .irqctl : {
        . = ALIGN(4);
        __irqctl = .;
        . += 4;
    } >ram

    .data : {
        *(.data);
        *(.rodata);
        *(.rodata.*);
        . = ALIGN(4);
    } >ram

    .bss : {
        *(.bss);
        . = ALIGN(4);
    } >ram

    __stack_top = ORIGIN(ram) + LENGTH(ram);
}

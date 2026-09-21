/* Same shape as nano/asm/nano.lds, with ram moved to PSRAM_BASE: nano_bus subtracts that
   base before indexing the PSRAM model's own array, so ram's origin here is what makes
   the model's word 0 the program's first RAM word. */
MEMORY {
    rom(rx)  : ORIGIN = 0x00000000, LENGTH = 16K
    ram(!rx) : ORIGIN = 0x10000000, LENGTH = 16K
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

MEMORY {
    rom(rx)  : ORIGIN = 0x00000000, LENGTH = 4K
    ram(!rx) : ORIGIN = 0x00010000, LENGTH = 64K
}

SECTIONS {
    .text : {
        *(.text.init);
        *(.text);
        *(.text.*);
        . = ALIGN(4);
    } >rom
}

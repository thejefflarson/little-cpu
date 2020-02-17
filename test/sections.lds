MEMORY {
    rom(rx): ORIGIN = 0, LENGTH = 96k
    ram(!rx): ORIGIN = 0, LENGTH = 32k
}

SECTIONS {
    ROM : {
        . = 0x00000000;
        *(.text);
        *(*);
        . = ALIGN(4);
    } >rom
}

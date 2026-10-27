/* A C program linked to run after the flash loader: ROM starts at BOOT_IMAGE_BASE. The Makefile defines
   BOOT_IMAGE_BASE and BOOT_ROM_BYTES from soc/boot_layout.h. */
MEMORY {
    rom(rx)  : ORIGIN = BOOT_IMAGE_BASE, LENGTH = BOOT_ROM_BYTES - BOOT_IMAGE_BASE
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
}

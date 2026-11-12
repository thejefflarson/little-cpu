/* The flash loader as the block-RAM boot ROM at the top of the SPRAM-text board's window. */
MEMORY {
    rom(rx)  : ORIGIN = BOOT_SPRAM_LOADER_BASE, LENGTH = 1K
    ram(!rx) : ORIGIN = 0x00010000, LENGTH = BOOT_SPRAM_RAM_BYTES
}
SECTIONS {
    .text : { *(.text.init); *(.text); *(.text.*); *(.rodata); *(.rodata.*); *(.srodata); *(.srodata.*); . = ALIGN(4); } >rom
    .data ORIGIN(ram) : { __data_start = .; __global_pointer$ = .; *(.sdata); *(.sdata.*); *(.data); *(.data.*); . = ALIGN(4); __data_end = .; } >ram AT>rom
    __data_load_start = LOADADDR(.data);
    .bss (NOLOAD) : { __bss_start = .; *(.sbss); *(.sbss.*); *(.bss); *(.bss.*); *(COMMON); . = ALIGN(4); __bss_end = .; } >ram
    __stack_top = ORIGIN(ram) + LENGTH(ram);
}

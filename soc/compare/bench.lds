/*
 * The memory map both harnesses in this directory implement, and the only one.
 * soc/compare/bench_littlecpu.v and soc/compare/bench_vexriscv.v are held to it
 * by their ROM_WORDS and RAM_WORDS parameters and by rtl/memory.v's BASE, which
 * both instantiate at its default.
 *
 * It is deliberately NOT the shipping map in test/asm/sections.lds. Both
 * regions are shrunk so the two designs fit the same part: 4 KB of ROM is
 * 8 SB_RAM40_4K and 2 KB of RAM is 4 more, which leaves room on an hx8k's 32
 * blocks for a register file of either core's size. The distortion is the
 * point of the harness rather than a shortcut -- one geometry, two cores.
 *
 * The program has no .data and no .bss, so there is no load-address trick here
 * and no startup copy: everything it reads it stored itself. That keeps the
 * image identical for both cores, neither of which then needs a runtime the
 * other does not have.
 */
MEMORY {
    rom(rx)  : ORIGIN = 0x00000000, LENGTH = 4K
    ram(!rx) : ORIGIN = 0x00010000, LENGTH = 2K
}

SECTIONS {
    .text : {
        *(.text.init);
        *(.text);
        *(.text.*);
        . = ALIGN(4);
    } >rom
}

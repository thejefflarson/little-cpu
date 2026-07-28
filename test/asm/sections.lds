/*
 * Two regions, per ADR-0008. This core is Harvard: imem_addr indexes ROM,
 * mem_addr indexes RAM, and the two buses are physically disjoint — they
 * are not two windows onto one address space. A single `*(*)` catch-all
 * (the old version of this file) put .text and .data in the same output
 * section at address 0, so every TEST_DATA block linked into ROM, where no
 * load could ever reach it.
 *
 * GNU ld's MEMORY command still models one flat linker address space (it
 * rejects two regions whose byte ranges overlap, even when the hardware
 * backing them is Harvard), so `ram` is placed well past `rom`'s ceiling
 * purely to keep the linker happy. `rom` and `ram` are sized generously
 * above the largest measured `.text`/`.data` in this suite (~1.5K text,
 * <64B data) so headroom, not link-map tightness, is the constraint.
 * test/testbench.v's simulation ROM/RAM arrays are sized to match (see the
 * comment there); the cxxrtl runner subtracts `ram`'s ORIGIN back out when
 * it pokes RAM contents via debug_items, and the iverilog leg's live bus
 * decode does the same at runtime.
 */
MEMORY {
    rom(rx)  : ORIGIN = 0x00000000, LENGTH = 8K
    ram(!rx) : ORIGIN = 0x00010000, LENGTH = 4K
}

SECTIONS {
    .text : {
        *(.text.init);
        *(.text);
        . = ALIGN(4);
    } >rom

    .data : {
        *(.tohost);
        *(.data);
        *(.rodata);
        *(.rodata.*);
        . = ALIGN(4);
    } >ram

    .bss : {
        *(.bss);
        . = ALIGN(4);
    } >ram
}

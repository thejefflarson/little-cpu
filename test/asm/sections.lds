/*
 * Two regions. This core is Harvard: imem_addr indexes ROM,
 * mem_addr indexes RAM, and the two buses are physically disjoint — they
 * are not two windows onto one address space. A single `*(*)` catch-all
 * (the old version of this file) put .text and .data in the same output
 * section at address 0, so every TEST_DATA block linked into ROM, where no
 * load could ever reach it.
 *
 * GNU ld's MEMORY command still models one flat linker address space (it
 * rejects two regions whose byte ranges overlap, even when the hardware
 * backing them is Harvard), so `ram` is placed well past `rom`'s ceiling
 * purely to keep the linker happy.
 *
 * BOTH REGIONS ARE THE SIMULATED MACHINE'S, not headroom picked by hand, and
 * test/memmap_test.sh fails if they stop being: `ram` is rtl/memory.v's 64 KB
 * at its base, and `rom` is test/testbench.v's 16 KB. That ROM is larger than
 * the SoC's 8 KB on purpose -- test/asm/rvc.S's page-boundary-straddle case
 * alone pads past 8K (`.align 12` then `.skip 4094`) before the rest of the
 * suite's ~1.5K text starts -- so an `.S` program's fit on the part is decided
 * by soc/rom_banks.py instead, when `make soc-timing` builds its image.
 *
 * Do not shrink either region back to "enough for this suite". `ram` said 4 KB
 * while both machines had 64 KB, and every program fit, so nothing said so.
 *
 * The cxxrtl runner subtracts `ram`'s ORIGIN back out when it pokes RAM
 * contents via debug_items, and the iverilog leg's live bus decode does the
 * same at runtime.
 *
 * This is the script for the assembly programs, and their `.data` reaches RAM
 * only because a harness put it there. The C programs use test/asm/boot.lds,
 * which links the same map with `.data`'s load address in `rom` so the startup
 * can copy it -- the only shape the SoC can boot, since its SPRAM cannot be
 * initialised from a bitstream.
 *
 * `.tohost` IS ITS OWN OUTPUT SECTION so that `.data` can have its own address.
 * rtl/decoder.v answers a load or store's region question off raw register bits
 * only when the base register sits deep inside a mapped window -- in it, and in
 * neither its first 2 KB block nor its last -- because a 12-bit signed offset
 * reaches 2 KB either way, and an access whose base is in an edge block bubbles
 * a cycle while the answer comes out of a flip-flop. `.data` therefore starts
 * one 2 KB block above `ram`'s ORIGIN, which is the same convention
 * test/asm/boot.lds' header states at length and the ASSERT below grades.
 * `.tohost` keeps the first block: the runners watch the first word of RAM.
 */
MEMORY {
    rom(rx)  : ORIGIN = 0x00000000, LENGTH = 16K
    ram(!rx) : ORIGIN = 0x00010000, LENGTH = 64K
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

    .data ORIGIN(ram) + 2048 : {
        *(.data);
        *(.rodata);
        *(.rodata.*);
        . = ALIGN(4);
    } >ram

    .bss : {
        *(.bss);
        . = ALIGN(4);
    } >ram

    ASSERT(ADDR(.data) >= ORIGIN(ram) + 2048 &&
           ADDR(.data) < ORIGIN(ram) + LENGTH(ram) - 2048,
           "layout: .data starts in ram's first or last 2 KB block, so every access through a pointer into it pays rtl/decoder.v's one-cycle region wait. Start it one 2 KB block above ORIGIN(ram).")
}

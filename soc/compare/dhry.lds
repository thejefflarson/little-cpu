/*
 * The memory map the Dhrystone run in this directory uses, and the reason that
 * run is a simulation and not a placement.
 *
 * soc/compare/bench.lds is the harness's placed map: 4 KB of ROM and 2 KB of
 * RAM, which is 12 of an hx8k's 32 block RAMs and as much as both cores can be
 * given at once. Dhrystone does not fit it and cannot be made to: Arr_2_Glob is
 * 10 KB on its own, and trimming the benchmark would produce a number nothing
 * could be compared against, which is the one thing running it is for. So this
 * map is the smallest power-of-two pair of regions the unedited benchmark links
 * into, both regions are larger than anything the part can hold, and the run is
 * iverilog-only.
 *
 * BOTH CORES RUN THIS ONE MAP. The whole point of the directory is that a
 * cross-core number comes from one geometry; a Dhrystone measured at one size
 * on one core and another size on the other would reproduce exactly the
 * incomparability this harness exists to remove.
 *
 * The two ORIGINs are soc/compare/bench.lds', unchanged, so the reset vector and
 * the RAM base are the ones both harness tops already implement.
 *
 * DELIBERATELY NOT INSET, AND DO NOT "FIX" IT. Every linker script under test/
 * now starts `.data` one 2 KB block above its RAM origin and puts the stack one
 * block below the top, because littlecpu's decoder answers a load or store's
 * region question off raw register bits only when the base register is that far
 * from a window edge. Doing it here would move ONE side of a two-core
 * measurement whose product is already owed a re-take on one tree, which is the
 * error this whole directory exists to prevent. The layout is a variable this
 * comparison should set for both cores at once, on the day it is re-run.
 */
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

    /* First in RAM, so the control window is at the region's ORIGIN and
     * soc/compare/dhry_tb.v can watch one constant address on both data buses
     * without reading a symbol table. */
    .dhryctl : {
        *(.dhryctl);
        . = ALIGN(4);
    } >ram

    /* .rodata IS IN RAM HERE, WITH NO LOAD COPY IN ROM, and that is a fact
     * about the measurement rather than a tidy-up. soc/compare/bench_vexriscv.v
     * gives its core no data path to the ROM at all -- the instruction memory
     * there is a read port for fetch and nothing else, and a load from a ROM
     * address reads back zero. Dhrystone is string literals, so anything read
     * out of ROM would be zeros on one core and real bytes on the other, which
     * is not one image running on two machines.
     *
     * So this section has no `AT>rom` and soc/compare/dhry_tb.v pokes it into
     * both cores' RAM before the run, which is what test/run_tests.sh does for
     * the assembly suite. soc/compare/dhry_start.S is test/crt0.S with the copy
     * loop removed for the same reason. The alternative was a bus mux in that
     * harness top, i.e. editing one of the two designs being compared. */
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

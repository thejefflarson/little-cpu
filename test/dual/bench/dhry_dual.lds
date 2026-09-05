/*
 * The linker script for the aggregate measurement: two independently linked
 * Dhrystone copies -- one per hart -- in one image. test/bench/bench.lds
 * cannot be reused unmodified: that script gives the whole machine to one
 * hart, one stack and one `.tohost` word at `ram`'s origin that a runner
 * polls for a shared verdict. Two copies need two stacks, and this build has
 * no shared verdict for a runner to poll -- see the guard word below.
 *
 * ROM and RAM budgets are the SAME 8 KB / 64 KB test/bench/bench.lds
 * enforces, because littledual.v -- the RTL this configuration actually
 * builds -- shares ONE `imemory` instance between both harts (one fetch
 * window, not two): rtl/littledualsoc.v instantiates it at ROM_WORDS(2048),
 * the SoC's real 8 KB, and the two harts' `Dhrystone`s have to fit inside it
 * together. `ram` is rtl/memory.v's 64 KB, likewise shared and unmodified by
 * NHARTS.
 *
 * ONLY ONE COPY OF DHRYSTONE'S OBJECT CODE IS COMPILED. The two linked into
 * this image are the same .o files run twice through
 * `objcopy --prefix-symbols`, which renames every symbol a copy defines or
 * references -- `main` to `h0_main`/`h1_main`, `tohost` to
 * `h0_tohost`/`h1_tohost`, and so on -- consistently within each copy, so the
 * two can share one link with no collision and no line of Dhrystone's source
 * touched.
 *
 * THE 2 KB INSET TRAVELS HERE TOO. `.data` (empty for this benchmark, but
 * still asserted) starts one block above `ram`'s origin, same as
 * test/bench/bench.lds; each hart's stack top sits one block below where its
 * own descent could reach `ram`'s last block or the other hart's stack. Two
 * insets, not the convention's usual one, and both are asserted below for the
 * same reason a single one is: nobody remembers a linker script's addresses
 * by reading the RTL.
 *
 * NO SHARED VERDICT. A one-hart Dhrystone build ends the simulation itself,
 * by writing to `tohost` at `ram`'s first word, which every runner polls. Two
 * harts finish at two different times under contention, and stopping the run
 * at the first one to finish would truncate whichever hart is still going --
 * exactly the failure test/dual_smoke.sh's `--hold-hart1` direction exists to
 * catch in a different shape. So the guard word below occupies `ram`'s first
 * word instead, always zero, and BOTH `tohost` copies land somewhere else --
 * this build is read out with a fixed `--cycles` budget and `--console` on
 * each hart's own report, never with a poll for PASS.
 */
MEMORY {
    rom(rx)  : ORIGIN = 0x00000000, LENGTH = 8K
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

    /* Always zero, and read by nothing: it exists only so `ram`'s first word,
     * which every runner's tohost poll watches, cannot come from either
     * copy's `tohost`. */
    .guard ORIGIN(ram) : {
        LONG(0);
    } >ram

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

    /* Hart 1's stack is the topmost 6 KB, hart 0's the 6 KB below it, both
     * inset from `ram`'s edges by the same margin test/bench/bench.lds
     * carries for one stack. Dhrystone's own call depth is a few frames; 6 KB
     * each is headroom, not a measured requirement. */
    __stack_top1 = ORIGIN(ram) + LENGTH(ram) - 2048;
    __stack_top0 = __stack_top1 - 6144;

    ASSERT(ADDR(.data) >= ORIGIN(ram) + 2048 &&
           ADDR(.data) < ORIGIN(ram) + LENGTH(ram) - 2048,
           "layout: .data starts in ram's first or last 2 KB block, so every access through a pointer into it pays rtl/decoder.v's one-cycle region wait. Start it one 2 KB block above ORIGIN(ram).")
    ASSERT(__stack_top1 - 1 >= ORIGIN(ram) + 2048 &&
           __stack_top1 - 1 < ORIGIN(ram) + LENGTH(ram) - 2048,
           "layout: __stack_top1 puts hart 1's stack in ram's first or last 2 KB block, so every lw/sw off sp there pays rtl/decoder.v's one-cycle region wait.")
    ASSERT(__stack_top0 - 1 >= ORIGIN(ram) + 2048 &&
           __stack_top0 - 1 < ORIGIN(ram) + LENGTH(ram) - 2048,
           "layout: __stack_top0 puts hart 0's stack in ram's first or last 2 KB block, so every lw/sw off sp there pays rtl/decoder.v's one-cycle region wait.")
    ASSERT(__stack_top0 > __bss_end,
           "layout: hart 0's stack top overlaps the combined .bss -- the two Dhrystone copies plus the mailbox no longer fit below it.")

    /* The no-shared-verdict property test/dual_cxxrtl.cc's tohost poll relies
     * on: it reads ram's first word every cycle as a global PASS/FAIL, so
     * that word must be the always-zero guard and never either copy's
     * `tohost` -- a property every other invariant here states as an
     * ASSERT, and this one is no less load-bearing for being about what is
     * ABSENT from an address. */
    ASSERT(ADDR(.guard) == ORIGIN(ram),
           "layout: .guard moved off ram's first word, so the runner's tohost poll is no longer reading a word this build keeps at zero.")
    ASSERT(ADDR(.tohost) != ORIGIN(ram),
           "layout: .tohost landed on ram's first word -- a hart's own verdict would end the run before the other hart finishes.")
}

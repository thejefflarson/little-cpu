# 0158 — The linker scripts inset `.data` and the stack by one 2 KB block

Status: accepted · a SOFTWARE convention, not a core change. No file under `rtl/` moved, `make fit`
reads the same 4109 `ICESTORM_LC` and `make netlist-digest` reads the same hash.

## Context

ADR-0129 bought precise load and store access faults for a cycle. `rtl/decoder.v` answers the region
question off raw register bits when `reg_rs1` sits **deep** inside a mapped window — in it, and in
neither its first 2 KB block nor its last — because a 12-bit signed offset reaches 2 KB either way,
so the effective address is in the block `reg_rs1` names or one on either side. A base register in an
edge block might cross the window edge, so the answer comes out of a flip-flop and the access bubbles
one cycle.

**Every linker script in this tree had put both of the things a compiled program dereferences most in
exactly those two blocks.** `__stack_top = ORIGIN(ram) + LENGTH(ram)` is the last block of RAM, so
every frame sat in it and every `lw`/`sw` off `sp` paid; `.data` started at `ORIGIN(ram)`, the first
block, so every pointer and gp-relative access paid too. That is the whole of Dhrystone's `REGION`
column — 212 752 of 1 756 248 cycles, 12.1% of the run — and 100.0% of the `.S` suite's load/stores
sat near an edge, which is the figure ADR-0129 recorded as the pessimal case and read as a property
of small hand-written programs. It was a property of the linker script.

The core is not what put the data there. A 64 KB window has 32 blocks and 30 of them are deep, so a
program that keeps its stack and its globals one block in reaches the fast arm on essentially every
access, and one that does not pays a cycle per access. **This core has a layout preference**, and
nothing in the tree stated it or checked it.

## Decision

Give `.data` an address one 2 KB block above `ram`'s ORIGIN, and put `__stack_top` one 2 KB block
below `ram`'s top, in every linker script that describes a machine this core is on:
`test/asm/boot.lds`, `test/asm/sections.lds`, `test/bench/bench.lds`, `test/bench/coremark.lds` and
`test/board/board.lds`. The address goes on the output section, not on a bare top-level `.` — a
`>ram` section takes its address from the region counter and would ignore one.

`.tohost` keeps the first block in all of them. The runners watch the first word of RAM for the
verdict, and a handful of accesses to one word is not what the convention is for. In
`test/asm/sections.lds` that meant splitting `.tohost` out of the single `>ram` output section it
shared with `.data` and `.rodata`, so `.data` could take an address of its own; the ROM load image
gained no padding, because a `.S` program's ROM image is `--only-section=.text` and a `.c` program's
`.data` keeps its `AT>rom` load address immediately after `.text`.

**The placement is graded, not remembered.** Each script carries two linker `ASSERT`s — one over
`ADDR(.data)`, one over `__stack_top - 1`, the highest byte the stack can reach, since `sp` is
decremented before anything is stored through it — requiring both to land in a block that is neither
`ram`'s first nor its last, with a message naming the cost. The link fails otherwise. Ten probes in
`test/probe_gates.sh` plant a bad layout in each script and require the link to fail with that
message, plus a control that links all five shipping scripts; `test/PROBES_EXPECTED` goes 590 → 600.

**`soc/compare/dhry.lds` is deliberately left at the conventional layout.** It feeds the cross-core
comparison whose DMIPS product CLAUDE.md already marks stale, both halves of which have to be
re-taken together on one tree before it can be quoted again. Moving one side of a two-core
measurement that is already owed a re-take would make the re-take harder to read, not easier. The
layout is a variable that comparison should set deliberately, on the day it is re-run, for both
cores at once.

## The measurement

Base commit `dba70e9`, macOS 26.3.1, `riscv64-elf-gcc` 16.2.0 with GNU ld 2.47.20260726, Yosys
0.68+post. Every instrument re-run on the same tree, before and after, with nothing else changed.

| instrument | before | after |
|---|---|---|
| `make dhrystone` cycles | 1 756 248 | **1 543 497** (−12.11%) |
| `make dhrystone` `REGION` column | 212 752 | **2** |
| DMIPS/MHz | 0.664 | **0.758** |
| DMIPS at 12 MHz | 7.97 | **9.10** |
| `make coremark` (16 KB ROM) | 1.811 CoreMark/MHz | **2.013** |
| `make cycles` suite | 42 319 cycles, CPI 2.00 | **41 052**, CPI 1.94 (−2.99%) |
| `make cycles` suite `REGION` | 2 224 | **947** |
| `make fit` | 4109 `ICESTORM_LC` | **4109** |
| `make netlist-digest` | `sha256:1a1d1fe3…` | **identical** |

Dhrystone's near-edge locality counter goes 212 752 accesses (56.6% of its load/stores) to **2**
(0.0%), and its `REGION` column follows it exactly — the counter was built to price this design
without building it, and it prices this one to the cycle as well.

The suite keeps 947 `REGION` cycles rather than reaching zero, and that is the convention working
rather than failing: `.tohost` is in the first block on purpose, and several programs address the
timer, the UART and deliberately unmapped addresses, none of which is three blocks wide and none of
which can reach the fast arm at all.

**Retire counts move by single digits and it is the linker, not the core.** Dhrystone retires
950 430 → 950 427 and the `.S` suite 21 118 → 21 122, because moving `.data` and `__global_pointer$`
changes which `la` sequences ld can relax. `test/OBSERVED_FLOOR` is graded with `>=` and no line
needed to move; `make test` is 74/74 with the failure list matching `test/EXPECTED_FAIL`, and
`make cosim-suite` is 69/74 against `test/COSIM_EXPECTED_FAIL` exactly.

## What this does NOT establish

- **The core did not get faster.** Its cycle count for any given access is exactly what it was.
  What changed is where the shipping firmware keeps its stack and its globals, so it stops paying a
  cost it never had to pay. A program that ignores the convention pays what Dhrystone used to pay,
  on this same core.
- **Any DMIPS or CoreMark figure quoted from here travels with the layout**, the way it already
  travels with the flags, the compiler and the string library. 0.758 DMIPS/MHz and 2.013
  CoreMark/MHz describe this core running a program linked one block in. They are not comparable to
  a number taken on a script that was not, and they are not comparable to another project's unless
  that project's layout is known.
- **Nothing here is a fact about `soc/compare/`.** That harness still links Dhrystone at the
  conventional layout for both cores, so its cycle factor is unchanged and its stale product is
  stale in exactly the same way it was.
- **Not a general result about linker layout.** It is a result about a window that is 32 blocks
  wide, where 30 of them are deep. The timer, the UART and the SPI controller are one or two blocks
  each, so nothing addressed through them reaches the fast arm whatever a script does, and no
  inset would help.
- **It closes nothing ADR-0129 opened.** The eighth stall reason is still there, still bubbles, and
  is still what makes an out-of-region access fault. This spends less time in it; it does not
  remove it.

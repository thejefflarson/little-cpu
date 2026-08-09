# ADR-0085: The memory map has one source, and every restatement is checked

**Status:** Accepted · 2026-08-08 · *Closes the defect class
[ADR-0084](0084-dhrystone-is-the-comparable-number-and-the-raw-share-does-not-transfer.md) found in
`test/testbench.v` and
[ADR-0081](0081-the-data-image-lives-in-rom-and-crt0-copies-it.md) found in `test/OBSERVED_FLOOR`.
No `rtl/` behaviour changes: the parameters removed from `rtl/littlesoc.v` resolved to the values
they still resolve to, and `make soc-timing`'s SPRAM and block-RAM census is the evidence.*

## Context

`test/testbench.v` is what every program in `test/asm` runs against. `rtl/littlesoc.v` is what
places on the part. Where the two describe the same thing differently, **the suite is grading a
machine that does not exist**, and nothing compared them.

That is not hypothetical. It bit three times in one day:

- **The data RAM.** The harness modelled 1024 words (4 KB) against the SoC's 16384 (64 KB) —
  sixteen times smaller. It was invisible for the project's whole life because every `.S` program
  fits under 4 KB, and it surfaced only when Dhrystone needed more.
- **`test/OBSERVED_FLOOR`** had no notion of C programs, so the first one went red on CI for a
  reason unrelated to the defect the floor exists to catch.
- **`.github/workflows/ci.yml`** carried a hand-copied RTL file list claiming it "cannot diverge".
  It had.

One shape each time: **a second description of the design, maintained by hand, with nothing
checking it against the first.** A comment asking two files to agree is the weakest instrument
available, and it is exactly what produced all three.

## What the audit found

Every place the two files describe the same thing, and every third-party copy of the same map:

| # | Fact | What was found | Class |
|---|---|---|---|
| 1 | RAM base `0x0001_0000` | Agreed, stated twice | Should be impossible |
| 2 | RAM size 16384 words | Agreed, stated twice — **this is the axis that already failed** | Should be impossible |
| 3 | Timer base `0x0002_0000` | Agreed, stated twice | Should be impossible |
| 4 | ROM size | 2048 on the part, 4096 simulated | Deliberate |
| 5 | `ram` in `sections.lds` / `boot.lds` | **4 KB, against both machines' 64 KB** | Defect |
| 6 | `rom` in the three linker scripts | Matched by hand | Needs a check |
| 7 | `kRamBase` in `cxxrtl.cc` / `cosim.cc` | Agreed, C++ cannot read a parameter | Needs a check |
| 8 | `MTIMER_BASE` in `riscv_test.h` | Agreed, assembly cannot read a parameter | Needs a check |
| 9 | `SOC_ROM_WORDS` in the Makefile | Agreed, hand-copied from `rtl/littlesoc.v` | Needs a check |
| 10 | RAM top against timer base | Abut, by two comments that had to agree | Needs a check |
| 11 | Reset: POR counter vs. driven input | Real, and correct | Deliberate |
| 12 | LEDs, `btn_n`, the image loader | Present in one, absent in the other | Deliberate |
| 13 | `TEXT_BYTES` in `formal/arbiter.v` | Says outright that the size does not matter | Deliberate |

Item 5 is the finding worth naming. **It is the same defect ADR-0084 fixed, one file over.** The
harness's 4 KB was corrected and the linker scripts' 4 KB was left, so the suite went on linking
every program against a RAM sixteen times smaller than either machine — and `sections.lds`'s
comment still claimed the harness was "sized to match". `test/bench/bench.lds`, written later,
already said 64 KB and Dhrystone already ran with its stack top at the top of it, which is what
shows the number was stale rather than a budget.

## Decision

**Order of preference: share a source, then check, then comment.**

### 1. The shared numbers are shared, not stated twice

`rtl/memory.v` already carried the base and the size as its parameter defaults, and `rtl/timer.v`
already carried its base. `rtl/littlesoc.v` and `test/testbench.v` were both merely *re-stating*
those defaults. **Both files now instantiate `memory` and `timer` with no parameter overrides at
all**, so there is exactly one statement of each number — in the module that implements it — and
the two integrators have nothing left to disagree about.

This costs no new file, no include-path plumbing and no macro: it is a deletion. A shared header
was considered and rejected as strictly worse, because it would have been a *fourth* copy sitting
alongside the module defaults rather than replacing them, and because yosys resolves `` `include ``
relative to the including file while iverilog is configured with `-I./rtl/`, so the two frontends
need different spellings of the same path.

The ROM stays written down in both files, because it is the one size that deliberately differs:
simulation has no block RAM to run out of, and `test/asm/rvc.S`'s page-boundary-straddle case pads
past 8 KB on purpose. It is now the **only** parameter override in either file, which is what makes
the deliberate difference the visible one.

### 2. The linker scripts describe the machine

`sections.lds` and `boot.lds` give `ram` 64 KB, which is what both machines have. The only
behavioural consequence is that `boot.lds`'s `__stack_top` — `ORIGIN(ram) + LENGTH(ram)` — moves
from `0x0001_1000` to `0x0002_0000`, the same value `bench.lds` has always given Dhrystone. Retire
counts are unchanged across the suite.

### 3. Everything that cannot share a parameter is compared

`test/memmap_test.sh` reads `rtl/memory.v` and `rtl/timer.v` as the source and fails when any
restatement disagrees. It runs from `make test`, needs only grep and sed, and covers items 5–10
above plus the two properties that keep the sharing honest:

- **neither integrator overrides `memory` or `timer`** — an override reappearing is the whole
  defect coming back;
- **neither integrator has *stopped* instantiating them** — otherwise the first check passes by
  silence.

It also asserts the simulated ROM is never *smaller* than the part's, which is the direction of
item 4 that would be a defect rather than a convenience.

Seventeen probes in `test/probe_gates.sh` force each comparison red, including the original
defect re-entered as `memory #(.RAM_WORDS(1024))`. The fixture is a copy of the shipping files
rather than a stub tree, so the control is the real repository and every red probe is one edit away
from it.

## Consequences

- A future change to the data RAM's size or base is made in `rtl/memory.v` and is inherited by the
  simulated machine with nothing to keep in step. The same for the timer's base in `rtl/timer.v`.
- Changing the ROM size on the part is still a change in three files — `rtl/littlesoc.v`, the
  Makefile's `SOC_ROM_WORDS` and `test/bench/bench.lds` — but the check now names all three the
  moment they disagree, instead of `soc/rom_banks.py` rejecting a program that fits.
- What this does **not** cover: `test/mem_tb.v`, `test/imem_tb.v` and `test/monitor_tb.v` pick
  small sizes of their own on purpose, because a unit bench that walked a 16384-word array would
  spend the suite's whole runtime proving the same boundary. They are benches of the modules, not
  descriptions of the platform, and are deliberately outside the comparison.
- The reset difference (item 11) is not closed and should not be: the SoC has to make its own reset
  because the FPGA comes out of configuration without one, and the harness is driven by its runner.
  It is already load-bearing elsewhere — every reset value in `rtl/timer.v` is zero partly because
  the cxxrtl runner never clocks an edge under reset, so a non-zero one would be invisible on the
  primary simulator and present on the board.

### Rejected: generate the linker scripts from the RTL

It would close items 5 and 6 by construction rather than by comparison, and it is the right answer
if the map ever grows a third region. Today it buys one file's worth of agreement for a code
generator, a build step and a generated-but-tracked artifact of exactly the kind
`test/monitor.v` already shows to be a standing cost. The comparison is a tenth of the machinery
and fails just as loudly.

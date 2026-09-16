# ADR-0190: The RISC-V gcc is pinned, and the benchmark figures move with it

**Status:** Accepted · 2026-09-14

## What was true

Every test and benchmark build resolved whatever RISC-V cross compiler happened to be on
`PATH`: `for candidate in riscv64-elf-gcc riscv64-unknown-elf-gcc; do ...`, repeated near
verbatim in the Makefile and about twenty shell scripts under `test/`, `soc/` and `nano/`.
riscv-formal, sail-riscv, svlint, Hazard3 and CoreMark are all SHA-pinned or
version-pinned with a `make *-setup` target; gcc was the one tool this repo neither
pinned nor declared floating the way the OSS CAD Suite is.

In practice that meant two different compilers: Homebrew's `riscv64-elf-gcc` 16.2.0 on
the owner's Mac, and the self-hosted CI pool's baked-in `riscv64-unknown-elf-gcc` 13.2.0
(installed there because the pods are `runAsNonRoot` and `apt` cannot install one at
runtime). CLAUDE.md's published Dhrystone and CoreMark figures came from the Mac;
`soc/compare/product.json`, the weekly cross-core stamp merged in #360, came from the
runners. **CLAUDE.md itself already says "the toolchain is part of the stamp"** — for
yosys/nextpnr it names the exact build in every figure it quotes — but gcc was invisible
in that accounting, so two numbers claiming to describe the same RTL disagreed and
nothing said why.

## The decision

**One pinned RISC-V gcc, fetched into the tool cache the way svlint and sail-riscv
are, and every consumer resolves it through `make`.**

- **What's pinned**: xPack's `riscv-none-elf-gcc` **15.2.0-1** (upstream GCC 15.2.0),
  prebuilt for macOS arm64 and Linux x86_64 — the two platforms this repo actually
  builds on — each checked against the SHA-256 xPack publishes for the release asset.
  15.2.0-1 was the newest release at the time this landed; anything from GCC 12 onward
  supports Zkt (`-march=...zkt`), so the version choice is about currency, not a
  floor. Verified by compiling one translation unit at each ISA/ABI pair this repo
  builds: `rv32imac_zicsr_zifencei_zkt`/ilp32 (the suite, Dhrystone, CoreMark),
  `rv32im`/ilp32 (`soc/compare`), `rv32emc` and `rv32em`/ilp32e (`nano/`) — all four
  compile clean.
- **How a build selects it**: `RISCV_GCC_VERSION` in the Makefile (`override`, refuses a
  command-line or environment value, exactly like `SAIL_RISCV_VERSION` and
  `SVLINT_VERSION`), with the SHA-256 for each platform's asset declared beside it.
  `make riscv-gcc-setup` downloads, verifies, and unpacks into
  `$(TOOL_CACHE)/riscv-gcc` (`~/.cache/little-cpu/riscv-gcc`, `XDG_CACHE_HOME`-movable,
  outside the checkout — `test/tool_cache_test.sh` now checks this directory too), the
  same idempotent stamp-and-rehash shape `sail-setup` uses. `mk/toolchain.mk` prepends
  `$(TOOL_CACHE)/riscv-gcc/bin` onto `PATH` whenever that binary exists there, the same
  way it already does for the OSS CAD Suite's `bin/`. Every consumer — the Makefile's six
  inline resolutions, `test/cosim.py`, and about twenty shell scripts — was rewritten
  from "search two candidate names on PATH" to "resolve `riscv-none-elf-gcc`, the one
  name the pin ships, and say `run make riscv-gcc-setup` if it's missing." Nothing
  outside `mk/toolchain.mk` and the setup target names a search path or a fallback
  compiler, so the only way a build reaches a different gcc is an environment that never
  ran `make riscv-gcc-setup` — and `test/riscv_gcc_pin_test.sh`, on `make test`'s path,
  fails loudly rather than silently falling through to whatever else answers to that
  name.
- **CI**: a new composite action, `.github/actions/setup-riscv-gcc`, replaces the
  `apt-get install gcc-riscv64-unknown-elf` / `verify-toolchain riscv64-unknown-elf-gcc`
  pair in every job that compiles a RISC-V program (`test`, `mutation-check-shard`,
  `cosim`, `soc-timing`, `ecp5-timing`, and the weekly `compare-product-schedule`). It
  caches the ~400 MB tarball with `actions/cache`, keyed on the pin (version, asset name,
  digest — `make riscv-gcc-pin` prints it, mirroring `make sail-pin`), and always runs
  `make riscv-gcc-setup`, which is a no-op on a verified cache hit. `clang`, used for the
  cxxrtl and co-simulation harnesses and unrelated to this pin, keeps its own
  `verify-toolchain` step.
- **The graded check**: `test/riscv_gcc_pin_test.sh` takes the pinned bin directory and
  fails if `command -v riscv-none-elf-gcc` resolves anywhere else — the failure mode a
  stale `PATH` or a second install earlier in it would otherwise cause silently. Three
  `test/PROBES_EXPECTED` labels force it both ways: the pinned install alone is green, a
  decoy earlier on `PATH` is red and says so, and no compiler at all is red and points at
  `make riscv-gcc-setup`. A fourth extends `test/tool_cache_test.sh`'s existing
  outside-the-checkout check to the new install directory.
- **`make setup`** now runs `riscv-gcc-setup` on both platforms instead of
  `brew install riscv64-elf-gcc` / pointing at `apt-get install gcc-riscv64-unknown-elf`;
  the toolchain paragraph in CLAUDE.md changes with it.

| | before | after |
|---|---|---|
| owner's Mac | Homebrew `riscv64-elf-gcc` 16.2.0 | xPack `riscv-none-elf-gcc` 15.2.0-1 (GCC 15.2.0), pinned |
| CI runners | image-baked `riscv64-unknown-elf-gcc` 13.2.0 | the same pinned 15.2.0-1, fetched by `make riscv-gcc-setup` |
| resolution | first of two names found on `PATH` | one name, one install, graded |

## The figures, re-taken

Every number below is `make dhrystone` / `make coremark` / `make nano-dhrystone` /
`make nano-coremark` run on this tree with the pinned 15.2.0-1, on the owner's Mac.
`make compare-dhrystone` / `make compare-coremark` give the cycle halves of the
cross-core comparison; the clock halves do not depend on gcc and are not re-swept here.

- **littlecpu, `make dhrystone`** (native ISA, `-O2`, 2000 runs): 788 cycles/Dhrystone,
  **0.722 DMIPS/MHz, 8.66 DMIPS at 12 MHz** — was 0.777 DMIPS/MHz, 9.32 DMIPS.
- **littlecpu, `make coremark`** (native ISA, `-O2`, SIMULATED AT 16 KB, 100 iterations):
  46,397,929 cycles, **2.155 CoreMark/MHz** — was 2.203 CoreMark/MHz.
- **littlecpu, `make coremark-rom-up5k`**'s score (`-Os -flto`, same 16 KB simulated
  budget, per-iteration ratio so 20 iterations reproduces the same score ADR-0166's own
  561,530-cycles/iteration measurement did): 563,013.5 cycles/iteration,
  **1.776 CoreMark/MHz** — was 1.780 CoreMark/MHz, a null within rounding: `-Os -flto`
  code is far less sensitive to a compiler-version bump than `-O2`'s is, which is itself
  evidence for pinning rather than against it. The link is 6,996 of 8,192 bytes, under
  budget (was 7,076).
- **nanocpu, `make nano-dhrystone`**: 2516.5 cycles/dhrystone, **0.226 DMIPS/MHz** — was
  0.225.
- **nanocpu, `make nano-coremark`**: 1,860,316 cycles/iteration, **0.538 CoreMark/MHz** —
  was 0.541.
- **cross-core cycle halves, `make compare-dhrystone`** (RV32IM, one shared C binary,
  one iverilog simulation): littlecpu 313,627 cycles (was 290,825), VexRiscv 262,827,
  0.838× littlecpu (was 0.873×), Hazard3 252,026, 0.804× littlecpu (was 0.869×).
  **littlecpu moved the most of the three** — a compiler swap is not guaranteed to move
  every core's C the same amount, the same point ADR-0160 makes about widening the ISA.
- **cross-core cycle halves, `make compare-coremark`** (RV32IM, same harness): littlecpu
  446,995 cycles / 2.237 CoreMark-MHz-equivalent (was 433,240 / 2.308), VexRiscv 426,430,
  0.954× littlecpu (was 0.986×), Hazard3 666,552, 1.491× littlecpu (was 1.536×).

`test/OBSERVED_FLOOR`'s `.S` floors did not move (they never depend on which gcc built
the `.c` programs); no `.c` floor moved either, and none needed a new baseline entry.
CLAUDE.md's own cross-core clock-and-cycle PRODUCT paragraph (the up5k-step and ECP5-Fmax
figures) is left untouched: those are `cycles × clock`, and mixing these freshly-measured
cycles with a clock sweep taken under the old compiler would violate CLAUDE.md's own rule
that "a product is a measurement only when both factors were taken on one tree AND one
toolchain." `soc/compare/product.json` is not re-stamped by this PR for the same reason
ADR-0183 gives for not re-stamping it from a PR branch: its `base` has to be a commit
reachable on `main`, and this repo squash-merges. The weekly
`.github/workflows/compare-product-schedule.yml` re-take, next scheduled after this
lands on `main`, takes the real stamp — both factors, one session — under the pinned
compiler.

## What this does not do

It does not pin clang (the cxxrtl/co-simulation host compiler) or the OSS CAD Suite,
both declared floating already for different reasons. It does not add Windows or
Linux arm64 assets — xPack ships them, but nothing in this repo builds on either
platform, so there is nothing to verify a checksum against; adding one is a small
follow-up if that changes. It does not change any `-march=`/`-mabi=` flag or any RTL —
`test/march_test.sh`'s site count is unmoved, and the figure changes above are entirely
the compiler's code generation, not a behavior change in the design.

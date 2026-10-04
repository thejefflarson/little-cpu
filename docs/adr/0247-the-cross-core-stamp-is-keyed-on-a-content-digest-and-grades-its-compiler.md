# ADR-0247: The cross-core stamp is keyed on a content digest and grades its compiler

**Status:** Accepted · 2026-10-03 · *Amends ADR-0183 (the stamp's `base`), ADR-0232 and ADR-0233.
No `rtl/` change ships from this ADR.*

## Context

`soc/compare/product.json` named the commit it was measured at, and `soc/compare/product_check.py`
graded staleness by diffing `rtl/` and `soc/compare/` against that commit. This repo squash-merges
and deletes branches, so a stamp taken anywhere but `main` names a commit nothing can resolve
(ADR-0183), and the diff never watched the benchmark sources, the Makefile or the riscv-formal pin,
all of which the measurement reads. The stamp's `tools` block was recorded and never graded, so
renaming the compiler (`riscv64-unknown-elf-gcc` to `riscv-none-elf-gcc`, ADR-0190) was not a
reason to re-measure. The run also sampled `dirty` once, before a 60 to 70 minute run, and
interpolated environment and tool strings into `eval` and `python3 -c`.

Two items of the same hardening already landed and are recorded here as done: the weekly
workflow's split into a read-only `measure` job and a `publish` job (ADR-0233, merged), and
`setup-oss-cad-suite`'s environment handling and tag and digest validation (merged with the
security pass). Both were verified on `main` before this change.

## Decision

- **A measured pair carries `digest`, a SHA-256 over the bytes the measurement read.**
  `soc/compare/product_digest.py` hashes every file under `rtl`, `soc/compare`, `test/bench`,
  `Makefile` and `formal/pin.mk` (tracked, plus untracked and not ignored), reading the working
  tree, in the way `soc/soc_pin.py` keys `soc/pin.json`. Blob bytes survive a squash merge where a
  commit id does not. `base` stays in the stamp as information and is still validated as a full SHA.
- **Excluded: `soc/compare/product.json`, `soc/compare/CYCLE_FLOOR` and `docs/comparison.md`.**
  The weekly refresh rewrites the stamp and the document, and `CYCLE_FLOOR` is edited from the
  stamp; digesting any of them would make the stamp stale on its own output, the defect ADR-0183
  fixed for the diff. The cost of the wider input set is deliberate: any `Makefile` edit now
  makes the stamp stale, because the Makefile builds the images and states the flags.
- **A stamp without `digest` is a legacy stamp and is still accepted**, graded as before by
  diffing `rtl/` and `soc/compare/` against its `base`. The committed stamp is one: its digest
  cannot be recomputed honestly for the tree it was measured on, and re-measuring is the weekly
  workflow's job. The first weekly run after this lands writes `digest`.
- **The compiler is graded, keyed by name; every other tool is only recorded.** The caller passes
  `--current compiler=<name>` and `--current compiler_version=<RISCV_GCC_VERSION>`; the stamp
  is stale when its `tools` block has no entry of that name, or when that entry's version string
  lacks the pinned version (`15.2.0-1` is matched as `15.2.0`). `yosys`, `nextpnr-*`, `icetime`,
  `iverilog` and `trellis-db` float with the OSS CAD Suite by design (CLAUDE.md: it is the one
  tool CI takes at the latest release), so equality over the whole block would mark every weekly
  run stale and make every run news. They stay in the stamp as provenance. A stamp with no readable
  `tools` block is stale.
- **`product_write.py` refuses a product whose two factors were measured with different tools.**
  `run_product.sh` samples `soc/print_toolchain.sh` before the clock sweeps (`--tools-block`) and
  again after the cycle simulations (`--cycle-tools-block`); an absent block, a line that is not
  `# NAME: VALUE`, a repeated name, or any difference between the two refuses. The block travels
  as one argument and `product_write.py` splits it, so no tool string reaches `eval`.
- **The tree is re-checked at the end.** Before the first stamp is written and again before
  CoreMark's, `run_product.sh` re-takes `HEAD`, `git status --porcelain --untracked-files=all`
  (less the artifact it rewrites), the digest, and the `HEAD` and status of the gitignored
  opponent clones `soc/compare/hazard3` and `formal/riscv-formal`, and refuses on any difference
  from the start. `dirty` is now the start status including the clones.
- **Input hardening.** Every `COMPARE_PRODUCT_SEEDS` word must be `default` or digits, and
  `run_product.sh` runs with `set -f`. Cycle factors go through a `cycle_factor` function that
  passes numbers as `sys.argv`, after the shell checks each is a count. `product_write.py`
  requires `--seeds` to name as many placements as each core has `--clock-ns` samples, and
  `--digest` to be `sha256:` and 64 hex digits.

Each refusal has a forced-red probe in `test/probe_gates.sh` (groups `product_check.py`,
`product_write.py` and `run_product.sh`).

## Consequences

- `make compare-product`'s own end-of-run `product_check.py` calls, `make compare-dhrystone`'s and
  the weekly workflow's `--require-news` step pass the two compiler fields.
- `product_diff.py --require-news` no longer treats an unresolvable `base` as news for a stamp
  that carries a digest.
- Not re-measured: no stamp, `docs/comparison.md` or `CYCLE_FLOOR` changes here.

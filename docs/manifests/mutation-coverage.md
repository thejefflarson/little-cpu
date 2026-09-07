# `test/MUTATION_COVERAGE`

Every `rtl/*.v` file's ruling on whether a mutation of it is caught by
anything at all.

## Why this exists

`make mutation-check` grades `test/MUTATION_DETECTORS`'s set of pairings
against the set of patches in `test/mutations/`, both ways round — but that
is a claim about the ELEVEN PATCHES that exist, and says nothing about the
FOURTEEN `rtl/*.v` files no patch touches. `rtl/uart.v`, `rtl/spiflash.v`,
`rtl/executor.v` and twelve others could each go without a single line
changing behaviour and nothing on `make mutation-check`'s path would say so,
because nothing there is asked the question. A new file added to `rtl/`
joins that silence by default. This file is the set-equality check
`test/MUTATION_DETECTORS` does not do: against `ls rtl/*.v`, in both
directions, the way `test/OBSERVED_FLOOR`'s name set doubles as the suite
manifest and `test/PROBES_EXPECTED` is a both-ways multiset.

## Format

One line per `rtl/*.v` file:

```
<rtl-file>  <mutation>
<rtl-file>  unpaired  <grader>
```

`<rtl-file>` is a path exactly as `ls rtl/*.v` prints it. `<mutation>` is a
name from `test/MUTATION_DETECTORS`'s first column — a real measured claim
that a mutation of THIS file is caught, in whichever direction
`MUTATION_DETECTORS` already demonstrates. `unpaired` is not an excuse: it
must be followed by the name of the bench (a `test/*_tb.v` run by
`make test-unit-<bench>`) or the formal component task
(`make -C formal components_<task>`) or other named `make` target that DOES
exercise this file, checked against the real thing rather than typed on
faith — a name nothing runs is red the same way a missing file is. A bare
`unpaired` with no grader is a line this file's own grader refuses to parse.

THE EDITORIAL PASS IS THE POINT. Fourteen of the nineteen lines are the
first time anyone asked, file by file, what actually catches a change to it,
rather than what a mutation happens to have been written against. Measure or
read before writing an entry; do not guess a grader's name to make a line
look covered.

Edit this file by hand, in the same commit that adds, removes, or repurposes
an `rtl/*.v` file — the same discipline `test/MUTATION_DETECTORS` and
`test/PROBES_EXPECTED` already carry.

## Files a real mutation patch touches

Each of these five is also paired against at least one OTHER mutation in
`test/MUTATION_DETECTORS`; the entry is one demonstrated pairing, not the
whole set `test/mutations/` has for it.

## Unpaired: caught by a unit bench

- `rtl/executor.v`'s arithmetic is what `test/exec_tb.v` exists to check
  differentially — the riscv-formal checks run under `RISCV_FORMAL_ALTOPS`
  and never touch the real multiplier or divider.
- `rtl/memory.v`'s own bench drives the two corners the suite never reaches:
  an out-of-range access, and the no-change read.
- `rtl/regfile.v`'s own bench pins the two-cycle read contract and the
  write-through bypass — the fabric no `.S` program isolates from the
  scoreboard around it.
- `rtl/regsel.v` has no top-level bench of its own; it is a submodule
  `rtl/decoder.v` instantiates twice (the issuing instruction's registers
  and the operand-fetch guess), and `test/decoder_tb.v`'s `UNIT_BENCH_SRC`
  list compiles it in for exactly that reason.
- `rtl/spiflash.v`'s own bench is the only grader of the controller's shift
  register and its busy bit; co-simulation cannot reach it (plain memory at
  that address) and no `.S` program exercises the SPI pins.
- `rtl/structs.v` declares the inter-stage struct types every pipeline
  module reads and writes; it has no behaviour of its own to mutate, but a
  field reordered, resized, or dropped from `fetcher_output`/
  `decoder_output` would break decode's struct reads immediately.
  `test/decoder_tb.v`'s `UNIT_BENCH_SRC` list compiles it in and is as
  direct a grader as this file gets.
- `rtl/uart.v`'s own bench decodes the transmit line at the configured
  divisor with five of its own failures forced; co-simulation cannot reach
  it (plain memory at that address) and no `.S` program can read back what
  it sent.

## Unpaired: caught by a component proof

- `rtl/busarbiter.v` has no unit bench; it is proved standalone by
  k-induction, with `--keep-going` so a starving handoff and a broken wait
  bound are both reported rather than the first alone.
- `rtl/fetcher.v` has no unit bench; `formal/pcloop.sv`'s no-wrong-path-state
  induction is what its `components_pcloop` task proves, over an instance
  that actually contains this file rather than the standalone fetcher model
  most other formal tasks assume.

## Unpaired: caught by a whole-core grader

- `rtl/littlecpu.v` is the exact module `formal/wrapper.v` instantiates as
  `dut`, so it is what every generated riscv-formal check grades.
- `rtl/writeback.v` drives the `rvfi_*` channel and the register-file write
  those same generated checks compare every cycle; it has no bench of its
  own, but a mutation to what it reports is exactly what `check` exists to
  catch.
- `rtl/littlesoc.v` places on the part but is never instantiated by a
  bench — `test/testbench.v` deliberately restates its map rather than
  instantiating it, because simulation has no block RAM to run out of.
  `test/memmap_test.sh` reads `rtl/littlesoc.v`'s own parameter defaults and
  compares them against `rtl/memory.v`, `rtl/timer.v`, `rtl/uart.v` and
  `rtl/spiflash.v`'s, both ways round, which is the one real check this file
  is under.
- `rtl/littledual.v` is what `test/dual_testbench.v` instantiates directly,
  and `make dual-smoke` is the grader that runs a program on it two ways —
  both harts, and hart 1 held in reset — and checks the shared count moving
  between them against the spec-checking monitor.
- `rtl/littledualsoc.v` adds the pins and the power-on reset around
  `rtl/littledual.v`; nothing simulates it, so its only real grader is
  `make dual-ecp5-timing`, which gates three exact mapping censuses (two
  register files, two multipliers, two ROM copies) before it publishes a
  frequency.

## `rtl/pairtable.v` — `unpaired  check`

A mutation of the learned successor-pair table is **architecturally undetectable by
construction**, and that is a fact about the design rather than a gap in the graders.
Everything the table produces is a *guess* at the next instruction's register pair, and
`operand_stall` in `rtl/decoder.v` lets nothing issue until the pair the register file
holds is the pair the issuing instruction reads. A wrong guess costs a cycle and can
change no architectural value.

Measured, not assumed. Deleting the tag comparison outright — `hit = entry[10]` in place
of the tag equality, the one term that makes an entry belong to the address reading it —
leaves all 75 suite programs passing with the same 22,081 retires, and moves only the
operand column of `make cycles`, 608 cycles to 1448. The one mutation that did go red,
always-hit with an inverted pair, went red as `uart.S BELOW-FLOOR retires` — which is
`test/run_tests.sh`'s 5000-cycle budget running out, not a value comparison. A detector
built on that would be grading slowness.

So `check` is named for what it *grades*, not for what goes red: `formal/wrapper.v`
instantiates `rtl/littlecpu.v`, which instantiates this file, and the generated
per-instruction checks compare `rvfi_rs1_rdata`/`rs2_rdata` against the spec model on
every retire while `reg_ch0` grades the register file under the write-through bypass this
guess feeds. Those are what say the table cannot corrupt a retire whatever it answers.

**The trap this leaves.** The ruling holds only while the guess is checked before use. If
a later change ever consumes the table's output without `operand_stall` standing behind
it, nothing in the mutation suite would notice, because there is no detector here to go
red. That is the line to re-read before touching either.

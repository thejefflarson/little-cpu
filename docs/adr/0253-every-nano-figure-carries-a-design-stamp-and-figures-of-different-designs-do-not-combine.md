# ADR-0253: Every nano figure carries a design stamp, and figures of different designs do not combine

**Status:** Accepted · 2026-10-17. Builds on ADR-0098 (the mixed-configuration error), ADR-0160
and ADR-0232 (a product is a measurement only when both factors share a tree and a toolchain).

## Context

This repo holds two cores that share a name prefix and almost nothing else. littlecpu is
RV32IMAC on block RAM and SPRAM, placed on an ice40 and an ECP5. nanocpu is RV32EC with a QSPI
flash and PSRAM, synthesised to sky130 for a Tiny Tapeout tile. Their areas, cycle counts and
per-MHz scores answer different questions, and the brief says nano "must never be quoted against"
littlecpu. That sentence was prose, and prose is how a nano DMIPS figure had already been read
beside littlecpu's. A figure needs the facts that produced it printed next to it, in a form a
program can refuse on.

## Decision

**A stamped figure ends in one bracket, written by one module.** `nano/figure_stamp.py` holds the
grammar, the list of legal designs and the three operations on them. A figure reads:

    45,120 um2 [nanocpu · rv32ec_zicsr · sky130hd tt_025C_1v80 · yosys 0.69+158 · 1a2b3c4d5e6f]

The fields are design, isa, target, flow and sha, separated by a space, a middle dot and a space.
The order puts what a reader needs to refuse a comparison (which core, which ISA, which part)
before what only an auditor needs (which tool, which commit). The bracket contains no `|`, so it
passes through a markdown table cell and a TSV column unchanged, and the same text serves a
sentence, a table and a log line; a table that is too narrow puts the bracket in its own column
(`make nano-qspi-timing` does). `isa` is the `-march` of `NANO_CFLAGS`, read by the emitter and
never typed. `flow` is `yosys -V`, or the tool a record names. `sha` is the tree's short commit,
with `+dirty` when tracked files differ; a record that predates stamps may say `unrecorded`, and
only `sha` may.

**Three layers enforce it, each with a red direction in `test/probe_gates.sh`.**

1. *Emission.* `make nano-area`, `make nano-timing`, `make nano-dhrystone`, `make nano-coremark`
   and `make nano-qspi-timing` print every figure through `figure_stamp.stamped`, and an
   emitter refuses to run when its build flags carry no `-march`. `figure_stamp.py check` reads
   any output and names each line that holds a figure and no valid stamp; the probes run it
   over every emitter's real output and over one with the stamp deleted.
   `test/figure_stamp_test.py` (on `make test`) reads the tracked files under `nano/` and
   `.github/workflows/nano-*.yml` and names any that print a figure without importing the module,
   so a new emitter cannot be quiet. The simulator's own raw `BENCH ...` line is an input to the
   report and is left unstamped; the report prints the stamped figure under it. The Tiny Tapeout
   hardening workflow prints no figure today and the scan covers it if that changes.
2. *Combination.* `combine` refuses rows that differ in `design`, `isa`, `target` or `flow` and
   names the field; `figure_stamp.py ratio A.json B.json` is the CLI and prints a quotient only
   after `combine` passes. `sha` may differ, because a delta between two commits is the point of
   a ratchet. `soc/compare/` is not extended to nano: the shared thing is the grammar, not the
   harness. `nano/timing_report.py`'s local-to-flow correlation is the one ratio the nano tree
   forms across targets, and it is declared: the record carries a stamp for each half, so both
   print their own provenance.
3. *Prose.* A line in a tracked markdown file that names nano and holds a figure with a unit
   (um2, ps, cycles, DMIPS/MHz, CoreMark/MHz) must carry a valid stamp. The check is per line,
   so a figure wrapped away from the word nano escapes it; that is the cost of a check cheap
   enough to run on every `make test`.

**ADRs numbered before this one are not rewritten.** An ADR is a measurement with a date on it,
and many of them quote nano figures without a bracket. Stamping them now would
invent provenance the author did not record. The prose check therefore applies to ADRs from 0253
on (`FIRST_STAMPED_ADR`) and to every markdown file outside `docs/adr/` and `docs/ideas/`, the
exemption `test/band_source_test.py` already grants for the same reason. The ADR index is
exempt because its rows summarise older records.

**One source, graded.** `test/figure_stamp_test.py` refuses a second file that joins or splits
on the separator or lists both designs as a literal. It catches those two shapes, not every
paraphrase of the grammar; a restatement in other words is a review matter.

## Declined

- *littlecpu's emitters adopt the grammar now.* They already carry stamps with their own fields
  (`soc/bands.py`, `soc/compare/product.json`), and the enum names `littlecpu` so a later
  adoption needs no change here. Moving them is separate work with its own sweep cost.
- *Extending `soc/compare/` to nano.* Its product machinery assumes one part and one memory
  system; nano shares neither.
- *Rewriting dated ADRs.* See above.

## Cost

Zero silicon area; no RTL changed. The emitters gain a required `--cflags` flag, and
`nano/timing_flow_correlation.json` gains two stamps whose `isa` and `sha` are the best the record
supports (`rv32im`, `unrecorded`).

## Open

`NANO_CFLAGS` states `rv32ec_zicsr`, so that is what the stamp names; the core also decodes M
and Zifencei per CLAUDE.md's description, and the build flag is not the full list. Moving the
flag is a `test/march_test.sh` change and is left to whoever settles which string is true.

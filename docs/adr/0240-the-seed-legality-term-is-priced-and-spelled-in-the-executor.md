# 0240 — The `seed` legality term is priced, and the executor compares the address itself

Status: Accepted · 2026-10-02. Measured on `main` at a2251875 (the merged D/X split and Stage C).
No `rtl/` change ships from this ADR.

## The question

The `seed` CSR (0x015, Zkr) is legal only under a read-write access: `csrrw`, or `csrrs`/`csrrc`
with `rs1 != x0` (the instruction writes). A read-only access (`csrrs`/`csrrc` with `rs1 = x0`) must
raise illegal instruction. Making the CSR real therefore adds one legality term, and CLAUDE.md
puts the fetch loop at 12 MHz with no spare period. This ADR prices that term before the ticket
that builds `seed` spends it.

Two facts about where the term lands on this tree. First, the illegal-instruction decision is in
`rtl/executor.v` (`instr_illegal`, built from `instr_valid` and `csr_readonly_write`), not in
`rtl/decoder.v`: D detects nothing about a CSR address, and X sees the address in
`in_instr[31:20]`. Second, `csr_write_op` already exists in X (a CSR access that is not a
`csrrs`/`csrrc` with `rs1 = x0`), so the term is `in_is_csr_access && addr == 0x015 && !csr_write_op`.

## Where the critical path runs today

`make soc-timing` on main (pinned placement, 13.24 MHz, 5,084 of 5,280 `ICESTORM_LC`) reads a path
that starts at `por_done` (the ROM read enable), runs through `imem` to D's `out` registers, the
scoreboard and stall chain (`accessor.in_valid`), D's `in_is_*` and `in_fwd_rs1` register inputs,
`imem.next_word` and `fetch_pc_next`, and ends on `imem.even_data`. No CSR, trap or
`instr_illegal` net is on it. Across the sixteen-seed sweep below the worst path of each placement starts at `por_done`, `accessor_out`, `imem.in_range2` or (once) `regfile.held_rs1`, and ends on `imem` data (once on `timer_mem_rdata`, once on `executor_out`) in all three trees; no worst path starts or ends in the CSR file or the trap cone. The term sits in X's trap cone, which reaches `fetch_pc_next` only through
`x_redirect_q`, a register; it can move the placement, not the path.

## What was built, and thrown away

Both spellings add `SEED` to `rtl/csrs.v`'s read case, so `implemented` is true for 0x015 (it must
be, or the term never fires), and add the term to `instr_illegal`. Neither makes `seed` return
entropy.

- **(a)** `rtl/csrs.v` publishes `csr_destructive = (addr == SEED)`, threaded through
  `rtl/littlecpu.v` to a new `rtl/executor.v` input, and X ORs `in_is_csr_access &&
  csr_destructive && !csr_write_op` into `instr_illegal`.
- **(b)** `rtl/executor.v` compares `csr_addr == 12'h015` itself; no new port.

## Method

`soc/baseline_sweep.sh` on each tree, sixteen placements (`default` and seeds 1..15), one tool
stamp (Yosys 0.68+48 `ff5817c34`, nextpnr-ice40 0.11-1-g62e659ed, icetime oss-cad-suite
20260811, Darwin arm64), run concurrently on one machine on 2026-10-02. Each tree is its own checkout,
so main's sweep ran once and both candidates pair against it by seed. Per-seed rows are in the CSVs
the sweep writes; the table below reproduces them. `soc/baseline_summary.py` refuses to subtract
sweeps with different `base` stamps, and these three differ by design, so the paired deltas were
taken from the CSVs directly.

| | main | (a) | (b) |
|---|---|---|---|
| tree | a2251875 | 07eda124 (throwaway) | 04432d77 (throwaway) |
| packed `ICESTORM_LC` | 5,084 | 5,073 (-11) | 5,144 (+60) |
| worst | 83.09 ns / 12.04 MHz | 84.93 ns / 11.77 MHz | 83.05 ns / 12.04 MHz |
| median | 76.59 ns / 13.06 MHz | 80.12 ns / 12.48 MHz | 77.72 ns / 12.87 MHz |
| best | 74.60 ns / 13.40 MHz | 77.47 ns / 12.91 MHz | 73.99 ns / 13.52 MHz |
| spread (worst over best) | 11.4% | 9.6% | 12.2% |
| placements under 12.00 MHz | 0 of 16 | **2 of 16** (seeds 5 and 15) | 0 of 16 |
| paired period vs main, median | | +5.1% (slower at 14 of 16) | +0.7% (slower at 9 of 16) |
| paired period vs main, worst seed | | +12.6% | +8.3% |

Per-seed MHz, seeds `default, 1..15` in order:

- main: 12.68 13.01 12.85 13.04 13.11 12.61 13.09 13.07 13.40 13.09 12.04 13.29 13.09 12.72 13.04 13.25
- (a): 12.07 12.89 12.15 12.82 12.62 11.93 12.36 12.19 12.49 12.58 12.47 12.60 12.75 12.91 12.41 11.77
- (b): 12.92 12.96 12.73 12.84 12.70 13.10 12.82 13.10 12.77 13.22 13.30 12.63 12.90 12.84 12.04 13.52

## Reading it

Main's sixteen-seed spread (11.4%) is wider than the 4-9% `soc/bands.py` states, and its worst
placement (12.04 MHz, seed 10) clears 12.00 by 0.3%; the placement spread is the standing risk on
this part, not either candidate.

Spelling (a) fails the requirement: two placements land under 12.00 MHz, and its median period is
4.6% above main's (+5.1% median of the per-seed pairs), outside the 3.6% edit-churn band. It is also
the spelling with fewer packed cells than main (-11), which is the measured case CLAUDE.md names:
cell count does not order the period. The likely mechanism is the extra port, which moves how ABC
factors the cone and so where nextpnr places it; this ADR did not isolate it.

Spelling (b) holds 12.00 MHz at sixteen of sixteen. Its worst placement equals main's to the
second decimal (83.05 against 83.09 ns), its median period is 1.5% above main's (+0.7% of the
per-seed pairs), inside the churn band, and it costs 60 cells, leaving 136 of 5,280 free.

## Decision

The follow-on `seed` CSR ticket builds **spelling (b)**: `rtl/executor.v` compares `csr_addr`
against 0x015 and ORs `in_is_csr_access && csr_addr == 12'h015 && !csr_write_op` into
`instr_illegal`, with `SEED` added to `rtl/csrs.v`'s address decode and no new port between
`csrs` and `executor`. Spelling (a) is declined on its own number.

## What this does not settle

- Only the legality term and the decode row were priced. The device behind `seed` (synchroniser,
  counter, corrector, status machine) is real logic, and its sixteen-seed sweep is owed by the
  ticket that adds it (a tied-off port is not free of the mapper, so even the port to
  `entropy_raw` owes one).
- Sixteen seeds on one machine is one draw of the placer's distribution. (b)'s 16 of 16 is
  evidence it clears 12.00 here, with main itself only 0.3% clear at its worst; a final spelling
  with different text can read differently, as the cells above show for a functionally identical
  port, so the follow-on re-takes the sweep on the tree it ships.
- The sweep's `default` row (12.68 MHz on main) is a different placement from `make soc-timing`'s
  pinned one (13.24 MHz, seed 20382078); the sweep never reads the pin.
- The access rule is `!csr_write_op`, the Zicsr suppression rule already in X: `csrrs`/`csrrc`
  with `rs1 = x0` do not write, and `csrrw` always does.

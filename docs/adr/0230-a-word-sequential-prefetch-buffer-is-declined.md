# 0230 — A word-sequential prefetch buffer is declined

Status: Declined · 2026-10-01

## Context

After the fetch refactor (ADR-0221, ADR-0220) the core places on the up5k at 12 MHz and leads
VexRiscv and Hazard3 in cycles there, but on ECP5 it clocks 37.23 MHz worst-of-twelve against their
~53 and ~49. The worst ECP5 path on `main` (11cc506) was the fetch loop: ROM output → decode
(instruction length or the predicted target) → `fetch_pc_next` → ROM address, one cycle.

VexRiscv and Hazard3 keep decode out of that loop by fetching whole words from a registered,
sequential address into a small buffer. The question was whether the same structure, with C kept,
buys enough ECP5 clock to pay for itself.

## What was built

On branch `thejefflarson/jef-1099-a-two-word-prefetch-buffer-that-takes-decode-out-of-the`
(74fe78f, kept, not merged): `rtl/fetcher.v` requests words from a registered `req_addr` advancing by
one word, tracks an outstanding request with an explicit valid bit so a stolen response retries, and
holds a three-word queue that decode reads from registers, a straddling instruction included. A
redirect, a taken guess and `fence.i` flush it. No decode-computed value reaches the ROM address in
the same cycle. `make test` passed, with a randomized `test/fetcher_tb.v` covering straddles,
retries, flushes and rewritten refetch targets.

## Measurement

All against `main` 11cc506, on the real flows, one toolchain.

| | main | prefetch buffer |
|---|---|---|
| ECP5 worst of 12 paired seeds (`soc/paired_sweep.sh`) | 37.23 MHz | 37.36 MHz (+0.3%) |
| ECP5 median | 38.04 MHz | 39.09 MHz (+2.8%) |
| Dhrystone, 2,000 runs (`make dhrystone`) | 1,206,025 cycles, 0.943 DMIPS/MHz | 1,485,709 cycles, 0.787 DMIPS/MHz (+23.2%) |
| CoreMark (`make coremark`) | 2.776 /MHz | 2.220 /MHz (−20.0%) |
| `make fit` | 4,347 | 4,511 (over `FIT_MAX_LC` 4,441) |
| `make soc-timing` LC | 5,084 / 5,280 | 5,186 / 5,280 |
| up5k worst of 8 seeds | 12.57 MHz | 12.08 MHz |

## Why it does not pay

- **The fetch loop was not the only path near the limit.** With it removed, the worst ECP5 path
  moved into X (`reset → executor.out`), at nearly the same period. The clock is bounded by X's
  forward → ALU/branch/address cone as much as by fetch.
- **A taken guess now costs two bubbles where `main` costs none**: the flush cycle presents the
  target and the word reaches decode a cycle later. 344,313 of Dhrystone's 1,485,709 cycles are
  the new "no instruction for D" stall.

A ROM-output bypass into decode would cut the taken-guess cost to one bubble, but puts ROM → decode
back on the critical path the buffer existed to remove.

## Decision

Declined. `main` keeps the stateless fetcher of ADR-0221. Closing more of the ECP5 gap needs X's
period shortened as well as fetch's — a deeper pipeline, with the cells and branch cycles that
costs — not a fetch buffer alone; that is not proposed here.

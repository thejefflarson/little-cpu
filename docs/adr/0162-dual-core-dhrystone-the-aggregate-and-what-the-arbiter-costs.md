# 0162 — Dual-core Dhrystone: the aggregate, and what the arbiter costs

Status: accepted (the measurement) · simulation only, `dual-sim` (cxxrtl); no board, no ECP5
bitstream. Nothing here is a gate or a ratchet, the same standing as `make dhrystone`.

## Context

`make dual-smoke` proves two harts count and is deliberately all it proves: one program, run both
ways, whose shared counter has to move. Nothing in this tree had ever run a benchmark on the DUAL
configuration, so there was no number for what a second hart costs the one that is running, and no
number for what two of them add up to.

Two questions, and they need different programs:

1. **Contention** — with hart 0 running Dhrystone, what does hart 1's own traffic cost hart 0's
   cycles? Tractable with one Dhrystone copy and a real second workload.
2. **Aggregate** — what do two harts running Dhrystone AT ONCE add up to, and at what clock? This
   needs two independently linked copies sharing rtl/littledual.v's one text storage, and the DUAL
   configuration's own clock, not the up5k's.

## The programs

**`test/dual/bench/dhry_contend.S`** is one unmodified Dhrystone (`test/bench/dhry_1.c`, `dhry_2.c`,
`dhry_port.c`, linked with `test/bench/bench.lds` exactly as `make dhrystone` links it) on hart 0,
and a hand-written lw/sw stream over a 64-word buffer on hart 1, forever. Not `test/crt0.S`: that
file's own park loop for a non-zero hart is real bus traffic, but read-only and single-address, and
this measurement's whole point is a workload that both reads and writes. `test/dual/bench/
run_contention.sh` builds it once and runs it twice — contended, and with `--hold-hart1` — reading
hart 0's own `Cycles` line out of its `--console` report both times.

**`test/dual/bench/dhry_dual_boot.S`** and **`dhry_dual.lds`** are the aggregate's: two independently
addressed Dhrystone copies in one image, one per hart. The two copies are the SAME object code —
`test/bench/dhry_1.c`/`dhry_2.c`/`dhry_port.c` compiled once — run twice through
`objcopy --prefix-symbols=h0_`/`h1_`, which renames every symbol a copy defines or references
(`main` to `h0_main`/`h1_main`, `tohost` to `h0_tohost`/`h1_tohost`, and so on) consistently within
each copy. No line of Dhrystone's source is touched, and no symbol collides. Hart 0 does the one
data-copy-and-bss-zero pass both copies' combined `.data`/`.bss` needs, releases hart 1 through a
mailbox (the same shape `test/dual/smoke.S` uses), and each hart then runs its own copy on its own
stack (`__stack_top0`/`__stack_top1`, both inset from `ram`'s edges the way `test/bench/bench.lds`
insets its one stack — ADR-0158's convention, restated for two).

**No shared verdict.** A one-hart Dhrystone ends the simulation by writing `tohost` at `ram`'s first
word, which every runner polls; two harts finish at different times under contention, and stopping
at the first would truncate whichever is still going. `dhry_dual.lds` puts a permanently-zero guard
word at `ram`'s first word instead — neither copy's `tohost` lands there — so the runner runs to a
fixed `--cycles` budget and `run_aggregate.sh` reads BOTH consoles afterward. `test/dual_cxxrtl.cc`
gained `--console <addr>` (repeatable) for this, `test/cxxrtl.cc`'s own flag, printed at every exit
path since this build has none of its own to wait for. The property is asserted, not just arranged:
`dhry_dual.lds` requires the guard section's address to be `ram`'s origin and `.tohost`'s to be
anything else, so a future reordering of `SECTIONS` fails the link instead of silently reviving the
early-exit this design exists to avoid.

**One `data_init`, not three.** Both new firmware files need the same copy-`.data`-then-zero-`.bss`
pass `test/crt0.S`'s `runtime_init` already does; neither can link `crt0.S` itself in, because its
`_start` would collide with the one each file defines its own way. `test/dual/bench/data_init.S`
holds that pass once, and `dhry_contend.S`/`dhry_dual_boot.S` both call it rather than each
retyping the loop.

## The ROM budget, measured before anything else

The programs share one 8 KB text window (`rtl/littledualsoc.v`'s `ROM_WORDS(2048)`) — the DUAL
configuration puts BOTH harts' fetch ports on ONE `imemory` instance, not two, so this is the same
ceiling `test/bench/bench.lds` enforces for one copy, not double it.

| build | `.text` | of 8192 | free |
|---|---|---|---|
| contention (one copy + the stream loop) | 3600 B | | 4592 B |
| aggregate (two copies + the boot) | **5860 B** | | **2332 B** |

Both fit, the aggregate with about 2.3 KB to spare — Dhrystone's own object code duplicates almost
exactly (2 × ~3480 B), and the boot dispatcher adds under 200 B. `.bss` (Arr_2_Glob’s 12 KB per
copy, doubled) is 24748 B against `ram`'s 64 KB, nowhere near tight.

## Measurement 1 — contention

Base commit `122ef7b`, macOS 26.3.1, `riscv64-elf-gcc` 16.2.0, `DHRY_RUNS=2000`,
`DHRY_CFLAGS` unchanged from `make dhrystone`'s own. One image, `dual-sim`, run twice:

| run | hart 0 `Cycles` | DMIPS/MHz |
|---|---|---|
| contended (hart 1 streaming) | 1 768 498 | 0.643 |
| isolated (`--hold-hart1`) | 1 748 026 | 0.651 |
| **delta** | **+20 472 cycles, +1.17%** | |

**This is not `make dhrystone`'s number, in either row.** Both runs go through
`rtl/littledual.v`'s `busarbiter`, whose grant is registered — a hart asks from decode a cycle
before its transaction proceeds — which `littlesoc`'s direct memory ports never pay, present or
absent a second hart. The isolated row is hart 0 alone on the DUAL design, not hart 0 alone on
`littlesoc`; it is 1 748 026 against `make dhrystone`'s own 1 506 943 (ADR-0154) for exactly that
reason, and the two are not comparable — reading them side by side would be the "two measurements
of two different machines in one table" error CLAUDE.md already names. **What IS comparable, and
new, is the delta**: +1.17% is the price hart 1's own load/store stream adds on top of the arbiter's
fixed cost, isolated from it by construction. Hart 1's loop is a pointer that increments and wraps
rather than an address recomputed from scratch each pass, which matters here: fewer instructions per
iteration is a higher issue rate for the SAME workload, so it is a more honest upper bound on what a
tight second workload can cost, not a different measurement.

## Measurement 2 — aggregate

Same commit, same flags, one image, both harts running full Dhrystone concurrently:

| hart | `Cycles` | DMIPS/MHz |
|---|---|---|
| 0 | 1 839 554 | 0.618 |
| 1 | 1 770 553 | 0.642 |
| **combined** | | **1.260 DMIPS/MHz** |

**The clock is ECP5's, from `make dual-ecp5-timing`, never 12 MHz** — the DUAL configuration is
ECP5-only (two fetch windows are 32 block RAMs against the up5k's 30). One placement, `LFE5U-25F-
6CABGA381`, default `ECP5_SEED`, this tree:

```
Fmax : 34.98 MHz (28.59 ns)
```

`make dual-ecp5-timing`'s own output says it plainly: **one placement is a sample, not a
measurement** — no ECP5 band exists for this part (CLAUDE.md), so this is a single seed, not a
twelve-to-sixteen-seed sweep, and is quoted as one.

At that clock: **44.07 DMIPS aggregate** (21.62 + 22.46 per hart). Both halves — DMIPS/MHz from
`dual-sim`, MHz from `dual-ecp5-timing` — are stated together and neither travels alone, the same
rule the cross-core product (ADR-0098) already states for two-factor numbers.

## What this does NOT establish

- **Not a claim about real ECP5 silicon.** No bitstream, no board — an iCESugar-Pro on this machine
  goes dark on any `littledualsoc` bitstream, an open hardware bring-up problem this ticket does not
  touch. Everything above is `dual-sim` (cxxrtl) and `nextpnr-ecp5`'s own placement estimate.
- **Not comparable to `make dhrystone`'s 0.777 DMIPS/MHz / 9.32 DMIPS at 12 MHz.** That is a
  different design (`littlesoc`, no arbiter) at a different, real clock (the up5k's 12 MHz crystal).
  This ADR's numbers are the DUAL design's own, at ECP5's placed clock, and the two must never be
  subtracted or ratioed against each other.
- **A margin and a placement are both perishable.** The 8 KB headroom is this tree's Dhrystone
  object code; a compiler or flag change moves it. The 34.98 MHz figure is one seed on one tree —
  re-take both before spending either.

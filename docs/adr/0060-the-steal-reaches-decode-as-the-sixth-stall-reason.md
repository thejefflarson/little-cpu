# ADR-0060: The steal reaches decode, as the sixth stall reason

**Status:** Accepted · 2026-08-02 · *Builds step 3 of
[`docs/ideas/one-address-space-over-two-memories.md`](../ideas/one-address-space-over-two-memories.md)
on top of [ADR-0059](0059-text-is-writable-and-the-arbiter-lives-in-the-memory.md). Lands
[ADR-0057](0057-what-writable-text-costs-in-ladder-depth-and-in-nanoseconds.md) condition 1 — the
three `[depth]` lines — with F and G re-measured against the shipping RTL rather than inherited.
Amends [ADR-0026](0026-stalls-are-four-reasons-over-two-mechanisms.md) and
[ADR-0009](0009-stall-protocol-semantics.md).
[ADR-0061](0061-fence-i-has-to-serialize.md) is the other half of this change and is separate
because its finding outlives it.*

## Context

ADR-0059 gave `rtl/imemory.v` the write port, the data read and the steal, and left both ends dark:
`mem_ren` was tied low at both instantiation sites and `fetch_stall` was unconnected. In that state
text is writable but not readable, and a store to text corrupts a fetch nobody stalls for.

Turning them on is two wires and one ruling.

## Decision 1 — `mem_ren` is `!reset && in_valid && is_load`, and it is load-bearing

`rtl/accessor.v` already computes that predicate for its load-response stall; the read enable is the
same expression with the reset term the request block applies. The idle bus presents `mem_addr = 0`,
which is inside the text range, so without it the memory would answer every idle cycle as a text
read, steal the fetch window and the core would never run. It is not an optimisation and it is not
cosmetic.

`test/accessor_tb.v` is new and exists for one signal. Stuck *high* is caught elsewhere — the
ladder's environment would steal every cycle and `hang` would go red — but stuck *low* is invisible
to everything else in the tree until a program does a text-region load, and none does yet. The bench
covers the request cycle, the response cycle, a store, an ALU op, a bubble, reset and all five load
widths. `UNIT_BENCHES` goes 7 → 8.

## Decision 2 — the steal bubbles, and a coincident freeze outranks it

`fetch_stall` is the sixth stall *reason* on ADR-0009's existing two mechanisms. It bubbles, exactly
like a RAW hazard and the operand-fetch cycle: `pc` holds and `out <= '0`. There is no flush and
nothing to flush — nothing issued, because `issuing = !reset && !stall` gates trap commit, the CSR
write and `minstret`, and `stall` sits second in `next_pc`'s priority chain above every redirect.

The coincidence with `divider_stall`/`accessor_stall` is the part worth ruling on:

- A **freeze** holds `decoder_out` because the instruction sitting in it has not been consumed yet.
- A **steal** bubbles because the window presented this cycle is a data word, so there is nothing to
  publish.
- On a cycle with **both**, holding wins. Bubbling would drop the held instruction, which nothing
  downstream would notice.

The publish block's arm order already gives that, which is exactly why it is stated rather than
left implied: reordering two `else if` arms is a silent change with no elaboration error and no
failing `.S` program. Both directions are asserted in `rtl/decoder.v`'s own `FORMAL` block and driven
as vectors in `test/decoder_tb.v`. This extends invariant 8's rule (a) rather than adding a
mechanism.

`formal/pcloop.sv` gains `fetch_stall` in `f_may_stall` and an assertion that the steal holds the
pc. That is ADR-0046's trap, avoided deliberately: a new stall reason plus a stale `f_may_stall`
makes the sequential-advance assertion cover a cycle on which the decoder legitimately holds the pc,
and the task goes red on correct RTL. It stayed red on `main` for eight commits the last time.

## Decision 3 — `formal/wrapper.v` models the arbiter; the four hand-authored tasks tie it low

The ladder drives `fetch_stall` from a transcription of `rtl/imemory.v`'s equation,
`(mem_ren || |mem_wstrb) && mem_addr < 0x2000`, registered. Left free it would be an input an
adversarial environment could hold forever, which starves `hang` and `liveness_ch0` — the failure
ADR-0042 found with `imem_data`.

The same-address stability assumption is weakened in the two places ADR-0057 identified: the compare
is dropped on the cycle after any text access (the read port was borrowed, so the window is the data
word) and, after a text *store*, on the cycle after that as well (the array changed, and the fetch
address is published a cycle early). Both are dropped compares rather than modelled values, which is
the wide side — an assumption can only make checks easier, so a depth floor derived under it is the
safe one.

`imemcheck`, `dmemcheck`, `cover` and `complete` tie `fetch_stall` low, and each says so at the
site. They model no memory at all — `imem_data` is free or `rand_const` and answered in the same
cycle — so a steal there would add stall cycles without adding coverage, and would push `cover`'s
five goals and `complete`'s depth-50 walk further out for nothing. The ladder is where a stealing
memory is modelled. What that leaves open is named in the consequences below.

## The measurement: F and G, re-derived here rather than quoted

ADR-0025 requires empirical evidence in either direction before a `[depth]` line moves, and ADR-0057
supplied it from scratch RTL. That RTL is not what shipped, so both sweeps were re-run against this
change, by the recipe in `formal/checks.cfg`.

| sweep | result | ADR-0057's spike |
|---|---|---|
| `hang`, check cycle 6 | `bad state property 0 reachable at bound k = 6` | red |
| `hang`, check cycle 7 | PASS | PASS |
| `liveness`, trig 10, gap 4 | red at k = 14 | red |
| `liveness`, trig 10, gap 5 | red at k = 15 | red |
| `liveness`, trig 10, gap 6 | PASS | PASS |
| `liveness`, trig 15, gap 5 | red at k = 20 | red |
| `liveness`, trig 15, gap 6 | PASS | PASS |

**F = 6, unchanged. G = 6, from 4.** Two triggers rather than one, so the gap figure is not an
artefact of where the trigger was placed, and the gap-5 counterexample is the non-vacuity witness
ADR-0046 describes: `rvfi_liveness_check.sv` opens with `assume(rvfi_valid)` at its trig cycle, so an
unreachable trigger would PASS at every gap instead of failing below 6.

F + G goes 10 → 12 and F + 2G goes 14 → 18, so three lines move: **`insn 15 → 19`**, **`ill 15 → 19`**
and **`reg 15 20 → 15 22`**. Each is its floor plus the one cycle of margin this table already
carries on those two families. Every other line clears 18 with room. The `csrc_*` pair is the one
family F and G do not bound; ADR-0057 re-measured its floor under this environment at 9, unchanged,
and 15 still clears it.

**A green ladder at the old depths would have proved nothing**, which is the reason the depths land
in this change rather than after it: ADR-0057 ran the full ladder at `insn 15` under the proposal's
environment and got 85/85, with three checks green having stopped asking.

## Consequences

- **CLAUDE.md invariant 8 is six reasons over two mechanisms**, and invariant 5 names `fence.i`
  alongside the CSR instructions and `mret` (ADR-0061).
- **The ladder costs about twice the wall time.** ADR-0057 measured 7m14s → 13m42s at `JOBS=4`
  locally; CI's `formal` job was 4m22s against `timeout-minutes: 20`, so the margin narrows from
  about 5× to about 2×. Worth knowing before the next depth increase, which has nowhere left to go
  without a budget conversation.
- **`insn` and `reg` still clear their floors by exactly one cycle.** ADR-0046 called that thin at
  F + 2G = 14 and it is no less thin at 18. A seventh stall reason re-opens all of this.
- **The suite's retire counts did not move**, measured program by program against
  `test/OBSERVED_FLOOR` rather than merely graded against it as a floor. No program in `test/asm`
  does a text-region data access — `.data` is based at `0x10000` and the bench's text ends at
  `0x4000` — so the steal never fires there. The brief predicted zero cost on the current suite and
  that is what was measured; it also means **the suite does not yet exercise this change end to
  end.** The programs that would (`selfmod.S`, `textload.S`, a contention test) are the brief's step
  5 and are not in this change.
- **Four formal tasks now assume no data access ever takes the fetch window.** That is a real
  assumption, recorded at each site in ADR-0049's form. `imemcheck` is the one where it will have to
  be paid off: the brief's step 4 gives it the write path so it can observe a text store, and the
  same change is where a stealing memory belongs in that task.
- **The core side costs no measurable time.** `make soc-timing` at Homebrew Yosys 0.67+post
  (`b8e7da6f`), machine load 9-15: **95.74 / 96.67 / 98.06 ns across three placements = 10.20-10.44
  MHz**, 35.9% logic / 64.1% routing, 34 logic levels (28 LUT + 7 carry), on
  `imem.in_range → imem.rom_even.0.0_RDATA[1]`. ADR-0059 measured 10.18-10.32 MHz for the memory
  half alone, so the two distributions overlap and turning the ports on sits inside the placement
  band. `SOC_MIN_MHZ` holds at every placement, with the same thin margin ADR-0059 recorded.
  `make fit` is **3899 cells** against 3875 before — 24, inside the ±50 churn floor — and under
  `FIT_MAX_LC` 4100. The ruling on ADR-0038's 12 MHz intent was made in ADR-0059 and is not
  re-opened here.
- **Four mutations, each red for its own reason.** Dropping `fetch_stall` from `stall` fails
  `components_pcloop` (`pcloop.sv:379`, the new hold assertion) and `decoder_tb`; making the bubble
  arm win the coincidence fails `components_decoder` (`decoder.v:1199`) and `decoder_tb`; dropping
  `instr_fencei` from the serialization term fails `decoder_tb`; tying `mem_ren` low fails
  `accessor_tb` on all five load widths.

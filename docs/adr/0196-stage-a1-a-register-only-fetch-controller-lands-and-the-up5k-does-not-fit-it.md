# 0196 — Stage A1's register-only fetch controller lands, correct and proven, and the up5k does not fit it

Status: Accepted for the mechanism; the up5k area result is a named kill criterion, hit and
reported rather than absorbed. 2026-09-18.

## What this is

The first of four Stage A merges described in `docs/ideas/the-fetch-address-reads-registers.md`.
`rtl/fetchqueue.v` (a 4-word FIFO, `req_valid` contract) landed with no caller on `main`; this
change builds that caller, `rtl/fetchctrl.v`, and wires it into `rtl/littlecpu.v` in place of the
direct `next_pc → imem_addr_next` publish. `imem_addr_next` is now `fetch_pc`, a register updated
from registers only, unconditionally. Decode is otherwise unchanged: it still reads register
values and resolves branches same-cycle, still owns issue, and a stalled cycle still re-presents
the same word. No predictor and no kill exist yet — every redirect pays a full queue refill — and
that cost is measured and recorded here, not gated, exactly as the brief asks.

## The mechanism

`fetch_pc`'s own update, every cycle: a registered redirect target if decode issued a
redirect-shaped instruction (trap, `mret`, `jalr`, `jal`, a taken branch, or `fence.i`) two cycles
ago; else `+8` when `rtl/fetchqueue.v` reports room; else hold. `redirect` is a new one-bit decoder
output — `issuing && (trap_taken || instr_mret || instr_jalr || instr_jal || branch_taken ||
instr_fencei)` — computed from signals decode already has; nothing about decode's own timing
changes to produce it. `fence.i` is in that list because text is writable and the queue prefetches
ahead of decode, so a store retired just before `fence.i` can leave stale words already buffered;
without it `selfmod.S` reads a pre-patch instruction out of the queue.

`rtl/fetcher.v` is rewritten to window an instruction out of the queue's head pair (`q0`/`q1`)
instead of two ROM outputs, using the same `{q1,q0} >> (pc[1]?16:0)` shift it always used; only the
source of the pair changed. It emits `pop = next_pc[31:2] != pc[31:2]`, unconditionally correct
because `next_pc == pc` on any stalled cycle collapses it to zero with no extra gating.

Two bugs a prior spike (ADR-0188's decoupled-fetch section) found and did not close are closed
here, both from the same root cause: `rtl/imemory.v` answers whatever address `fetch_pc` presents
every cycle, with no handshake, so a cycle `fetch_pc` does not genuinely advance must not have its
answer queued.

- **A steal is confirmed a cycle after it happens, by which point a second, honest request has
  already gone out.** `fetch_stall` (imemory's read port stolen by a text-range load or store)
  arrives registered, one cycle behind the steal itself — by the time it is visible, `fetch_pc` has
  already advanced to the *next* address and presented it to imemory, unaware. Both that request's
  answer and the stolen one must be kept out of the queue, and `fetch_pc` must retry the stolen
  address once the second one is drained — not launch a third. `fetchctrl.v` tracks `stolen_pc`
  (the previous cycle's `fetch_pc`, updated unconditionally) and `fetch_stall_d1` (the steal flag
  delayed one more cycle), and gates the queue's `req_valid` on `!fetch_stall && !fetch_stall_d1`
  while retrying `fetch_pc <= stolen_pc` the cycle the steal is confirmed. The two premature
  responses are discarded and both addresses are re-requested in order.
- **A redirect settles over two cycles, not one, and both must discard.** Decode's `redirect`
  arrives during the issue cycle; `fetchctrl` captures it as `redirect_apply` one cycle later, and
  only then does `fetch_pc` take the target — a cycle later still than the address the *previous*
  cycle presented. Two stale requests are already outstanding by the time the target is visible to
  imemory: the one issued the redirect cycle and the one issued the settling cycle after it.
  `flush` (the queue's discard) is asserted for `redirect_apply || redirect_apply_d1`, a two-cycle
  window, not one.

`fetchqueue.v`'s own `req_valid` contract (`waiting`'s next-state, `fetch_stall ? 1 : launch`) was
already the fix landed ahead of this ticket for ADR-0188's second bug (a hand-rolled
address-change inference prone to exactly this class of off-by-one); this session did not need to
touch it.

## The assumed contract becomes a checked one, and it found a real bug immediately

`rtl/fetchqueue.v`'s FORMAL block states its caller contract as an `assume`
(`!req_valid || cnt <= DEPTH-2`); nothing composed with it formally before this ticket, so the
assume had never been checked from outside. `formal/pcloop.sv` now instantiates
`fetchctrl`+`fetchqueue`+`fetcher`+`decoder` the way `rtl/littlecpu.v` does, and
`formal/components.sby`'s `pcloop` task reads `fetchctrl.v`/`fetchqueue.v` with `-formal -noassume`
— the assume drops, the assertions inside stay as properties the composed proof has to prove, the
same pattern `traps.sv` already uses for `decoder.v`. The contract is asserted where it is *driven*
(`fetchctrl.v`'s own FORMAL block: `assert(!req_valid || queue_count <= 2)`, mirroring the assume
exactly, so a caller mismatch cannot hide behind two different spellings of the same rule).

Composing it found two bugs the whole session's cxxrtl run (75/75) never exercised, because cxxrtl
zero-initializes registers and neither ever happens to leave X-shaped garbage behind in simulation
the way a free BMC step-0 register value does:

- `rtl/fetchctrl.v`'s `stolen_pc` and `redirect_target_reg` had no reset value. A steal or a
  redirect settling on the very first few cycles after reset would retry or land on whatever these
  registers powered up holding. Fixed: both reset to zero alongside the rest of the state.
- `rtl/fetchqueue.v`'s own `assert(cnt <= DEPTH)` was not gated by `clocked` (an existing local
  flag every other assertion in the file already reads) and so was free to fail on step 0, before
  `cnt` has ever been reset, purely because BMC gives a free register no initial value. One line:
  `assert(cnt <= DEPTH)` → `if (clocked) assert(cnt <= DEPTH)`.

Both are area-neutral, formal-only fixes to code that predates this ticket by one commit
(`b9bce2c`) and had simply never been exercised by a composed proof.

## `formal/pcloop.sv`'s three properties

The old file's `assert(imem_addr == past_imem_addr_next)` encoded the old design's whole point —
the fetch port and the architectural pc were the same signal — and is simply false now that they
are decoupled by a queue; it is deleted, not weakened. Three new properties replace it, all proven
by k-induction (`make -C formal components_pcloop`, PASS):

1. **`fetch_pc` advances by 8, retries two cycles back, lands on a redirect target two cycles after
   decode computed it, or holds — never anything else.** A four-way disjunction
   (`f_fetch_pc_advanced || f_fetch_pc_held || f_fetch_pc_retried || f_fetch_pc_redirected`), guarded
   to start checking once two real cycles of history exist.
2. **The buffer's word and pc stay consistent**, restated as an independent check on `pop`'s own
   condition (`fetcher_pop == (next_pc[31:2] != pc[31:2])`) computed at pcloop's own level rather
   than trusting `rtl/fetcher.v`'s identical `assign` — an edit to either file that stops agreeing
   with the other now has a proof to catch it. An earlier draft of this property compared
   `fetch_pc[31:2]` against `pc[31:2]` directly ("fetch never falls behind decode"); it is true in
   every realistic trace but false under free 32-bit address wraparound (`mtvec` is a fully free
   CSR value in this harness, and BMC found a trace where a redirect target near `0xFFFFFFFF` makes
   `fetch_pc`'s own `+8` wrap past zero while `pc` sits at the top of the address space) — a
   harness artifact, not a design defect, and the property was replaced rather than patched with an
   unprincipled bound.
3. **A word that never reaches decode never issues.** Nothing discards a buffered word in this
   stage — there is no predictor and no kill — so this is trivially true by construction; it is
   written now so Stage A2, which adds a real kill, only has to strengthen it. Stated as
   `decoder_out.valid` (what issued *last* cycle, since `out` is registered) implying the buffer was
   not empty last cycle either — except across a divider hold, where `decoder_out.valid` reports a
   republished value carrying no opinion about the current buffer at all.

`formal/components.sby -> pcloop` is `mode prove`; PASS means proven for all time, not bounded.

## `formal/traps.sv`: minimum wiring, one real fix found doing it

`traps.sv` builds `decoder`+`csrs` (plus a free-input `fetcher`) without the core, per its own
header. `rtl/fetcher.v`'s port change means its `fetcher` instantiation now feeds `q0`/`q1` the
free `imem_data`/`imem_data2` inputs it already had — the same standing those signals always had,
under new names. `decoder.v` gained `buffer_empty` (wired as a new free input, the same standing
`fetch_stall`/`bus_wait`/`imem_fault` already have) and lost `fetch_stall` (removed, see below);
`redirect` is a new output, left unconnected.

One thing was not "minimum" and needed fixing: `hard_stall`, `traps.sv`'s own restatement of
"decode issues nothing," still read `fetch_stall` after the decoder no longer does — a stale
mirror of a dependency that moved. Left alone, this makes `traps.sv`'s harness weaker than the
design it is modeling (it would permit issuing on a cycle it should not) rather than merely
unaffected; fixed to `divider_stall || buffer_empty || bus_wait`, matching decode's actual gate.

`traps-region-probe.py`'s `wrong-cause` mutation surfaced a second instance of the same class of
staleness, in shipping `rtl/decoder.v` rather than in the formal harness: `ls_answer_valid`'s hold
condition (`ls_answer_valid <= ls_answer_valid && ls_access && (bus_wait || fetch_stall)`) is about
surviving every *other* reason this same latched load or store cannot yet issue — a job
`stall_own`'s own membership already names, and that membership moved from `fetch_stall` to
`buffer_empty` earlier in this same change. Left reading the old signal, the probe found (via
`components_traps`, which reads `decoder.v` with `-formal -noassume`) that the mutant's cause-swap
now surfaces as a **decoder-internal** assertion failure instead of `traps.sv`'s own — the mutation
probe's whole point, silently defeated, because `buffer_empty` and `fetch_stall` are independent
free inputs in this harness and the coupling `ls_answer_valid` relied on no longer holds. Fixed by
the same swap, `(bus_wait || buffer_empty)`; `fetch_stall` is now read nowhere in `rtl/decoder.v`
and the port is deleted. `make -C formal components_traps` (k-induction) and both its forced-red
prerequisites (`traps-region-probe.py`, `traps-tval-probe.py`) pass.

## The six-place stall-reason ritual

`fetch_stall` leaves decoder's `stall` OR, the publish arm, and the `read_rs1`/`read_rs2` guess mux
— all three now read `buffer_empty`. It stays as a decoder port nowhere; it is deleted entirely,
read now only by `rtl/fetchctrl.v` (to gate the queue's `req_valid`) and nowhere else. All six
declared sites — `rtl/decoder.v`'s signal/OR/publish arm/FORMAL asserts, `test/decoder_tb.v`'s
OR-identity check and both-ways vectors, `test/cxxrtl.cc`'s bucket, `test/stall_report.py`'s
`REASONS`/`HEADINGS`, `formal/pcloop.sv`'s `f_may_stall`, and this file's own commitment 8 — were
edited in this commit; `test/stall_sites_test.py`'s static `SIGNAL_TO_REASON` table was edited
alongside them and now reports all eight reasons agreeing across all six sites again.

## Verification, run in full

`make -C formal remeasure-fg`: **F 6 → 8, G 6 → 8** (first retire costs the buffer fill; the worst
retire gap grows with it). `[depth]` floors re-derived and bumped where the old ones fell below the
new F+2G/start+G: `insn`/`fault`/`ill` 19 → 24, `reg`'s second number 22 → 23; `csrw`/`pc_fwd`/
`pc_bwd`/`liveness`/`unique`/`causal`/`causal_mem`/`hang` were already clear of their new floors.
`formal/genchecks-audit.py`: 86 checks, all at or above their floor. Memcheck depths
(`check-memcheck-depth.py`, floored at F+2/F+G+2): `imemcheck` 15 ≥ 10, `dmemcheck` 20 ≥ 18, both
already clear with no edit.

Every proof and the generated checks were run against a **local clone of the pinned riscv-formal
SHA inside this worktree** (`c992aa61fdfe0846c5ed90324c596202a1c69b76`), not a symlink to the main
checkout's clone. A first attempt symlinked `formal/riscv-formal` at the main checkout the way
CLAUDE.md's own worktree note prescribes for files *inside* riscv-formal's own tree; the
`@basedir@/../../rtl/...` paths `checks.cfg`'s generated scripts use resolve `..` against the
symlink's real, physical location, which is outside this worktree, so they would have opened
main's unmodified `rtl/`, not this branch's. It surfaced immediately and loudly rather than
silently: `make -C formal remeasure-fg`'s first run errored with `rtl/fetchctrl.v` not found,
because that file does not exist on `main` at all — no generated-check result was ever produced
against the wrong tree. Re-cloned as a real directory at the pinned SHA
(`git clone --no-checkout .../formal/riscv-formal formal/riscv-formal && git checkout --detach
<sha>`) before any measurement; every number in this ADR is from that clone. `formal/components.sby`'s
tasks (`pcloop`, `traps`, `decoder`, `executor`, `accessor`, `busarbiter`) never depend on
riscv-formal's own basedir at all — their `[files]` lists are relative to `formal/` itself — so
they were never at risk.

**Every component proof passes by k-induction**: `components_decoder`, `components_executor`,
`components_accessor`, `components_pcloop`, `components_traps`, `components_busarbiter`.

**The cxxrtl suite is 75/75**, matching `test/EXPECTED_FAIL` (empty) exactly. `test/OBSERVED_FLOOR`
moved on one line, for the reason CLAUDE.md already gives `mtimer.S`'s and its own prior move of
this exact line: `uart.S`'s poll loop costs more cycles now that every redirect pays the queue
refill, so fewer loops complete in the fixed 5000-cycle window. `1381 1375` → `940 934`, both
figures the run's own observed values, not a margin. No other floor moved.

**`make test-units`**: all 14 unit benches pass, `fetchqueue_tb.v` included, unmodified.
**`make lint`**: clean, both passes (RVFI macros off and on). **`make elaborate-strict`**: clean,
no warnings beyond the standing `Deep recursion` allowlist entry. **`make testbench.vvp`** /
**`make waves`**: build and run clean (only the allowlisted `writeback.v`-class `sorry`s, confirmed
present identically on `main` before this change, so not a regression). **`make dual-build`**,
**`make dual-elaborate`**, **`make dual-smoke`**: all pass — two harts counted 32 total retires,
one hart alone counted 16, on the smoke program neither `bus_wait` nor the two-hart arbitration
interact with `fetchctrl` at all, since each hart's `imem_addr_next` is its own independent port
(`rtl/imemory.v`'s `NHARTS`-wide generate block, unchanged).

## Measured: cycles

Dhrystone (`make dhrystone`, `DHRY_RUNS=2000`, the Makefile default): **1001 cycles/Dhrystone,
0.568 DMIPS/MHz**, against this same tree's pre-change figure of 0.722 DMIPS/MHz (ADR-0190) — a
**27.1% increase in cycles per unit of work**. This is well beyond the stage's own eventual +3%
ceiling and is expected and recorded rather than gated, exactly as the brief states: no predictor
exists yet, so *every* redirect — 16.93% of Dhrystone's issues, per the hazard-column accounting
CLAUDE.md already carries — pays a full multi-cycle queue refill instead of the old same-cycle
publish. `make cycles`' own accounting for the suite (not Dhrystone) shows `region_stall`
essentially unaffected (the load/store side is untouched) and the `fetch`/buffer-empty column now
carrying the redirect-refill cost the old `fetch_stall` column never did. CoreMark (`make coremark`) did not finish inside this session's time budget on two attempts (a
120s and a 200s timeout); it built and ran cleanly up through the image-size report before being
terminated, with no error, and is left unmeasured rather than guessed at.

## Measured: area, and the up5k does not fit

`make fit` (the core alone): **4680 `ICESTORM_LC`**, against the tree's own `FIT_MAX_LC` of 4219 —
**+583 cells**, re-derived and re-ratcheted to 4802 (4680 + the standing 68-cell churn band +
54-cell toolchain gap) in this commit, quoting this local run per CLAUDE.md's own caveat that `fit`
is toolchain-dependent and a CI number, when one exists, is the one to quote instead.

**`make soc-timing` does not place.** nextpnr-ice40 fails during initial placement —
`ERROR: Failed to expand region (0, 0) |_> (25, 31) of 5521 ICESTORM_LCs` — meaning the SoC's own
demand is at least 5521 `ICESTORM_LC` against the part's 5280, before routing or timing are even
attempted. This is the brief's own named top risk, realized: "The placed SoC is ~4,900–4,950
`ICESTORM_LC` of 5,280, about 330 free" and "+100 to +400 LC" was the brief's own budget and
worst-plausible-case estimate for this exact change; the measured core-alone growth of 583 cells
already exceeds the worst-plausible SoC-level estimate, and the SoC's own shortfall (at least 241
cells past the part's total capacity, before any placement slack) is larger still. This is
precisely the kill criterion the ticket names: *"the SoC does not place under 5,280 at depth 4 (→
depth-2 skid, then the reserve)."*

**Neither named recovery is attempted in this commit.** The depth-2 skid (the brief's own estimate:
roughly −100 LC, "no output mux, shifting work toward D") is not enough on its own against a
measured shortfall several times that size, and redesigning the queue's depth now would reopen the
`fetchqueue.v`/`fetchctrl.v` interaction this session spent its formal effort proving correct,
under materially less time than is available to verify a second design honestly. Dropping
`rtl/spiflash.v` (the brief's named reserve, ~80–100 LC) is a real feature removal with test
consequences (`test/asm/spiflash.S`, `spioverlay.S`, `test/OBSERVED_FLOOR`, `SOC_EXPECT_*`) that
the brief itself treats as a deliberate, separate decision, not a quick patch. Both are named here
as the next step, not taken here.

**Paired timing sweeps, `soc/paired_sweep.sh`, and `make icesugar-*`/ECP5 measurements are not run
in this commit.** A design that does not place has no placement to sweep; the ECP5 side (16 KB of
free block RAM, no fabric-region failure expected there given the up5k's own shortfall is measured
in `ICESTORM_LC` and SPRAM/EBR counts unique to this part) is plausible but unmeasured, and is named
as a candidate next check rather than assumed.

## Consequences

- **The mechanism is correct and formally proven; the design as spelled does not fit its primary
  target part.** Both are true at once and neither excuses the other: shipping RTL that is provably
  right about pc timing but cannot be placed on the board it is meant for is not "done," and
  abandoning correct, verified work over an area number that has two named recovery paths already
  in the brief is not the right call either.
- **Two graders are owed and not run**: `make -C formal check` (the generated per-instruction
  checks, 86 of them, all rebuilt at F=8/G=8) and `make -C formal complete`/`complete_cover`/
  `cover`/`imemcheck`/`imemcheck_cover`/`dmemcheck`/`dmemcheck_cover` were started but this session's
  time ran out before every one of them finished; see the PR body for which, if any, completed and
  what they reported. `imemcheck.sv` in particular carries a real, unresolved question independent
  of whether it happened to finish: its oracle assumes `imem_addr` (the currently-decoded
  instruction's own address) and `imem_data` (whatever `fetch_pc` most recently requested) are the
  same cycle's value, which was true by construction in the old design and is not anymore now that
  a queue decouples them. Nothing in this session establishes whether that makes the check
  vacuous, wrong, or accidentally still sound; it is flagged rather than fixed.
- **CI's own `fit` job number, not this local run's, is the one to re-quote `FIT_MAX_LC` against**
  once available, per CLAUDE.md's own toolchain-dependence caveat.
- **Stage A2 and A3** (the predictor and the kill) are what this stage's own cycle cost is spent
  recovering, per the brief's sequence; neither is started here.

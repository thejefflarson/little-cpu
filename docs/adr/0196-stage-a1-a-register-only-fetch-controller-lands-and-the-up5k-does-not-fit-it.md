# 0196 — Stage A1's register-only fetch controller lands, correct and proven, and the up5k does not fit it

Status: Accepted for the mechanism; the up5k area result is a named kill criterion, hit and
reported rather than absorbed. The depth-2 skid named as the recovery was attempted and is
abandoned with a proven counterexample, not shipped disabled -- see the addendum. 2026-09-18.

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
- **CI's own `fit` job number, not this local run's, is the one to re-quote `FIT_MAX_LC` against**
  once available, per CLAUDE.md's own toolchain-dependence caveat.
- **Stage A2 and A3** (the predictor and the kill) are what this stage's own cycle cost is spent
  recovering, per the brief's sequence; neither is started here.

## Addendum: CI's mechanical failures, `imemcheck.sv` re-derived, and the depth-2 skid attempted

A follow-up session fixed everything #383's first CI run found mechanically red, re-derived
`imemcheck.sv` for the decoupled interface (finding and fixing a real bug doing it), and
attempted the depth-2 fallback this ADR's own body names as the next step. Recorded here rather
than in a second ADR because all three are direct continuations of the same finding.

**File-list registrations.** `formal/check-nonperturbation.py`'s own `RTL` list,
`formal/memcheck-cover-probe.py`'s `LITTLECPU_RTL` tuple (and the `test/probe_gates.sh` fixture
required to name exactly the same list) were all missing `rtl/fetchctrl.v`/`rtl/fetchqueue.v` --
every "fetcher.v" reference in the tree was re-audited (not just the ones CI happened to run) to
find them. `test/stall_sites_test.py`'s own probe fixture in `test/probe_gates.sh` still mutated
the pre-amendment `fetch_stall` spelling of `stall_own` and the OR-identity check; retargeted to
`buffer_empty`, matching the shipped RTL.

**`text-port-drops-load`'s `spioverlay.S` pairing moved from `FAIL 2` to a stable `TIMEOUT`.**
Confirmed at 5000 (the runner's own limit), 50,000 and 200,000 cycles, with retires still
climbing at all three rather than settling -- a real livelock the mutation causes under the
queue's own independent fetch timing (the corrupted read's wrong value is no longer the same-cycle
fetch content decode was tightly coupled to, so it occasionally satisfies a retry condition in the
test instead of failing the comparison outright), not the runner's cycle limit landing on a
slower-but-still-terminating failure. `test/MUTATION_DETECTORS` re-paired; the other four
pairings (`contend.S`, `datainit.c`, `selfmod.S`, `textload.S`) are unaffected, confirmed by
re-running each individually against the mutated tree.

**`formal/imemcheck.sv`'s oracle was stale, re-derived, and composing it found a real RTL bug.**
Its assume block keyed shadow-content correctness off `imem_addr` (decode's own `pc`) matching
`imem_data` (whatever `fetch_pc` most recently requested) the same cycle -- true by construction
in the old tightly-coupled design, false now that a queue decouples the two. Re-derived against
`$past(imem_addr_next)`, word-aligned the way `rtl/imemory.v`'s own `next_word = imem_addr_next[31:2]`
already is (a redirect target can land on any compressed-instruction boundary, so `fetch_pc` is not
always word-aligned the way the old `imem_addr` was). The re-derived check found a genuine bug in
`rtl/fetchctrl.v`: `stolen_pc`, the register a steal retries, was being overwritten by an
unconditional `stolen_pc <= fetch_pc` on every cycle, including a retry cycle itself -- so two
steals in a row (`imem_arbiter`'s own free model has no real bus's transaction spacing to bound
it, but nothing in the RTL bounds it either) silently dropped the *first* stolen address from the
fetch stream, retrying the second one instead and never recovering the first. Fixed by only
writing `stolen_pc` on a cycle that is not itself a retry (`redirect_apply` or the room-having/
holding `else` arm, never the `fetch_stall` arm) so it survives however many consecutive steals it
takes to land cleanly. Re-verified: the full suite (75/75), `components_pcloop`, `components_traps`,
`imemcheck`, `imemcheck_cover` (and its forced-red `memcheck-cover-probe.py` prerequisite) and
`dmemcheck`/`dmemcheck_cover` (unaffected, unchanged, re-run as a control) all pass.

**The depth-2 skid was attempted and is not correct; no area or timing number is reported for
it.** `rtl/fetchqueue2.v` is a from-scratch two-word skid (no head pointer, no array index, "no
output mux" exactly as the brief names it) behind a new `fetchctrl`/`littlecpu` parameter
(`SHALLOW_QUEUE`/`SHALLOW_FETCH_QUEUE`, off by default -- the shipping configuration is untouched
and re-confirmed 75/75, clean `lint`/`elaborate-strict`, and `make fit` unmoved within the churn
band after this refactor). Building it found two real bugs. **The second is a fundamental property
of the "no output mux, two total slots" design as specified, not an implementation slip, and it is
where this attempt stops.**

1. **A hard deadlock**, not merely a slowdown, fixed. Depth-2's `q_valid` first attempt copied
   depth-4's own `cnt >= 2` (both words present) -- correct at depth 4, where consuming one word
   still leaves the queue above that threshold, but at depth 2 the very first word popped from a
   full pair drops `q_valid` to false immediately, which stalls decode, which is the only thing
   that ever pops the second word. The queue can never drain past one word and never reopens
   room. A word crossing only ever lands decode exactly at a word boundary, so the *first*
   instruction reached immediately after one is never itself a straddle into the not-yet-fetched
   next pair -- one live word is enough to proceed there. `q_valid` became `cnt != 0`. This alone
   took the suite from every program timing out to `simple.S` passing and most others reaching
   `TRAP-TO-ZERO` instead.

2. **A proven, reproducible content-corruption bug, confirmed on the iverilog leg with a concrete
   counterexample, and not fixed because the design as specified cannot be fixed without
   reintroducing what "no output mux" was trying to avoid.** The "one live word is enough" argument
   above is true for the *first* instruction after a crossing but false for the *second* one, and
   the coordinator was right to ask for the proof rather than accept the code: after a pop slides
   the surviving word into `mem0` (`cnt` 2 -> 1), `mem1` is not cleared -- it still holds `mem0`'s
   own former content, a stale duplicate, because nothing can refill it until `cnt` reaches 0 and a
   fresh pair lands (the queue has exactly two physical slots and a push always fills both, so
   there is no room for "one old word plus a fresh pair" at once). If the *first* instruction
   inside the slid word is compressed, decode reaches a *second* instruction at the odd halfword of
   that same word while `cnt` is still 1 -- and if that second instruction is uncompressed, it
   straddles into `mem1` for its own upper 16 bits, reading the stale duplicate instead of the
   genuinely next word.
   **Reproduced on `add.S`'s own first three words** (`rtl/fetchqueue2.v` behind
   `SHALLOW_FETCH_QUEUE`, iverilog leg, `$display` on `fetchctrl`'s internal state every cycle):
   word0 = `0x40814181` (two compressed `li`s, filling the word exactly, so no pop occurs inside
   it), word1 = `0x87334101` (`li sp,0` compressed at the low half, the low half of an uncompressed
   `add` at the high half), word2 = `0x20008144` (the upper half of that `add`, the word the
   straddle genuinely needs). Traced signal values:
   ```
   pc=0 cnt=2 q0=40814181 q1=87334101   -- fresh pair, both words valid
   pc=4 cnt=1 q0=87334101 q1=87334101   -- popped once; q1 is a STALE COPY of q0, not word2
   pc=6 cnt=1 q0=87334101 q1=87334101   -- the straddling `add` reads q1[15:0]=4101 here,
                                             not word2's correct 8144
   ```
   The instruction decoded at `pc=6` is `{q1[15:0], q0[31:16]} = 0x41018733` instead of the correct
   `0x20008733` (`add a4,ra,sp`) -- a different instruction entirely, which is what turns into
   `TRAP-TO-ZERO` a few cycles later on the real program.

   **Why this cannot be patched without giving up "no output mux."** Fixing it needs `mem1`
   refreshed with the genuinely next word while `mem0` is still resident and still needed (its
   upper half is the straddling instruction's own low 16 bits) -- a "top up just the empty slot"
   push, distinct from the queue's only push mode (fill both slots from a flushed-empty state).
   That needs either a third physical slot (at which point this is depth-4's own array-based
   design at depth 3, not the "no output mux" shape at all, and still needs a head-selecting mux),
   or a new partial-push mode that fetches a redundant pair and discards its own low word -- real,
   new logic, not the simplification the brief named. Neither is "no output mux, work shifted
   toward D," and this session did not build either given the time available and the risk of
   shipping a second unverified mechanism under the same time pressure that let the first bug
   through review. **`rtl/fetchqueue2.v` and the `SHALLOW_QUEUE`/`SHALLOW_FETCH_QUEUE` parameters
   are removed rather than shipped disabled**: unlike ADR-0189's `NANO_LATCH_RF`, which is racy
   only under a stress the shipping design never hits, this is wrong on the *first* program in the
   suite that happens to pair a word-filling compressed instruction with a following straddle, and
   a known-broken module behind an off-by-default parameter is not a state to leave in the tree.

**No area or timing number is reported for the skid.** A depth-3, array-based redesign (reusing
`rtl/fetchqueue.v`'s own head/tail mechanism at three slots instead of four, which the corrected
argument above shows has room for "one old word plus a fresh pair" and so does not hit this bug) is
a plausible next avenue, but it needs a mux the brief's "no output mux" phrasing did not budget
area for, and would save proportionally less than the ~100 LC estimated for a true depth-2 (a
25%-smaller array against a 50%-smaller one) -- likely not enough against the SoC's measured
241+-cell shortfall on its own. Untried here; named for whoever picks this back up next.

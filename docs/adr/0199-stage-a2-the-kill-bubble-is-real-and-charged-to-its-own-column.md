# 0199 — Stage A2: `kill` is a real, proved signal, charged to its own column

Status: Accepted. 2026-09-18.

## What this is

The second of four Stage A merges described in `docs/ideas/the-fetch-address-reads-registers.md`,
built on Stage A1 (`rtl/fetchctrl.v`/`rtl/fetchqueue.v`, ADR-0196) rather than on `main` — A1 still
does not fit the up5k, so this stacks on its branch per the owner's direction, and `soc-timing`
fails here for the reason ADR-0196 already names, not for anything this ADR does.

## What A1 already had

A1's own discard mechanism is correct and already proven: `rtl/fetchctrl.v`'s `flush` — asserted
for `redirect_apply || redirect_apply_d1`, a two-cycle window — discards both ROM responses already
in flight when a redirect fires, and `rtl/fetchqueue.v`'s `flush` input zeroes the queue on the
same cycle, dropping a same-cycle genuine response too rather than racing it. `formal/pcloop.sv`'s
three fetch-controller properties, `rtl/fetchctrl.v`'s own `FORMAL` block and `test/fetchqueue_tb.v`
already prove this discard correct — that work is not repeated here. `fence.i` already flushes the
buffer: `redirect`'s own list already includes `instr_fencei`, precisely because text is writable
and the queue prefetches ahead of decode, per ADR-0196's own text.

**`fencei-wait-and-store-port`'s `asm selfmod.S FAIL 2` pairing was re-measured, not reconfirmed,
and it no longer fires.** Applying the mutation to a clean checkout of A1's own tip (11a5ec7, before
any change in this ticket) and running `selfmod.S` against it: `PASS retires=47 spec-checked=44` —
the program no longer catches the mutation, on A1's branch alone, unrelated to anything built here.
The reason is exactly the mechanism this ticket's own `kill` sits on top of: the mutation drops
`instr_fencei` from `serialize` and drops a store from `imemory.v`'s `text_access`, which is the
pair of safeguards the OLD, queue-free design needed. A1's redirect-triggered queue flush is a THIRD,
independent safeguard that did not exist when this pairing was written, and it alone is now
sufficient to give `selfmod.S` the fresh text it checks for, regardless of whether `serialize` or
`text_access` still do their old job. `bench decoder_tb` and `bench imem_tb`, which test `serialize`
and `text_access` directly rather than through a full program, still catch the mutation — confirmed
by `make mutation-check` itself, whose set-equality grading names only `selfmod.S` as silent, never
the two benches. Per `docs/manifests/mutation-detectors.md`'s own instruction ("measure it, do not
assert it" — "a mutation nothing catches does not belong here... it is a fact about the machine to
record"), the `selfmod.S` line is removed from `fencei-wait-and-store-port`'s pairing; the two bench
lines are unchanged. **`text-port-drops-load`'s full five-detector pairing (`contend.S`, `datainit.c`,
`selfmod.S`, `spioverlay.S TIMEOUT`, `textload.S`, `imem_tb`) is unaffected and reconfirmed as-is** —
that mutation drops only the store-port-steal, not `serialize`, and the flush alone does not
substitute for it.

**What was missing was not a mechanism — it was a name and a column.** Every cycle A1's discard
spent was charged to `buffer_empty`, the generic "the queue has fewer than two words" stall reason,
indistinguishable in the accounting from a cold start or a run of steals. Nothing separated "the
queue is empty because we are refilling after a redirect" from "the queue is empty for some other
reason," and nothing proved that a word caught mid-discard could not still issue — `formal/pcloop.sv`
said as much in Property 3's own comment: "trivially true here since nothing kills yet."

## The mechanism this ticket adds

One new signal, `rtl/fetchctrl.v`'s `redirect_recovering`: a register set the cycle decode's own
`redirect` fires and cleared the cycle `buffer_empty` itself first reads false again. Because
`redirect` requires `issuing`, which requires `!stall`, which is already false throughout the
window `redirect_recovering` is set, nothing can raise a second `redirect` while the first is still
being recovered from — the register never needs to arbitrate between two in-flight discards.

**The clear condition is off `buffer_empty`, not off `q_valid` directly, and the first draft had
that wrong.** `q_valid` can still read true for one cycle after the redirect fires: the flush that
zeroes the queue lands on the *next* edge, not this one, so a same-cycle read of `q_valid` sees the
pre-flush count. Clearing `redirect_recovering` off `q_valid` let it drop one cycle early — the
cycle the flush is still discarding a genuine response — and every cycle after that fell through to
`stall_sites`'s ordinary bucket chain (mostly `operand`, since a redirect also invalidates the
guessed register pair `operand_stall` tracks) instead of `kill`. Measured on Dhrystone before the
fix: `kill=188771`, 1.16 cycles per redirect against 162,772 redirects — implausibly low next to
ADR-0188's own 1-cycle prototype and this stage's 2-cycle discard alone. Re-measured after
correcting the clear condition to `buffer_empty`: `kill=488313`, **3.00 cycles per redirect**,
`operand` falling from 429,540 to 174,217 and `fetch` from 61,531 to 17,321 — the two buckets that
absorbed the misclassified cycles, moved because they sit earlier in the eight-reason priority
chain a `kill` cycle no longer reaches. No cycle count changed (`cycles=2052685`, `issue=945272`,
`RETIRES 945270`, identical both ways) and no formal property depended on the exact duration, so
this was caught by re-deriving the number the ticket asks for, not by a red check.

`rtl/decoder.v` receives it as a new input and computes `kill` locally:

```
assign kill = buffer_empty && redirect_recovering;
```

The AND is the whole mechanism, and it is deliberately drawn inside decoder.v rather than trusted
from fetchctrl: it is what makes `kill => !issuing` provable from decoder's own structure alone,
with no assumption about what `redirect_recovering` means, in any harness — `stall_own` already
ORs in raw `buffer_empty` unconditionally, so `kill`'s truth already implies `stall`'s, which already
implies `!issuing`. `rtl/decoder.v`'s `FORMAL` block asserts exactly that
(`assert(!kill || !issuing);`), and it holds in `components_decoder` (where `redirect_recovering` is
completely free), in `formal/traps.sv`'s own harness (same standing as `buffer_empty` there always
had), and in the composed `formal/pcloop.sv` proof, where Property 3 is restated one cycle later
against the real queue's own `kill` output instead of `buffer_empty` — no longer trivial, since a
future stage's real predictor is exactly what could let a word survive past a kill if this property
did not hold. `formal/decoder-kill-probe.py` is the forced-red prerequisite: dropping the
`buffer_empty` conjunct from `kill`'s own `assign` makes `components_decoder` find a counterexample
at that exact line, run automatically ahead of the real proof.

**No control signal changed.** `stall`, `stall_own`, `stall_other`, the publish block's bubble
condition, `next_pc`'s case and `bus_request` all still read raw `buffer_empty`, unchanged from A1 —
a kill cycle bubbles the pipe for exactly the reason any other empty-buffer cycle does. `kill` is
read nowhere except by the accounting layer and the two proofs above; it decides no gate.

## The accounting split

`test/cxxrtl.cc` now reads `uut decoder kill` before consulting `uut decoder stall`'s bucket list: a
killed cycle is charged to a new `kill_cycles` counter and none of the eight stall buckets, even
though `stall` is also true on it (kill implies buffer_empty implies stall, never the reverse).
`test/stall_report.py` gains a `KILL` column, printed beside `ISSUE` per the brief's own wording,
and its identity becomes `issue + kill + unattributed + the eight reasons = cycles`. The `fetch`
column's own meaning narrows with it: it now counts an empty buffer that is *not* a redirect's own
discard-and-refill — a cold start, or a run of steals delaying the first fill — which on the shipping
suite and Dhrystone is a small residual next to what moved to `kill`.

`test/stall_sites_test.py` needed **no change to its OR-identity checks**: `kill` was never a
member of `stall`'s composition, so the six-place ritual's existing tables are already correct.
What the script already carried — unused until this ticket — was the guard against the OTHER
direction: `unknown_and_missing`'s message has named "the future non-stall kill bubble" and its
exile from every OR since before this ticket started, and `test/probe_gates.sh`'s
`"a future kill wrongly ORed into the OR-identity is red, and named"` probe (added in A1) already
forces that guard red on a mutated `dut.kill` term. Both were exercised, unmodified, against the
real `kill` port added here for the first time.

## `soc/fetch_ahead/` is retired

That directory's own spike (ADR-0188) patched `test/cxxrtl.cc` at runtime, at named anchors, to add
exactly this `kill`/`redirect` accounting to a tree its own patches built out-of-tree — the shape
this ADR's own `kill`/`KILL` columns are modeled on. With a real `kill` signal now shipping in
`rtl/decoder.v` itself, that reproduction has nothing left to reproduce: its patches target
`rtl/decoder.v` lines A1 already rewrote wholesale (the merged fetch-and-decode `next_pc` case the
patches assume does not exist on this branch), it is off `make test` and CI, and CLAUDE.md already
calls it "a spike with no gate," the same standing as `soc/depth/`. `soc/fetch_ahead/` is deleted —
`apply.sh`, `apply-decoupled.sh`, `cycles.py`, `sweep.sh`, both `.patch` files, and its
`.gitignore` line. `docs/adr/0188-*.md` and `docs/ideas/the-fetch-address-reads-registers.md` still
name the retired path; both are left as written, since an ADR and a brief are measurements and
plans with a date on them, not living documentation of what the tree currently holds.

## Verification

`make -C formal remeasure-fg`: **F = 8, G = 8, both reproduce exactly** — this ticket adds no
stage, no scoreboard slot and no new stall reason; `kill` is a relabeling of cycles A1's own F/G
sweep already counted as `buffer_empty`.

`make -C formal all`: every target PASS. `genchecks-audit.py`: 86 checks generated, `[depth]`
floors F=8/G=8, all 86 at or above theirs, `EXPECTED_CHECKS` matches exactly. The generated suite
itself: **86 checks, 86 pass, 0 fail**. `dmemcheck`/`imemcheck` and their cover controls: PASS.
Every component proof passes by k-induction: `components_decoder` (with its two Zkt probes and the
new `decoder-kill-probe`), `components_executor`, `components_accessor`, **`components_pcloop`**
(with `pcloop_cover`, and carrying the corrected `redirect_recovering` clear condition — this run
elaborated `rtl/fetchctrl.v` after the fix landed), `components_traps` (with both its region and
tval probes), `components_busarbiter`.

The cxxrtl suite is 75/75, matching `test/EXPECTED_FAIL` (empty) exactly. **`test/OBSERVED_FLOOR`
moved on no line**: `kill` is an accounting relabeling, not a cycle-count change, so every program's
retire-count floor is exactly what A1 left it at.

`make test-units`: all 14 unit benches pass, `decoder_tb.v`'s new both-ways `kill` vectors and its
new `kill && issuing` runtime check included. `make lint`: clean, both passes. `make
elaborate-strict`: clean. `make probe-gates`: every graded comparison forces red for its own reason,
`formal/decoder-kill-probe.py`'s own eleven cases (mirroring `decoder-zkt-probe.py`'s shape at one
case) included. `make mutation-check`: 11 mutations, 25 pairings (26 before the `selfmod.S` line
above came out), every one caught by exactly its detectors. `make cosim-suite`: matches
`test/COSIM_EXPECTED_FAIL`. `make dual-smoke`: passes, two harts, one text storage.

**Dhrystone**, `make dhrystone` (`DHRY_RUNS=2000`): **1001 cycles/Dhrystone, 0.568 DMIPS/MHz,
identical to A1's own figure to the cycle** — `kill` reclassifies cycles already spent, so a
benchmark that measures cycles alone cannot see the difference. `RETIRES 945270`, `cycles=2052685`,
`issue=945272`, both runs. **`kill=488313`, `redirect=162772` (a `--stalls`-only instrumented
build, not shipped): 3.00 cycles per redirect**, the number this stage owes Stage A3's predictor —
see the fix above for why the first draft read 1.16. `make fit`: **4697 `ICESTORM_LC`**, +17 against
A1's own 4680 (`FIT_MAX_LC` 4802 untripped) — inside the churn band, `kill` costs one AND gate and
one register. `make soc-timing` still fails to place, for
A1's own named reason (the SoC's own demand exceeds the part before this ticket's own two-gate
addition could move it either way); this is the expected, reported failure the ticket names, not a
regression this ticket owns fixing. **`make ecp5-timing` was run, and it places**: the up5k's own
shortfall is `ICESTORM_LC`-specific (the part is out of logic cells), and ECP5's 56 `DP16KD` block
RAMs give this design headroom the up5k's 30 do not — synthesis censuses clean (`DP16KD` 36,
`TRELLIS_DPR16X4` 32, `MULT18X18D` 4, all as declared, block RAM resets clean), placement and
routing succeed at the declared 200 MHz constraint. **Fmax: 43.33 MHz**, one placement, published
with no ratchet per this instrument's own standing — not compared against A1's own ECP5 figure,
since A1 never took one (its own SoC does not place on either part it measured, and ECP5 was left
"plausible but unmeasured" rather than assumed). `soc/paired_sweep.sh`'s twelve-seed sweep is not
run here: one placement is a sample, and the up5k side of that sweep still has nothing to place
against, so a paired comparison is not yet possible on this branch.

## Three mechanical gaps found in A1's own branch, fixed here

None of these are about `kill`; each blocked `make test` from completing at all and is fixed as a
mechanical CI gap, the same standing ADR-0196's own addendum gives its equivalent fixes.

- **`test/MUTATION_COVERAGE` had no ruling for `rtl/fetchctrl.v`.** A1 added the file but never
  added its line; `test/mutation_coverage_test.sh` catches exactly this. Ruled `unpaired
  components_pcloop`, matching `rtl/fetcher.v`'s own line — both are exercised only through the
  composed proof, no standalone bench.
- **`test/probe_gates.sh`'s `check-memcheck-depth.py` fixture still built its own synthetic
  `checks.cfg` against F=6/G=6**, the value before A1's own remeasure to 8/8; its `fixture_anchor`
  tripwire against the real `formal/checks.cfg` caught the drift. Every depth and message in that
  fixture is re-derived from F=8/G=8 (floor 18 for a two-retire property, 10 for one-retire).
- **`test/probe_gates.sh`'s two `formal/remeasure-fg.py` probes assumed the same stale F=6.** One
  expected the sweep's first row to be cycle 3 (`F - BELOW` at F=6); it is 5 now. The other narrowed
  `BELOW/ABOVE` to make F's own sweep unable to bracket a flip point, using a stub threshold (8)
  chosen to sit just past the OLD narrow window's upper bound; bumped to 10 to sit past the new one
  (F=8, `ABOVE=1` ⇒ upper bound 9).

## Consequences

- **`kill` is now load-bearing where it was decorative**: `formal/pcloop.sv`'s Property 3 is a real
  property of the composed queue's own signal, not a comment explaining why nothing needed proving
  yet.
- **The redirect cost A1 measured (27.1% more Dhrystone cycles) is now visible in its own column,
  at its real size**: 3.00 cycles per redirect, not the 1.16 an off-by-one in the clear condition
  would have shipped — Stage A3's predictor is measured against whichever of the two ships, so the
  fix is the difference between a real target and a understated one.
- **ECP5 places at 43.33 MHz where the up5k does not fit at all** — the two parts' shortfalls are
  different resources (`ICESTORM_LC` against block RAM headroom), and this is the first ECP5 figure
  taken anywhere on the Stage A line; A1 left it "plausible but unmeasured."
- **Stage A3 and A4** are unstarted here: the predictor that would actually spend `kill`'s cost, and
  `formal/traps.sv`'s own rewiring for a different stage, are separate branches off A1.

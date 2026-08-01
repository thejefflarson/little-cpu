# ADR-0040: The ladder refuses a negedge regfile rather than mis-modelling one, and `make check` had been re-grading the previous run

**Status:** Accepted · 2026-07-31 · *Supplements ADR-0004, ADR-0017, ADR-0022, ADR-0025, ADR-0031,
ADR-0033, ADR-0036, ADR-0038; records data for a future regfile ADR without pre-deciding it*

## Context

A design brief proposed a negedge-read regfile as the escape hatch if ADR-0004's flip-flop regfile
blocks timing closure on the up5k (it is on `docs/adr/README.md`'s deferred list under that name).
The brief assumed such a regfile would force `clk2fflogic` on the riscv-formal ladder and risk a slow
or inconclusive `reg_ch0`.

A follow-up investigation reported something worse. Driving yosys directly — `prep -flatten -nordff`
(how every generated `.sby`'s `[script]` section ends) followed straight by `write_btor` —
a negedge `$dff` is accepted **with no warning**, and in the emitted btor2 `clk`
is declared an input and never used while the read register and the storage array advance on the
same step. The half-cycle relationship is silently discarded. If that survived into the ladder, the
ladder would go green in the same wall time against a circuit that is not the RTL:
`docs/THREAT_MODEL.md` category 1, false assurance, and strictly worse than the inconclusive result
the brief worried about.

This ADR is the measured answer. It changes no RTL, and it does **not** decide whether to build a
negedge regfile — that decision is still deferred, and this is the data it should be made with.

Everything below was measured on one 10-core machine at `317b86c` — whose `rtl/` is byte-identical
to `f98f92f`'s — with `btor btormc` (ADR-0024) under `RISCV_FORMAL_ALTOPS` (ADR-0010). Full-ladder
runs used `-j10`; single-check runs were serial. The negedge and posedge-read regfiles are scratch
experiments — the spike's instrument, not its deliverable — and are not in the tree.

## Finding 1: the polarity loss is real, and the ladder never takes that path

The reported btor2 shape reproduces exactly. Against the negedge regfile **alone**:

```
$ yosys -p "read_verilog -sv regfile_negedge.v; prep -flatten -nordff -top regfile; write_btor iso.btor"
$ grep -n "clk" iso.btor
3:2 input 1 clk ; regfile_negedge.v:10.23-10.26
```

Node 2 appears nowhere else in the file as an operand — its only other occurrence is
`43 sort bitvec 2`, a width argument. `clk` is declared and never used, and `next` for the read
register and `next` for the storage array sit on the same step. Confirmed.

**But sby does not hand the `[script]` section's output straight to `write_btor`.** It runs its own
`prep` model step first (`sby_core.py`'s `make_model`), and with `multiclock` off — the default,
and not settable from `checks.cfg` at all: genchecks' `[options]` parser ends in
`else: print(line); assert 0`, so `multiclock on` there is an `AssertionError`, and nothing but
`mode`/`expect`/`append`/`depth`/`skip` is ever emitted into a generated `.sby`'s `[options]` —
that step is:

```
async2sync
chformal -assume -early
opt_clean
formalff -setundef -clk2ff -ff2anyinit -hierarchy
```

`formalff -clk2ff` is the guard, and on the full `rvfi_testbench` it **fails closed**:

```
prep: ERROR: CLK clock on $flatten\wrapper.\dut.\regfile.$procdff$2308 ($dff) from module
             rvfi_testbench also used with opposite polarity, run clk2fflogic instead.
prep: finished (returncode=1)
DONE (ERROR, rc=16)
```

`reg_ch0` dies in 1.0s with no status of `PASS`, and `formal/check-baseline.sh` counts anything that
is not `PASS` as a failure. **A negedge regfile is a hard stop on the generated ladder, with the
remedy named in the message. It is not a silent green.** The reported green was measured on the
regfile in isolation; the extrapolation to the full testbench does not hold, and the difference is
one yosys pass that sby inserts and a bespoke script does not.

That last clause is the part worth carrying forward. **The hazard is real wherever this repo drives
yosys itself rather than through sby.** `formal/equiv.sh` does exactly that, and any standalone
regfile equivalence proof written as a yosys script would too. In those places a negedge `$dff` will
be modelled as a posedge one, silently, and nothing will say so.

## Finding 2: `clk2fflogic` is what produces the false green

The remedy the error message names makes the ladder pass — vacuously.

Adding `clk2fflogic` through `checks.cfg`'s `[script-link]` hook (the seam
`genchecks-local.py:449,724` provides, and the only one available given ADR-0031 forbids patching
that vendored copy) and leaving `[depth]` alone, `reg_ch0` **passes on a regfile that is provably
wrong**. Two independent broken designs, both green:

- the negedge model changed to `posedge` on the read — one character, and it makes every read a full
  cycle stale, the exact defect ADR-0004 was written to fix. Without `clk2fflogic` this is a
  counterexample at k=20 in 3.3s. With it: **PASS in 4.3s**.
- `rtl/regfile.v` with the rs2 write-through bypass deleted — a one-line, direct violation of
  invariant 6. Without `clk2fflogic`, a counterexample at k=20 in 13.9s. With it: **PASS in 5.4s**.

Doubling `reg`'s depth line to `reg 30 40` does not rescue it: the broken posedge-read regfile still
**PASSES**, now in 67.2s. That is the shape of a check that has stopped asking the question, not one
that is looking harder.

## Finding 3: the vacuity is structural, and `checks.cfg` cannot express the fix

`clk2fflogic` makes one clock cycle **two** BMC steps. riscv-formal's depths are in clock cycles;
sby's `depth`/`skip` are in BMC steps. genchecks ties them together rigidly — both `check_insn` and
`check_cons` derive, from the single `[depth]` column, all three of:

```
skip  = <column>          depth = <column> + 1          `define RISCV_FORMAL_CHECK_CYCLE <column>
```

Every generated `.sby` on this ladder has `skip == CHECK_CYCLE` and `depth == CHECK_CYCLE + 1`.
Under `clk2fflogic` the check cycle lands at BMC step `2·CHECK_CYCLE + 1`, so the assertion is never
reached inside the horizon and every check is vacuous. Measured directly: holding
`CHECK_CYCLE 20` and widening **only** sby's horizon by hand to `depth 43 / skip 40`, the broken
posedge-read regfile produces its counterexample at exactly **k = 41**, and the deleted-rs2-bypass
probe produces one at **k = 41** on bad-state property 1 (the rs2 assertion — the right one).

So the horizon has to be decoupled from the check cycle by a factor of two. **`checks.cfg` has no
way to say that.** Raising the `[depth]` column raises both numbers together, which is why
`reg 30 40` above stayed green. Getting it right means either forking `genchecks-local.py` — which
ADR-0031 forbids and `make -C formal genchecks-check` enforces — or post-processing all 82 generated
`.sby` files with a script that reads each `CHECK_CYCLE` and rewrites each `depth`/`skip`. That
script would be a new, unenforced, load-bearing piece of the oracle, and a `.sby` it silently missed
would be a check that passes without checking. That is the same class of defect as Finding 4.

The cost, on top of that. Two clean serial single-check comparisons, same machine, same check, only
the model and the horizon differing:

| check, regfile | stock ladder | `clk2fflogic` + corrected horizon | ratio |
|---|---|---|---|
| `reg_ch0`, rs2 bypass deleted | 13.9s → counterexample | 186.0s → counterexample | 13× |
| `reg_ch0`, posedge-read model | 3.3s → counterexample | 47.0s → counterexample | 14× |

Extrapolating 13–14× onto the stock ladder's 310s puts a correctly-configured `clk2fflogic` ladder
somewhere around an hour. The full-ladder run attempted here is consistent with that but is **not** a
clean measurement and should not be quoted as one: it had completed 4 of 82 checks after roughly 20
minutes at `-j10`, on a machine whose load average was above 30 from concurrent work, and it was
stopped rather than finished. The serial per-check ratios above are the defensible numbers.

## Finding 4: `make -C formal check` could not re-run the ladder

Independent of the above, and the reason a spike whose entire output is before/after comparisons had
to fix it first. Two defects compounding, both confirmed by measurement:

1. `checks` was a plain file target, and it is a **directory**. A directory's mtime bumps whenever
   an entry is created inside it, and `sby` creates `checks/<name>/` for every check it runs. After
   one ladder run, `checks/` is newer than `checks.cfg`, `genchecks-local.py` and `EXPECTED_CHECKS`
   all at once, so make reports it up to date forever. **Editing `checks.cfg` between two runs did
   not regenerate the ladder.**
2. `genchecks-local.py:72` is `sbycmd = "sby"` with no `-f`, and the makefile it emits hardcodes that
   string. sby refuses to overwrite an existing workdir, so every re-invocation aborted and left the
   prior `status` file untouched. `make -BC checks`, which `check` used to pass, forces the **rule**,
   not sby — it made the aborts louder without making anything re-run.

Measured at `317b86c`, two `make -C formal check` invocations from the same tree:

| | wall | status files written | reported |
|---|---|---|---|
| run 1 (fresh `checks/`) | 310.4s | 82 | 82 checks: 82 pass, 0 fail |
| run 2 (immediately after) | 22.0s | **0** | 82 checks: 82 pass, 0 fail |

Run 2 printed `ERROR: Directory '<name>' already exists` for all 82 and every status file's mtime
was byte-identical to run 1's. **It reported a verdict it had not computed** — about RTL that, in
the general case, has changed underneath it since. `docs/THREAT_MODEL.md` category 1, and the
highest-value class of finding that document names.

With the fix, two consecutive `make -C formal check` invocations from the same tree:

| | wall | status files written | `already exists` lines | reported |
|---|---|---|---|---|
| run A | 322s | 82 | 0 | 82 checks: 82 pass, 0 fail |
| run B | 428s | 82 | 0 | 82 checks: 82 pass, 0 fail |

**Status files sharing an mtime between A and B: 0.** Every check re-executed; both runs match
`EXPECTED_CHECKS` and `EXPECTED_FAIL` in both directions and exit 0. (The 322s/428s spread is this
machine, not the change — see Consequences.)

## Decision

**1. `clk2fflogic` is not adopted, and the ladder keeps a posedge regfile.** Go/no-go on the two
options the spike was asked to choose between:

- **(a) `clk2fflogic` + re-derived depths — NO.** It is not reachable from `checks.cfg` (Finding 3),
  the only route to it is a genchecks fork ADR-0031 forbids or a new unenforced `.sby` rewriter, and
  its default-configured behaviour is a silent false green on designs the stock ladder catches in
  seconds. Adopting it would *introduce* the failure mode this spike was chartered to rule out.
- **(b) the fallback — YES, if a negedge regfile is ever built.** A standalone regfile equivalence
  proof, with the ladder wrapper continuing to see a posedge model.

**2. If (b) is built, the substitution lives at one named seam in `formal/wrapper.v` and nowhere
else.** Not an `ifdef` inside `rtl/regfile.v`, and not a swapped `read_verilog` line in
`checks.cfg`'s `[script-sources]` — the second is invisible from the RTL and is drift by
construction, arriving through the formal harness instead of through an `ifdef`, which is precisely
what the brief rejects. The seam is: `formal/wrapper.v` instantiates the posedge regfile explicitly,
by name, with a comment stating that the ladder is proving the core against a posedge model and
naming the equivalence proof that discharges the difference. A reader of `wrapper.v` must be able to
see that a substitution happened without reading `checks.cfg`.

**3. The equivalence proof must state its read-timing assumption explicitly, per ADR-0017.** An
`assume` is a promise made to the solver on the reader's behalf, and the promise here is the load-
bearing one: that `rs1`/`rs2` are stable from before the falling edge to the rising edge that
samples the result. Nothing in the ladder checks that — it is a timing-closure property, not a
functional one — so it must be named where it is made, together with what does check it. And per
Finding 1, that proof must not be written as a bespoke yosys script that ends in `write_btor`,
because that is the one context in which the polarity it exists to reason about disappears without
a diagnostic.

**4. `formal/Makefile`'s `checks` target becomes `.PHONY` and deletes `checks/` before
regenerating.** `-B` comes off the `check` sub-make in the same change: with no status file on
disk every rule fires on its own, and keeping a flag that forces rules rather than reruns is what
made this look handled for as long as it did. `genchecks-local.py` is not patched to add `-f`
(ADR-0031). Regeneration costs about a second. Re-grading a finished ladder without re-running it is
still available and is what `make -C formal check-baseline` is for (ADR-0022).

## What this spike establishes about `reg_ch0`

The spike needed a mutation `reg_ch0` demonstrably catches, because without one every timing number
above is uninterpretable — a fast green and a vacuous green look identical. The investigation that
prompted this ticket reported that deleting the rs2 write-through bypass **passes** `reg_ch0` on the
shipping design, which would have meant `reg_ch0` does not cover invariant 6.

**That does not reproduce.** Measured against `rtl/regfile.v` at `317b86c`, stock ladder
configuration:

| mutation to `rtl/regfile.v` | `reg_ch0` | wall |
|---|---|---|
| *(none — shipping design)* | PASS | 25.6s |
| rs2 write-through bypass deleted | counterexample, **bad state property 1**, k=20 | 13.9s |
| rs1 write-through bypass deleted | counterexample, bad state property 0, k=20 | 6.3s |
| both bypasses deleted | counterexample, bad state property 0, k=20 | 6.2s |
| `regs[waddr] <= wdata + 1` | counterexample, bad state property 0, k=20 | 5.1s |
| `waddr != 0` write suppression removed | PASS | 48.4s |

`reg_ch0` **does** cover invariant 6, in both directions, and it distinguishes them: deleting the
rs2 bypass trips bad state property **1**, which is `rvfi_reg_check.sv`'s rs2 assertion specifically,
not a generic failure. **Deleting the rs2 write-through bypass is therefore this repo's liveness
probe for `reg_ch0`** — one line, a direct invariant-6 violation, and it flips the check in about
14 seconds. Reach for it before believing any `reg_ch0` result obtained under a changed
configuration. Under `clk2fflogic` at genchecks' own depths it goes silent, which is how Finding 2
was established rather than assumed.

The one mutation that passes is not a coverage hole. Removing the `waddr != 5'd0` guard lets writes
land in `regs[0]`, but the read mux returns 0 for `rs1`/`rs2 == 0` unconditionally, so the written
value is unreachable by construction and there is no architectural difference for any check to find.
It is recorded here so the next reader does not spend the same hour on it.

**Read "counterexample" above as sby status `ERROR`, not `FAIL`.** `btorsim` is absent from this
environment, so once btormc reports `bad state property N reachable at bound k = M SATISFIABLE` the
`engine_0.trace` step dies with `COMMAND NOT FOUND` and sby exits `DONE (ERROR, rc=16)` — the check
genuinely failed and only the witness is missing. Same mechanical caveat ADR-0025 records for its
raised sweep. `check-baseline.sh` counts `ERROR` as non-PASS, so nothing is laundered; but ADR-0036's
open gap is that `formal/EXPECTED_FAIL` matches on names only, so grep for the `reachable at bound`
line rather than trusting the status word when using the probe.

## Consequences

- **`make -C formal check` now always re-runs.** A partial ladder can no longer be resumed; every
  invocation is a fresh run. That is the right trade for a gate, and `check-baseline` covers the
  re-grade case. Wall cost of the regeneration is about a second on a ~5-minute run.
- **Two full-ladder wall times on the same machine and the same tree: 310.4s and 532.1s.** Run-to-run
  spread on this hardware is a factor of 1.7 — this box runs several worktrees at once — so no
  ladder-level timing comparison in this repo should be read as meaningful below about 2×. Prefer
  serial single-check comparisons, as the tables above do, and say which kind a number is.
- **A standing warning for bespoke yosys scripts.** `formal/equiv.sh` — and anything else that drives
  yosys directly and ends in `write_btor` or `write_smt2` rather than going through sby's `prep`
  model step — does not get `formalff -clk2ff`'s polarity guard. Today the RTL is entirely posedge so
  nothing is lost; the moment that stops being true, those scripts model a different circuit and say
  nothing about it.
- **ADR-0025's depth derivation is unaffected**, because `clk2fflogic` is not adopted. Had it been,
  every number in that derivation would have needed doubling *and* decoupling from
  `RISCV_FORMAL_CHECK_CYCLE`, and the second half is not expressible in `checks.cfg`.
- **The negedge-BRAM regfile stays on the deferred list.** This ADR deliberately does not decide it.
  What it removes is one argument against it — the ladder will not silently mis-model it — and adds
  a concrete cost and shape for the decision when it is made.

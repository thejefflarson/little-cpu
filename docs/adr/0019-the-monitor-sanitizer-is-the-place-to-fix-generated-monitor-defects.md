# ADR-0019: The monitor sanitizer is where generated-monitor defects get fixed

**Status:** Accepted · 2026-07-28 · *Supplements ADR-0006, ADR-0013, ADR-0014; constrains CLAUDE.md invariant 7*

## Context

JEF-628 drove the RVFI ports and put `test/monitor.v` into both sim legs for the first time. The
moment the monitor became live, `div.S` and `rem.S` started failing — and the proposed response was
to add them to `test/EXPECTED_FAIL` with the analysis inline.

The analysis was right about the cause and wrong about the remedy.

`test/monitor.v`'s generated `monitor_insn_div` / `monitor_insn_rem` compute their spec result as a
three-way conditional whose first two branches are unsigned (`{32{1'b1}}` and a concatenation for
DIV; the raw `rvfi_rs1_rdata` input and `{32{1'b0}}` for REM) and whose last branch is
`$signed(a) / $signed(b)`. Per IEEE 1800 sign-context rules the conditional's own type is unsigned,
and that propagates down into its context-determined operands — so the division is evaluated
**unsigned**, silently. `wire [32-1:0] result = ...` cannot rescue it: an expression's type does not
depend on its left-hand side.

Verified independently at integration, not taken on report:

- The two sites were driven directly with known-correct RVFI values for the DIV/REM encodings.
  `-7 / 2` yields spec `0x7ffffffc` instead of `0xfffffffd`; `-7 % 2` yields `1` instead of `-1`.
  `7 / 2` is correct, so the harness is not itself broken. This rules out the alternative
  hypothesis — that the RVFI wiring was feeding the monitor bad values — because the monitor is
  wrong on inputs it was handed by the reviewer, with no core involved.
- Under yosys the `$div`/`$mod` cells carry `A_SIGNED 0` / `B_SIGNED 0`. Not a simulator quirk.
- The defect is confined to these two sites. Every other `$signed` in the file is either a
  self-determined `$signed(...)` assignment, a relational operator (operand signedness settled
  between the two operands, not by context), or a shift whose left operand determines the
  expression type. `DIVU`/`REMU` have no `$signed` at all.

## Decision

**A defect in the generated monitor is fixed in the build-time sanitizer, never in
`test/EXPECTED_FAIL`.**

`test/monitor.sim.v` — the gitignored derivative JEF-628 already introduced to strip the
`$time`-in-`$display` that yosys cannot elaborate — gains a second `sed` rule that rewrites

```
$signed(rvfi_rs1_rdata) / $signed(rvfi_rs2_rdata);
```

to wrap the arithmetic in `$signed(...)`, making it self-determined so the enclosing unsigned
conditional can no longer downgrade it. Same for `%`. Two lines change; the rule is anchored on the
exact operand names and the trailing semicolon, so it cannot silently match somewhere else.

**Both sim legs now read `test/monitor.sim.v`.** Previously only the cxxrtl leg did, and
`testbench.vvp` read `test/monitor.v` directly. Leaving that split would have meant the two legs
checking materially different specs — precisely the drift the shared `RISCV_FORMAL_MACROS` variable
in the same Makefile exists to prevent. One sanitized artifact, both legs. The cost is that the
iverilog leg's error banner loses its `%0t` timestamp; that is a diagnostic nicety, not a check.

`test/monitor.v` stays byte-identical to its generated form. **CLAUDE.md invariant 7 is intact and
is the reason this ADR exists**: "never hand-edit it" is a rule about the tracked file, not a
prohibition on transforming a build artifact derived from it. Read the other way, invariant 7 would
make any generated-tool defect permanently unfixable, which is not what it is for.

### Why not `EXPECTED_FAIL`

ADR-0014 is explicit that the baseline is a debt ledger for *this core's* defects, burned down one
line at a time by the fix that makes the test pass. A monitor-side defect has no such burn-down
path: nothing anyone does to `rtl/` would ever remove those lines. Parking them there converts a
finite, fixable tooling bug into a permanent hole in the headline metric, and — worse — into two
instructions the live monitor never checks. `div.S`/`rem.S` are exactly the tests that exercise
signed division, the arithmetic ALTOPS guarantees riscv-formal will never look at (ADR-0010). That
is the last place to accept a blind spot.

The check that this is not laundering a core bug into a fixed oracle: with the sanitizer rule in
place, `div.S` and `rem.S` **pass**, and the suite is 47/47 with `test/EXPECTED_FAIL` empty. Had the
core's division been wrong, correcting the oracle would have made the failure louder, not quieter.

### Upstream

The defect is in riscv-formal's `insns/generate.py` output at the pinned SHA, so it is upstream's to
fix properly. Report it; do not bump the pin speculatively chasing it (ADR-0013 — a pin bump is its
own deliberate act with its own re-verification cost). If a future pin bump fixes the codegen, the
`sed` rule stops matching and becomes dead; the sanitizer rule should then be deleted, and
`make monitor-check` plus a 47/47 run are what confirm it is safe to.

## Consequences

- `make test` stays 47/47 with an empty `test/EXPECTED_FAIL`. No regression in the headline metric,
  and signed DIV/REM are covered by the live monitor rather than exempted from it.
- A new class of change is sanctioned: transforming the generated monitor at build time. It is
  deliberately narrow — each rule must be anchored, minimal, and justified in a comment at the rule,
  and the diff `test/monitor.v` → `test/monitor.sim.v` should stay small enough to read in full. It
  is currently six lines. If it ever stops being reviewable at a glance, that is the signal to fix
  the generator upstream instead.
- The sanitizer is now load-bearing for correctness, not just for elaboration. A future reviewer
  must treat a change to it as a change to the oracle.

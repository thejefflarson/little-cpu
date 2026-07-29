# ADR-0017: A component-proof `assume` must name the structural fact it models

**Status:** Accepted · 2026-07-28 · *Supplements ADR-0006; ratified on integrating `a4662a2`*

## Context

`make -C formal components_decoder` failed on `main` and passes as of `a4662a2`. A proof going
green is exactly the kind of result that deserves a hostile look, because a proof that passes
vacuously is worse than one that fails honestly — it converts an open question into a false
reassurance, and nobody reopens it.

The failing assertion was, on `main`:

```verilog
always_ff @(posedge clk) past_pc <= $past(fetcher_pc);
always_ff @(posedge clk) if(clocked && !branch_jump && $past(uncompressed))
  assert(past_pc + 4 == pc);
```

Verified during integration: this fails at basecase **step 0**, and it is the `+ 4` uncompressed
assertion, not the `+ 2` compressed one that the CI comment and the folklore both named.

It fails because it is a **broken property, not a decode hole**. `components_decoder` stands alone
— no fetcher is instantiated — so `in.pc` is a free input. `past_pc <= $past(fetcher_pc)` therefore
compared `pc` against a two-cycles-stale sample of a signal the solver was free to choose
arbitrarily. No implementation of the decoder could ever have satisfied it. ADR-0006's remark that
the decoder's failure was "a decode-hole detector, not a broken property" was about the `$onehot`
`one_of` assertion nearby; it does not apply here, and this ADR records that distinction so the
next reader does not re-derive it.

`a4662a2` fixed it correctly, by modelling the missing structure:

```verilog
always_comb assume(in.pc == pc);   // littlecpu.v wires fetcher.pc <= decoder.pc,
                                   // and fetcher.v drives out.pc = pc combinationally
always_ff @(posedge clk) past_pc <= pc;
```

## Decision

**Accept the assumption, and require that every `assume` added to a component proof states the
structural fact it models and where that fact is itself checked.**

The `in.pc == pc` assumption is sound: `rtl/fetcher.v` assigns `out.pc = pc` combinationally and
`rtl/littlecpu.v` wires the decoder's `pc` output straight into it. It is a faithful model of the
real instantiation, and modelling a neighbour's wiring is the normal and correct way to make a
standalone component proof meaningful.

But it must be recorded that the assumption is **where the content went**, because two facts
established during integration bound what this proof now proves:

1. **Removing `assume(in.pc == pc)` alone reverts the proof to FAIL at the same assertion.** The
   assumption is precisely what closed it — not the added `!prev_stall` / `!prev_reset` guards.
2. **The proof is not vacuous.** Mutating `pc_inc` from `uncompressed ? 4 : 2` to `? 4 : 4` is
   still caught.

So the residual strength of the pc-increment pair is real but narrow. Given the assumption,
`pc <= fetcher_pc + pc_inc` becomes `pc <= pc + pc_inc`, and the assertions reduce to pinning
`pc_inc`'s two constants and the exclusivity of the `pc` writers — no non-branch arm of the decode
`case` may write `pc`, and the stall mux may not advance it. **What they no longer prove is the
fetcher↔decoder pc loop itself**, which is now assumed rather than checked.

That loop is checked at M2, by the full-core riscv-formal wrapper, where the real fetcher is
instantiated and nothing is assumed about it. That is the right level for it; it is not a gap so
much as a relocation, and this ADR is where the relocation is written down.

The new guards are not a vacuity hole: the case they exclude (`prev_stall`) has its own assertion,
`prev_stall → pc == past_pc`, added by the same change. Excluding a case *and* asserting it
separately partitions the state space rather than shrinking it.

## Rationale

The alternative was to demand a stronger standalone proof — instantiate a fetcher inside the
component task, or carry `pc` history in a way that survives a free input. Rejected: it would
duplicate at the component level a property M2's wrapper proves properly, on a project whose whole
plan is getting the riscv-formal ladder green. Spending M1 effort on a weaker version of an M2
result is the wrong trade.

What is *not* acceptable is the assumption going unrecorded. An `assume` is a promise made to the
solver on the reader's behalf, and an unexplained one is indistinguishable from a mistake.

## Consequences

- `components_decoder` is now a CI gate (`.github/workflows/ci.yml`), so it cannot regress
  silently.
- **`components_fetcher`, `components_accessor`, and `components_writeback` "pass" with zero
  assertions** — their `ifdef FORMAL` blocks contain only reset assumptions. Confirmed by running
  all three during integration. CI deliberately does not run them, and ADR-0006's slate to delete
  them stands. A green vacuous task in CI would be worse than no task. **Reporting them as passing
  proofs is not a result**, and should not be read as one in any engineer's report.
- Proving the fetcher↔decoder pc loop without assumption is added to M2's scope.
- The rule generalises: when a component proof needs a neighbour's behaviour, model it explicitly
  in an `assume`, name the file and line the model comes from, and say which higher-level check
  discharges it.

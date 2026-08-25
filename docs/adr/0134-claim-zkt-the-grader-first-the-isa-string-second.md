# 0134 — Claim Zkt: the grader first, the ISA string second

Status: accepted

## Context

Zkt (Data Independent Execution Latency) adds no instruction and defines no `misa` bit. It is a
promise, conditional on which of a listed set of instructions a vendor implements: *"if they claim
to have Zkt and implement any of the listed instructions, it must have data-independent latency."*
The list (rvkrypto's `zkt-list.adoc`) includes RV32I arithmetic, logical and shift, `MUL`/`MULH`/
`MULHSU`/`MULHU` and the arithmetic C encodings. It excludes `DIV`/`REM`, loads, stores, branches
and jumps.

Zkt is a **2-safety property**: run the same instruction with the same register numbers and two
different sets of register values, and the cycle count must agree. A single-trace check cannot
express a 2-safety property, so nothing under `formal/checks.cfg` reaches it and a BMC depth would
mean nothing here even if one were generated.

## Decision — read the design's own argument, grade it structurally, then state it

`rtl/decoder.v`'s `stall` is the exact OR of eight named reasons. Seven of them — `hazard`
(scoreboard and serialization together), `operand_stall`, `divider_stall`, `fetch_stall`,
`atomic_stall` and `bus_wait` — are built from register NUMBERS, instruction bits and control
state: which register a scoreboard slot names, whether a struct's `valid` bit is set, which
opcode decoded. None reads a register-file DATA output. The eighth, `region_stall`, does: it asks
whether `reg_rs1`'s own block sits near a mapped window's edge, so a plain load or store might
answer a cycle late. It is gated on `ls_access`, true only for the twelve load and store
encodings — none of which is on Zkt's list. A Zkt-listed instruction therefore has `ls_access`
false, `region_stall` false regardless of what `reg_rs1` holds, and every other reason blind to
register values by construction: its cycle count cannot depend on an operand's value.

`test/zkt_isolation_test.py` decides the first half of that argument the way
`formal/check-nonperturbation.py` decides RVFI's non-perturbation obligation: structurally, over
a fan-in graph, and explicitly weaker than a behavioural proof. It parses `rtl/decoder.v`'s own
continuous assigns and runs three checks, not one.

First, forward taint from `SEEDS` — `reg_rs1`, `reg_rs2` and `executor_out.rd_data`, every
decoder input this file finds wider than a register NUMBER that can carry a register-file or
CSR-file DATA output — with `region_stall` blocked from propagating further, so a path THROUGH it
is the expected shape and a path AROUND it is not. None of the seven value-blind reasons (nor
`hazard`'s own two components, nor the `stall`/`stall_own`/`stall_other` aggregates, read with
`region_stall`'s own contribution set aside) may be tainted this way.

Second, a fan-in walk over the *same* seven reasons and `region_stall`, in the other direction:
every identifier they read, transitively, that is not itself a continuous assign's left-hand side.
Forward taint alone only ever notices a leaf that happens to be a SEED; a value carried through an
always_ff register or a submodule's output port (`out.rs1 <= reg_rs1`; `branch_taken`, laundering
`cmp_eq`/`cmp_lt`, which are themselves seeded) has no assign of its own for forward propagation to
test, so it never gets marked and is read as clean by omission. The fan-in walk closes that gap: any
leaf it reaches that is not named in `KNOWN_CLEAN_LEAVES` (a module input or an always_ff register
proven to carry only a register NUMBER or control state) or in `SEEDS` stops the run rather than
being treated as safe.

Third, `region_stall`'s own assign is checked for a top-level `||` beside its `&&` chain — a
depth-zero split on `&&` alone still finds `ls_access` among the terms even when an unrelated `||`
term has been added beside the whole conjunction, which defeats the gate and is invisible to taint
too, because `stall`/`stall_other` read `region_stall` by name and never look inside its assign.

All three are checked against the real RTL, not asserted about it, and each has a demonstrated red
direction, wired into `make probe-gates`: deleting the `ls_access` gate; routing a `reg_rs1` bit
into `hazard` directly; moving an intermediate (`live_rs1`) into an `always_comb` block, which the
fan-in walk must stop on rather than silently accept; adding a top-level `||` beside
`region_stall`'s gate; and routing an `executor_out.rd_data` bit into a stall reason, which SEEDS
now names.

The second half is `rtl/executor.v`, read rather than graded: `MUL`/`MULH`/`MULHU`/`MULHSU`
resolve in decode's `init` state with no counter, so they take one cycle regardless of the
operands. `DIV`/`REM` are the one place this design is genuinely **not** constant-time — the
normal path runs the restoring divider for 32 cycles, and two early exits short-circuit to one
cycle: `rs2 == 0` and the signed-overflow case `INT_MIN / -1`. That is exactly why Zkt's list
excludes `DIV`/`REM` rather than the claim being false for including them.

**The load/store address-timing caveat is backwards from the usual case, and belongs here rather
than being rediscovered.** Most cores that vary load latency do it by CACHE STATE; this design has
no cache, and varies by ADDRESS ARITHMETIC instead — a load or store near a mapped window's edge
costs a cycle a deep access does not. Loads and stores are excluded from Zkt's list, so this does
not touch the claim, and the standard constant-time threat model already treats addresses as
non-secret, so the caveat sits inside the model rather than needing its own exception. The same
`region_stall` bubble is a global broadcast — it holds every in-flight instruction, including an
ALU operation behind an edge-adjacent load — which is ordinary in-order pipeline behaviour
attributable to the (excluded) load, not a new timing channel of its own.

The ISA string is claimed last, once the grader is in place: `rv32imac_zicsr_zifencei_zkt`. Zkt
has no `misa` bit — the privileged spec defines none, the way it defines none for the two
Z-extensions already claimed — so the string is the only runtime statement of it, and
`test/march_test.sh` grades the same seven sites it already grades, now against the extended
string. `formal/checks.cfg`'s `isa rv32imc` and the Makefile's `MONITOR_GEN -i rv32imc` are
untouched: there is no spec model for Zkt at the pinned riscv-formal SHA any more than there is
for A, so widening either would generate nothing and leave the check set claiming an ISA it does
not check — the same trap already recorded for the A extension.

## Cost

No RTL changed. `make fit` and `make netlist-digest` are unmoved because there is nothing for
either to see: this is a claim about a property `rtl/decoder.v` already has, decided by a new
hermetic Python check and an extended `-march` string, not a datapath change. No sweep is owed.

## Consequences

- `test/zkt_isolation_test.py` is a new `make test` prerequisite (`zkt-isolation-test`) and a new
  probe group in `test/probe_gates.sh`, with five named red directions and an anti-vacuity control
  that fails if the taint graph ever finds no path from `reg_rs1` to `region_stall` at all.
- The check is explicitly scoped to `rtl/decoder.v`'s continuous assigns, but it does not read a
  signal it cannot trace to one as clean. A stall reason's dependency moved into a procedural block,
  or a new decoder input wider than a register NUMBER, stops the run unless a human has classified
  it in `KNOWN_CLEAN_LEAVES`, `SEEDS` or `NON_VALUE_INPUTS` — so a refactor that changes what this
  script can see is a hard error at the exact signal it can no longer trace, not a silent pass.
  `SEEDS` and `NON_VALUE_INPUTS` are graded both ways against `rtl/decoder.v`'s own port list (read
  the way `test/port_connect_test.py` reads `littlecpu`'s): an unclassified wide input is red, and
  so is a classification that no longer names one.
- `DIV`/`REM`'s two early exits are not graded by this script — they are read against
  `rtl/executor.v` and recorded here. A future change to the divider's early-out arms should
  re-read this ADR's context before assuming Zkt is unaffected, since `DIV`/`REM` are excluded
  from the list precisely because of them.

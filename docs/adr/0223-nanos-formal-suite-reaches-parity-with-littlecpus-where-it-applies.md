# ADR-0223: nano's formal suite reaches parity with littlecpu's where it applies

**Status:** Accepted · 2026-09-30

## Context

nano (`nano/nano.v`, harnesses in `nano/formal/`) and littlecpu (`rtl/`, harnesses in `formal/`)
share the riscv-formal pin, the check generator (`formal/genchecks-local.py`,
`formal/genchecks-audit.py`) and the depth rules (`formal/depth_rules.py`). nano's suite had
fallen behind littlecpu's in five ways, and one nano target was red on main:

- CI never ran `components_traps`, `ill_e` or `ill_e_cover`, all three in `nano/formal/Makefile`'s
  `all`. `components_traps` is the only oracle for nano's trap entry and interrupt path, because
  every other nano harness ties `irq_meip` to 0. Nothing tied a Makefile's `all` list to the
  workflow, so a proof could sit in `all` and run nowhere.
- `make -C nano/formal ill_e_cover` failed on main (ADR-0218 noticed and left it). Its one goal, an
  E-illegal load reported with `rvfi_rd_addr == 16` and `rvfi_trap`, is unreachable on a correct
  core: a trapping retirement reports `rd` as 0.
- nano implements `mscratch` and generated neither `csrw_mscratch` nor `csrc_any_mscratch`.
- littlecpu has `make -C formal nonperturbation` (the structural proof that the RVFI
  instrumentation is unread) and the interrupt tie-off manifest; nano had neither.
- `checks.cfg` `#omit fault_ch0` said nano's bus "carries no fault line", while nano raises causes
  5 and 7 from a range test and reports `rvfi_mem_fault`.

## Decision

**A Makefile's `all` list is graded against CI.** `test/formal_ci_coverage_test.py`, on `make
test`'s path as `formal-ci-coverage-test`, reads `formal/Makefile` and `nano/formal/Makefile` and
fails when a target in `all` is run by no `make -C <dir> ...` line in `ci.yml`, by name or as a
transitive prerequisite of a target that is (`complete` runs `complete-exclusions` through its
rule). A matrix step written `components_${{ matrix.proof }}` expands over the workflow's own
`proof:` values, a comment or a `- name:` line does not count as a run, and `check` counts only
when both halves of its CI form are present: a `check-shard` line and a `check-baseline.sh` line
naming that design's `checks` directory. The script's `EXCEPTIONS` table is empty: every target in
both lists is run. Its forced-red probes in `test/probe_gates.sh` drop a nano step, a littlecpu
matrix entry and the collector, comment a step out, and add an unrun target to each `all`.

**nano's CI now runs** `components_traps`, `ill_e`, `ill_e_cover` (in `formal-extra`),
`nonperturbation` (in the `nonperturbation` job), and `interrupt-tie-off` through `check` and
`check-shard`, which now depend on it as littlecpu's do.

**`ill_e_cover` is a real control.** Its goal is a LOAD whose reported `rs1` is x17, retired as a
trap: reachable on the correct core (step 6), and the only field of the three E-illegal register
names a trap leaves reported. `make ill_e_cover` now also runs `cover-depth-tie.py` over its own
log against `ill_e.sby`'s depth, and is behind `ill-e-cover-probe`, which reuses
`formal/memcheck-cover-probe.py` (its `--check` now takes `ill_e`, nano only): the mutant assumes
`mem_ready` low, states a `cover property (!reset)` sentinel that must be reached, and requires the
goal unreached. That is the memchecks' own control, at the same shape, so an over-constraining
assume in `ill_e.sv` that made the property vacuous would go red.

**mscratch is generated.** `[csrs]` gains `mscratch any`, which generates `csrw_mscratch_ch0` and
`csrc_any_mscratch_ch0` under the same `[assume !csr[wc]_.*]` restriction to RV32E's register range;
`nano/nano.v` drives `rvfi_csr_mscratch_*` from `execute_instr`, held for the retire the way
mcycle and minstret are, under `RISCV_FORMAL_CSR_MSCRATCH`. `csrc_any` gets its `#floor` (9, the
`csrc_upcnt` shape) and a `[depth]` line, and `EXPECTED_CHECKS` two names. Red directions, run by
hand on the generated checks (both revert to PASS): writing `csr_new_value ^ 1` into `mscratch`
takes `csrc_any_mscratch_ch0` to FAIL, and reporting `~csr_new_value` as `wdata` takes
`csrw_mscratch_ch0` to FAIL.

**`fault_ch0` stays omitted, and the reason was wrong.** nano can refuse an access (causes 5 and 7),
so "no fault line on the bus" was not the ground. It was tried: `fault` in `[depth]`, upstream's
`RISCV_FORMAL_CSR_MCAUSE` defined (the check's source does not parse without it) and
`rvfi_csr_mcause_*` driven at execute. It fails on a correct core for two reasons, both in nano's
RVFI report. Measured: with the report otherwise as shipped it fails at the check's `mcause == 7`
assertion, because `captured_load_fault`/`captured_store_fault` come from the region test alone,
so an illegal encoding with a load or store opcode that also names an out-of-window address
reports an access fault while `mcause` says 2; gating both on `trap_cause_value` fixed that and
took all 79 generated checks green. But `captured_is_opm` also sets `rvfi_mem_fault` for an
M-extension encoding, which the check reads as an instruction fetch fault and requires `insn == 0`
of, and removing it made `make nano-test` report `divide.S` and `mul.S` as MONITOR-ERROR 106: the
sanitized monitor's spec model executes mul/div, nano traps them, and the flag is what excuses
the disagreement. So the flag has two consumers that disagree about what it means, and the whole
attempt was reverted, `nano.v` carries no fault or mcause change, and the omit line now says this.
DECISION NEEDED: excuse M encodings to the monitor through a signal of their own, then the
gated fault flags and the mcause report can land and `fault_ch0` can be generated; not done here
because it edits the oracle. `RISCV_FORMAL_MEM_FAULT` stays defined because `nano.v` and the
generated instruction checks read it.

**nonperturbation is one script over both designs.** `formal/check-nonperturbation.py` takes a
design name (`littlecpu`, the default, or `nano`) and a table of sources, top module, and the
defines that switch the instrumentation on; the comparison after that is unchanged. nano's build
reads the pin's `rvfi_macros.vh` first because its ports come from that header. nano: 12,490 cells
gold and gate, identical, against 15,685 instrumented and 41 `rvfi_*` ports. The gate had no red
direction for either design; `formal/nonperturbation-probe.py` adds one, a real output that reads
bit 0 of an `rvfi_*` port under `ifdef RISCV_FORMAL` (the plain build must still elaborate, or the
gate goes red for the wrong reason: measured, my first mutant did exactly that), requiring the
gate's own "cell histogram DIFFERS" line. It is a prerequisite of both Makefiles' `nonperturbation`
target and is exercised by `test/probe_gates.sh` against a stub checker.

**The interrupt tie-off has a manifest for nano.** `formal/check-interrupt-tie-off.py --core nano`
grades `nano/formal/INTERRUPT_TIE_OFF`: five harnesses tie `.irq_meip(1'b0)`; `traps.sv` is a new
`FREE` record that must instantiate the core and must not tie it, so the one harness that sees an
interrupt is graded as well as the ones that must not; and any other port a `HARNESS` file holds
at a constant is red (nothing else is). The upstream half, the `rvfi_intr` and interrupt-CSR sweep
of the pinned clone, is unchanged and shared. There is no `MULTIHART_TIE_OFF`: nano is one hart with
one initiator and no arbiter, so it has no grant wait or snoop to tie. `docs/manifests/interrupt-tie-off.md`
says so.

## Parity with littlecpu

| littlecpu | nano |
|---|---|
| `complete`, `complete_cover`, `complete-exclusions` | present |
| `check`, `check-shard`, `check-baseline`, `checks`, `EXPECTED_CHECKS`, `EXPECTED_FAIL`, `check-solver-%` | present |
| `dmemcheck`, `imemcheck`, their `_cover`s and probes, `memcheck-depth` | present |
| `components_traps` with `traps-region-probe`, `traps-tval-probe` | present; **added to CI here** |
| `interrupt-tie-off` | **added here** (`FREE` record for `traps.sv`) |
| `nonperturbation` | **added here**, one script for both, with a new forced-red probe for both |
| `remeasure-fg` | present |
| `genchecks-check` | not applicable: `nano/formal` runs the same `formal/genchecks-local.py`, which `make -C formal genchecks-check` already diffs against the pin; `check-rvfi-insn-check` (nano's own) grades the one forked check file |
| `multihart-tie-off` | not applicable: one hart, no arbiter, no grant wait or snoop input |
| `cover` (`cover.sby`, five goals) | not applicable, and the substitute is adequate: nano's generated `cover` reaches two retires, and `complete_cover` reaches every opcode class the core executes (LOAD, STORE, OP, OP-IMM, BRANCH, JAL, JALR, LUI, AUIPC, all three RVC quadrants, and a trapping M encoding). littlecpu's five goals add multiple reads, writes, long and compressed retires because its fetch window pairs words; nano fetches one parcel stream serially, so there is no pairing to reach |
| `pcloop_cover`, `components_pcloop` | not applicable: no pipeline, no wrong-path state to induct over |
| `busarbiter_cover`, `components_busarbiter` | not applicable: no arbiter |
| `components_decoder`, `components_accessor` | not applicable as modules: nano has no separate decoder or accessor. `complete`, `ill_e` and `traps` grade decode; `dmemcheck` and `components_memreq` grade the access, and `components_qspi` the bus behind it |
| `components_executor`, `executor-zkt-probe`, `decoder-zkt-probe`, `zkt_isolation_test` | not applicable: no M extension (no multiplier or divider to prove) and nano claims no Zkt |
| `abc-engine` | not applicable: `nano/formal/complete.sby` uses `btor btormc`, not `abc bmc3` |
| `retune-checks.py` (`insn_*` re-pointed at `btor pono`) | not applied: a speed change measured on littlecpu's runner pod for its own slowest checks; nano's `insn_*` finish in about 11 s at depth 30 and its four shards are not a bottleneck. Owed only if a nano shard becomes one |
| `mutation-check`, `MUTATION_DETECTORS` | **open**: nano has no mutation table. DECISION NEEDED: a nano `test/mutations/` set paired with its detectors is the next parity item and is not built here |
| generated `insn`, `reg`, `pc_fwd`, `pc_bwd`, `causal`, `causal_mem`, `liveness`, `unique`, `hang`, `ill`, `csrw_mcycle`, `csrw_minstret`, `csrc_upcnt_*` | present |
| generated `csrw_mscratch`, `csrc_any_mscratch` | **added here** |
| generated `fault` | omitted on nano, reason corrected and measured above; DECISION NEEDED on the M-encoding excusal |
| generated `csrc_inc_*` | omitted on both, same reason: red on a correct core |
| generated `bus_*`, `causal_io` | omitted on both, with reasons in each `checks.cfg`: nano's fault-line and MMIO-region omits stand |

## Consequences

- `formal-extra` gains three steps and the `nonperturbation` job two runs of each script; their wall
  times are recorded in the pull request that landed this.
- A pin bump that adds an upstream check reading `rvfi_intr` now also reddens nano's manifest, which
  is the point of sharing the script.
- `ill_e`'s own property still reads `rvfi_rd_addr[4]`, and a trapping retirement reports `rd` as 0,
  so its `rd` term is dead: an instruction E-illegal by `rd` alone is not asserted trapping. The
  `rs1` and `rs2` terms are live and the wrong-rule probe covers them. Strengthening the antecedent
  from `rvfi_insn[11:7]` is a separate change.
- nano still misreports `rvfi_mem_fault` for an illegal encoding that also names an out-of-window
  address (cause 2, flagged as an access fault). Nothing graded reads it today.

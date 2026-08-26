# 0137 — The Zkt grader moves to the elaborated netlist, and two of its checks off the netlist entirely

Status: accepted

## Context

ADR-0134's `test/zkt_isolation_test.py` read `rtl/decoder.v`'s own source text: a regex over
`assign` statements, a hand-rolled walk of `&&`/`||` splits, and a `KNOWN_CLEAN_LEAVES` allowlist
for anything the text scan could not trace. Three rounds of review defeated it, and every defeat
was a fact only elaboration knows, not a fact regex-and-AST reading of Verilog can be taught:

- A value written procedurally (`out.rs1 <= reg_rs1` inside an `always_ff` block, or
  `branch_taken` inside an `always_comb` one) has no continuous `assign` of its own. Forward taint
  followed only `assign` left-hand sides, so a path running through either one was invisible to
  it, and the fan-in walk's answer for both was "leaf — read `KNOWN_CLEAN_LEAVES`", which trusted
  them without ever checking what they depend on.
- `ls_answer_valid` was in that same allowlist, claimed as control state. Its SET arm is
  `ls_capture = region_stall && !stall_own`, and `region_stall` reads `ls_settled`, itself read off
  `reg_rs1[31:11]` — a function of a register VALUE, not a fact the allowlist entry ever argued for.
- An `assign` inside an un-taken `` generate if (0) `` (or a dead `` `ifdef ``) parses as a real
  continuous assign to a regex that does not understand generate or macro semantics, and can mask a
  genuine driver sitting underneath two textual definitions of one name.
- `input logic [XLEN-1:0]` reads as width 1 to a `RANGE` regex that matches only a numeric literal
  after the colon, not a parameter.

## Decision, part one — elaborate, then read connectivity off the netlist

`test/zkt_isolation_test.py` follows `formal/check-nonperturbation.py`'s template: run `yosys -q
-s`, write a JSON netlist (`proc; flatten; memory_map; simplemap`, no `opt_clean` — a pure
bit-select like `ls_block = reg_rs1[31:21]` has no cell of its own, and a fanout sweep would fold
its name away in favour of `reg_rs1`'s, which is exactly the name this script needs to keep), and
read cells and named nets from the result. Zero regexes. Every one of the four defeats above is
closed by construction rather than by a new special case: a `generate` arm, an `` `ifdef ``,
procedural assignment and a parameterised width are all already resolved by the time this script
sees the netlist, and a port's width is `len(bits)`, not a pattern match.

**The property.** `stall` and the eight reasons it is built from — `ls_access`, `serialize`,
`hazard_rs1`, `hazard_rs2`, `hazard`, `operand_stall`, `atomic_stall`, `stall_own`, `stall_other` —
must never depend, on the elaborated netlist, on a register-file or CSR-file DATA output
(`reg_rs1`, `reg_rs2`, `executor_out.rd_data`) except through `region_stall`, the one reason
allowed to.

## Decision, part two — region_stall's own gate and ls_access's encoding set move to a proof

Two further facts used to be decided by the same structural netlist walk that decides the taint
property above: that `region_stall` can only assert alongside `ls_access`, and that `ls_access` is
true for exactly the eight base load/store encodings. **Three rounds each defeated a different
structural spelling of the same two facts**, and each fix was correct in isolation:

1. Graded `ls_access` by NAME. `region_stall`'s AND-tree walk stopped the moment it reached a net
   called `ls_access`, without asking what `ls_access` resolved to — `assign ls_access =
   instr_ls_load || instr_ls_store || instr_add;` passed unchanged, because `instr_add` carries no
   register-file DATA output for the taint check to find and `ls_access` still names `ls_access`.
   Fixed by a fourth check walking `ls_access`'s own OR tree to an exact declared set.
2. Treated `$_NOT_` as polarity-preserving. Both walks recursed through a NOT cell and returned the
   child's leaves UNCHANGED, so `region_stall = !ls_access && !ls_settled && !ls_answer_valid` —
   which asserts on every NON-load/store instruction, the exact opposite of the intended gate —
   bottomed out at the leaf `ls_access`, not `!ls_access`. Fixed by threading an accumulated
   polarity flag through both walks and tagging each leaf with it.
3. `(ctype in AND_TYPES) != negated` reads True for ANY non-AND cell type under odd polarity, not
   only an OR — a `$_MUX_`, `$_XOR_`, a comparator, an adder. `region_stall =
   !(ls_settled ? !ls_access : 1'b0);` — a real dependence on `ls_settled`'s (and so `reg_rs1`'s)
   value — walked straight through the MUX as though it were the AND it is not, reaching the leaves
   `{ls_access, !ls_settled}` with no OR found. Fixed by checking a cell is an AND or an OR at all
   before either polarity comparison runs.

**That is a pattern, not three unrelated bugs.** Both properties are single-trace, exact-set-equality
invariants with **no VALUE comparison in them at all** — the same shape as
`assert(out.is_amo == (out.is_amoswap || ...))`, which already sits in `rtl/decoder.v`'s own
`ifdef FORMAL` block a few lines away. A structural walk over a netlist is an over-approximation,
built for a 2-safety question a single-trace BMC check cannot express (which is exactly what the
taint half of this script still needs it for). Handed a 1-safety exact equality instead, an
over-approximation is the wrong tool: it has edges — a name, a polarity, a cell type — that an
exact decision procedure does not, and each round found a different one.

So `rtl/decoder.v` now states both facts as assertions instead of leaving them to be re-derived by
a Python walk:

```systemverilog
always_comb if (clocked) assert(!region_stall || ls_access);
always_comb if (clocked)
  assert(ls_access == (instr_lb || instr_lbu || instr_lh || instr_lhu ||
    instr_lw || instr_sb || instr_sh || instr_sw));
```

Asserted against the eight base encoding signals rather than `instr_ls_load`/`instr_ls_store`,
because those two intermediates are nothing but this same OR restated in two pieces — asserting the
flat sum says everything the two-piece version would, plus that the split matches it exactly.
`instr_lw`/`instr_sw` each fold two further compressed forms in (`c.lwsp`/`c.lw`, `c.swsp`/`c.sw`)
behind their own `assign`, one hop past what either assertion states, the same one-hop boundary the
old structural walk drew everywhere it stopped at a named net.

`make -C formal components_decoder` proves both by k-induction (the `decoder` task is already
`mode prove`). `formal/decoder-zkt-probe.py` is the demonstrated red direction, the pattern
`make -C formal traps-region-probe` set for `formal/traps.sv`: it builds two cores one line of
`rtl/decoder.v` apart — one with the `ls_access` conjunct dropped from `region_stall`, one with an
encoding added to `ls_access` — and requires each to fail at its own assertion's line, wired as a
prerequisite of `components_decoder` so the control cannot be skipped. Its own grading (a respelled
assertion or mutation site stopping the run, a solver that wrote no verdict, a proof going red
somewhere else) is covered by a `test/probe_gates.sh` group against a stub `sby`, mirroring
`traps-region-probe`'s own.

**All eight historical defeats from the three rounds above were re-run by hand against the new
proof, and every one still fails** — seven report one of the two new assertions as their FIRST
failure, and the eighth (`ls_access = !(instr_ls_load || instr_ls_store)`, a NOT wrapped around the
whole OR) reports a different, pre-existing sign-extension assertion first (it reads `ls_access`
under an `if (ls_access)` guard nearby) but fails the new encoding-equality assertion too, confirmed
with `--keep-going`.

## What test/zkt_isolation_test.py keeps

Two checks, both genuinely 2-safety and with no exact decision procedure to hand a solver instead:

1. **Forward reachability**, from the bits of `reg_rs1`/`reg_rs2`/`executor_out.rd_data`, following
   every cell's inputs to its outputs including a flip-flop's D to its Q — so a value laundered
   through a register is reachable the same as one computed combinationally this cycle. `region_stall`
   is blocked from being used as a taint SOURCE past its own one hop
   (`formal/check-nonperturbation.py`'s own restricted-taint discipline, reused rather than
   reinvented): it can still become reachable, since `stall` reads it directly and `out`'s own
   bubble condition re-lists it beside `hazard`/`operand_stall`/etc, but nothing it feeds is treated
   as tainted BECAUSE of that read.

   Blocked the same way, for the same reason: `out.rd`, `out.is_amo`, `out.valid`,
   `executor_out.rd`, `executor_out.valid` (`CONTROL_FIELDS`). **What licenses blocking them is not
   their width** — a 5-bit field can perfectly well carry a value-dependent decision — **it is that
   every path a register VALUE can take to reach one of them is TRAP-MEDIATED**: `out.rd` reads 0
   instead of the real destination exactly when a register value decided the PREVIOUS instruction
   was misaligned or out of region, and taking a trap is architecturally VISIBLE (a different
   instruction retires, at a different pc), not a covert timing channel a Zkt-listed instruction's
   cycle count could leak an operand VALUE through. The 5-bit bound `control_field_bits` enforces is
   kept as a TRIPWIRE on top of that argument, not in place of it — it catches a field growing wide
   enough to plausibly carry a raw VALUE rather than a decision ABOUT one, the same line
   `SEED_PORTS`/`NON_VALUE_PORTS` already draw for input ports, but it cannot by itself tell a
   genuinely trap-mediated 5-bit field from a coincidentally narrow VALUE slice; that argument is
   made by eye, per field, in `CONTROL_FIELDS`' own comment. `executor_out.valid`/`executor_out.rd`
   are inert rather than wrong — primary INPUT bits with no driving cell inside `decoder` for
   forward taint to ever reach — and kept anyway, for the same reason `out`'s two are named rather
   than left to a width rule.

   `region_stall`'s own captured state — `ls_capture`, `ls_answer`, `ls_answer_valid` — gets a
   second, independent reachability pass, seeded from THEIR OWN bits rather than from
   `reg_rs1`/`reg_rs2`, with the same two blocked sets: none of the nine reasons may read one of
   these three directly, whatever mediates the read.
2. **Port coverage, both ways**, off the netlist's own measured widths rather than a `[N:0]` match:
   every decoder input wider than 5 bits must be `SEED_PORTS`/`STRUCT_FIELD_SEEDS` or
   `NON_VALUE_PORTS`/`STRUCT_FIELD_NON_VALUE`, and a struct-typed port's field offsets are read off
   a tiny satellite module elaborated against `rtl/structs.v` alone: a renamed field fails that
   elaboration outright, an added or resized one fails a sum-of-widths check. Every classification
   is checked stale in both directions.

**What is NOT proven.** The taint half is the same explicitly weaker-than-equivalence stance
`formal/check-nonperturbation.py` states for itself: connectivity, not a 2-safety proof by itself —
it is a sound over-approximation of one. `rtl/executor.v`'s divider is unchanged: read by eye, not
graded here, exactly as ADR-0134 recorded.

## Cost

No RTL behaviour changes. Both new assertions sit inside `rtl/decoder.v`'s `ifdef FORMAL` block;
neither `make fit` nor `make soc-timing` (nor `netlist-digest`, which shares `soc-timing`'s synth
recipe, `SOC_SYNTH`) ever defines `RISCV_FORMAL` — their yosys recipes pass no `-D` flags at all, so
`` `ifdef FORMAL `` is never true when either reads `rtl/decoder.v`. Thirteen yosys elaborations per
`make probe-gates` run for `test/zkt_isolation_test.py`'s own group (down from twenty-one before
this round, the two checks it lost included); `formal/decoder-zkt-probe.py`'s own group runs against
a stub `sby` and elaborates nothing.

## Consequences

- `test/zkt_isolation_test.py` no longer has a `KNOWN_CLEAN_LEAVES`-shaped escape hatch, and now
  keeps only the half of the original argument that is genuinely 2-safety: forward reachability and
  port coverage. `and_tree_leaves`, `bool_tree_leaves`, `LS_ACCESS_TRANSPARENT`,
  `LS_ACCESS_ENCODINGS` and `GATE_TERM` are gone along with the checks that used them.
- `rtl/decoder.v` states, as assertions, the two facts that used to be re-derived from its netlist
  by a Python walk. `make -C formal components_decoder` proves both by k-induction;
  `formal/decoder-zkt-probe.py` is their demonstrated red direction and a prerequisite of that
  target, the same relationship `traps-region-probe` has to `components_traps`.
- `CONTROL_FIELDS`' comment states the TRAP-MEDIATED property that actually licenses the exemption,
  not merely the width bound that used to be its only stated reason.
- `test/probe_gates.sh`'s `test/zkt_isolation_test.py` group is fifteen probes (down from
  twenty-three): the control, three shapes of forward reachability (a direct read, a value
  laundered through a register, a value carried through a comparator), the other seed, the two
  findings that closed the `ls_answer_valid`-family leak (a direct read, and the same read behind a
  dead `generate if (0)` decoy), an unclassified wide port, `CONTROL_FIELDS` emptied against the
  shipping decoder and its width bound, a stale classification, an undriven stall-reason name, the
  anti-vacuity control, and the two usage-error cases. A new `formal/decoder-zkt-probe.py` group is
  thirteen probes, mirroring `traps-region-probe.py`'s own: the control, both directions of "an
  assertion that cannot fail is worth nothing", both directions of "a proof going red elsewhere is
  not evidence", a solver with no verdict, an empty status file, two respelling guards, the RTL
  moving away, and the two ways reading `formal/components.sby`'s own `decoder` task script can come
  up empty.

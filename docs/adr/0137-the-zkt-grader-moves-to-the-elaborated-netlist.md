# 0137 — The Zkt grader moves to the elaborated netlist

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

## Decision — elaborate, then read connectivity off the netlist

`test/zkt_isolation_test.py` now follows `formal/check-nonperturbation.py`'s template: run `yosys
-q -s`, write a JSON netlist (`proc; flatten; memory_map; simplemap`, no `opt_clean` — a pure
bit-select like `ls_block = reg_rs1[31:21]` has no cell of its own, and a fanout sweep would fold
its name away in favour of `reg_rs1`'s, which is exactly the name this script needs to keep), and
read cells and named nets from the result. Zero regexes (`grep -c 're\.compile\|re\.match\|re\.search'`
is 0). Every one of the four defeats above is closed by construction rather than by a new special
case: a `generate` arm, an `` `ifdef ``, procedural assignment and a parameterised width are all
already resolved by the time this script sees the netlist, and a port's width is `len(bits)`, not a
pattern match.

**The property.** `stall` and the eight reasons it is built from — `ls_access`, `serialize`,
`hazard_rs1`, `hazard_rs2`, `hazard`, `operand_stall`, `atomic_stall`, `stall_own`, `stall_other` —
must never depend, on the elaborated netlist, on a register-file or CSR-file DATA output
(`reg_rs1`, `reg_rs2`, `executor_out.rd_data` — the only decoder inputs wider than the 5 bits a
register NUMBER ever needs) except through `region_stall`, the one reason allowed to, because it is
conjoined with `ls_access`, true only for the twelve load and store encodings Zkt's list excludes.

Five checks:

1. **Forward reachability**, from the bits of `reg_rs1`/`reg_rs2`/`executor_out.rd_data`, following
   every cell's inputs to its outputs including a flip-flop's D to its Q — so a value laundered
   through a register is reachable the same as one computed combinationally this cycle. `region_stall`
   is blocked from being used as a taint SOURCE past its own one hop (`formal/check-nonperturbation.py`'s
   own restricted-taint discipline, reused rather than reinvented): it can still become reachable —
   that is the point, since `stall` reads it directly and `out`'s own bubble condition re-lists it
   beside `hazard`/`operand_stall`/etc — but nothing it feeds is treated as tainted BECAUSE of that
   read. `region_stall` can only be true for a load or a store, so its influence on a Zkt-listed
   instruction's own stall decision is zero regardless of what gates it.

   Blocked the same way, for the same reason: `out.rd`, `out.is_amo`, `out.valid`,
   `executor_out.rd`, `executor_out.valid`. Reading a register NUMBER (`rd` is `[4:0]`, the SAME
   width the SEED/NON_VALUE split already draws its line at) or a single control flag back — which
   is what `live_rs1`/`live_rs2`'s RAW-hazard check and `atomic_stall` do — is not reading a VALUE,
   even though which number or flag ends up there can itself have been decided by a fault check
   that read one (`out.rd` reads 0 instead of the real destination when the previous instruction
   trapped, and whether an ATOMIC traps for misalignment reads `reg_rs1`). Without blocking these,
   real RTL — nothing mutated — fails: `out.valid`'s bubble condition reads `region_stall` a SECOND
   time, independently of `stall`'s own read of it, and `live_rs1`/`live_rs2` read `out.rd`/
   `out.valid` back for the RAW hazard check the design has always made on register NUMBERS.
   Discovering this is what the netlist-based rebuild is *for* — the RTL-text version could not see
   procedural drivers at all, so it never had to confront whether this path was safe.
2. **`region_stall`'s own captured state** — `ls_capture`, `ls_answer`, `ls_answer_valid` — gets a
   second, independent reachability pass, seeded from THEIR OWN bits rather than from
   `reg_rs1`/`reg_rs2`, with the same two blocked sets. This is what closes the finding above: none
   of the nine may read one of these three directly, whatever mediates the read.
3. **The gate's own shape.** `region_stall`'s driving cell — or a chain of `$_AND_`/`$_NOT_` cells
   feeding it, the netlist's analogue of one text-level `&&`/`!` expression — must bottom out at a
   set of NAMED leaves that includes `ls_access`, and must never pass through an `$_OR_` cell on the
   way. A `||` added beside the `&&` chain shows up as an OR cell sitting directly on this path,
   however the source text spelled the precedence.
4. **Port coverage, both ways**, off the netlist's own measured widths rather than a `[N:0]` match:
   every decoder input wider than 5 bits must be `SEED_PORTS`/`STRUCT_FIELD_SEEDS` or
   `NON_VALUE_PORTS`/`STRUCT_FIELD_NON_VALUE`, and a struct-typed port's field offsets are read off
   a tiny satellite module — `input <typedef> val` plus one output per field, sized with
   `$bits(val.<field>)` — elaborated against `rtl/structs.v` alone: a renamed field fails that
   elaboration outright, an added or resized one fails a sum-of-widths check. Every classification
   is checked stale in both directions, which is the asymmetry the old `KNOWN_CLEAN_LEAVES` never
   had: `SEEDS`/`NON_VALUE_INPUTS` were graded against the port list both ways, but a struct field
   or a submodule output landing in `KNOWN_CLEAN_LEAVES` was trusted forever with no check that it
   was still reached, still narrow, or still true. `CONTROL_FIELDS`' own fields are held to the same
   5-bit bound `SEED_PORTS`/`NON_VALUE_PORTS` draw the line at — a field measured wider on the
   elaborated netlist is an error, not a silently-widened block.
5. **`ls_access` itself.** Check 3 grades `ls_access` by NAME — it stops the moment `region_stall`'s
   AND-tree reaches a net called `ls_access`, without asking what `ls_access` resolves to. `assign
   ls_access = instr_ls_load || instr_ls_store || instr_add;` passed checks 1–4 unchanged: `instr_add`
   carries no register-file DATA output, so reachability never reaches it, and `ls_access` still
   names `ls_access`. This check walks `ls_access`'s own OR tree — through `$_AND_`/`$_OR_`/`$_NOT_`
   cells, unlike check 3's AND-only walk, because `ls_access` IS an OR of ORs — past its two named
   intermediates (`instr_ls_load`, `instr_ls_store`, seen THROUGH rather than stopped at) down to the
   eight base loads and stores it is declared to equal exactly, in both directions. `instr_lw`/
   `instr_sw` each fold two further compressed forms in behind their own `assign`, one hop past where
   this check stops, the same one-hop boundary check 3 already draws everywhere else.

**What is NOT proven.** This is the same explicitly weaker-than-equivalence stance
`formal/check-nonperturbation.py` states for itself: connectivity, not a 2-safety proof. A signal
that is *structurally* wired to a register value but only ever *functionally* activated by a gate
this script does not itself verify (the way `region_stall`'s is verified, narrowly, by check 3)
would read as reachable and fail loud rather than pass quiet — which is the failure direction this
class of check should err toward. `rtl/executor.v`'s divider is unchanged: read by eye, not graded
here, exactly as ADR-0134 recorded.

## Cost

No RTL changed. Seventeen yosys elaborations per `make probe-gates` run (one per mutated fixture,
the two usage-error probes excepted — they exit before this script ever reaches one) add a few
seconds to that target; `make test`'s own single run is one elaboration. No `fit` or `soc-timing`
number can move, because nothing under `rtl/` was touched.

## Consequences

- `test/zkt_isolation_test.py` no longer has a `KNOWN_CLEAN_LEAVES`-shaped escape hatch. Every
  signal this script did not itself verify either has a driving cell it can walk, or is a decoder
  input port classified in one of four tables and checked stale in both directions.
- `CONTROL_FIELDS` (`out.rd`/`out.is_amo`/`out.valid`, `executor_out.rd`/`executor_out.valid`) is a
  narrow, named exemption, not a width-based rule: blocking every ≤5-bit signal automatically was
  considered and rejected, because a stall reason reading a genuine ≤5-bit SLICE of a register
  value directly would then go undetected — the exemption has to name the specific fields it
  covers, the way `region_stall` itself is one named signal and not a class. Its own fields are
  still held to the 5-bit bound (check 4's amendment): the table's only written justification is
  that bound, and until now nothing asserted it. `executor_out.valid`/`executor_out.rd` are inert
  rather than wrong — they are primary INPUT bits with no driving cell inside `decoder`, so blocking
  them as taint sources is a no-op — and are kept anyway, because the table states what is safe to
  read back, not merely what currently matters.
- `ls_access` is graded by what it resolves to, not by its name (check 5's amendment): an encoding
  added to or dropped from its own OR tree is red, naming which one, whether the mutation sits in
  `ls_access`'s own `assign` or one hop down in `instr_ls_load`/`instr_ls_store`'s.
- `test/probe_gates.sh`'s group is nineteen probes: the control, both directions of the gate's own
  shape, three shapes of forward reachability (a direct read, a value laundered through a register,
  a value carried through a comparator), the other seed, the two findings above (the direct
  `region_stall`-family read, and the same read behind a dead `generate if (0)` decoy), an
  unclassified wide port, both directions of `ls_access`'s own encoding set, `CONTROL_FIELDS`
  emptied against the shipping decoder and its width bound, a stale classification, an undriven
  stall-reason name, the anti-vacuity control, and the two usage-error cases.

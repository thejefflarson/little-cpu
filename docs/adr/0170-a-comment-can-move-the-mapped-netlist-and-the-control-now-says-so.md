# ADR-0170: A comment can move the mapped netlist, and the control now says so

**Status:** Accepted · 2026-09-07 · *Amends ADR-0122. Narrows the digest's claim in the direction
already measured, and moves the digest's comment-class control onto a file where it can fail for
the reason it exists. No commitment touched, no gate on `make test` or CI.*

## Context

PR #291 (`45f0b9b`) cut every file in `rtl/` to at most 5% comment lines and rewrote what survived
-- a diff that is entirely comments and blank lines. Measured fresh (Yosys 0.68+48 `ff5817c34`,
nextpnr-ice40 0.11-1-`g62e659ed`, no `make`, every input SHA-256'd): that commit moved the SoC's
mapped netlist **6,266 -> 6,289 cells (+23 `SB_LUT4`, 0.37%)**. Synthesis is deterministic --
repeated fresh runs of both trees digest identically -- and the move bisects to `rtl/csrs.v` alone;
each of the other fifteen `rtl/` files carries the comment diff with the netlist unmoved.

ADR-0122 built exactly the control this should have failed: `soc/netlist_determinism.sh` mutates a
tree with a comment, a blank line and a dead tie-off and asserts the canonical digest and the placed
bitstream are both unmoved. It never caught this, because its comment-class case lands two lines at
the top of `rtl/littlesoc.v` -- 153 lines, and never once measured to move. **The control was correct
in shape and pointed at a file it had already proven safe.** A grader that inserts a comment where a
comment cannot move anything is not exercising the class it is named for.

## The mechanism, now demonstrated rather than inferred

A separate investigation, run in parallel with this ticket on the same toolchain: the two versions
of `rtl/csrs.v` parse to an **identical AST** (975 node lines, locations masked) and identical RTLIL
once `src` is masked. Every one of the 42 changed lines matches `^\s*//`; every non-comment line is
byte-identical in sequence. The two files' *meaning* is unarguably the same.

The carrier is yosys's own auto-generated cell-name string, which embeds a `file:line` pair (for
example `$logic_and$rtl/csrs.v:92$1`). `abc9_ops.cc` builds a topological sort over cell names
(`TopoSort<IdString, RTLIL::sort_by_id_str>`) before `write_xaiger` emits them, and
`sort_by_id_str` is `IdString::lt_by_name` -- a `memcmp` over the name bytes, comparing the embedded
line number's digits as characters rather than as a number. Everything before that pass is a pure
renaming: per-stage RTLIL dumps (`flatten`, `coarse`, `map_ram`, `map_ffram`, `map_gates`,
`map_ffs`, `map_luts`), with `src` dropped and `` `line `` directives masked, are byte-identical at
every stage, and the `xaiger` header matches exactly (`aig 18593 5126 0 9238 13467`); the named
PI/PO sets are identical, only their *order* differing (1,237 of 7,671 positions). ABC9's own
order-sensitive passes (`&dch`/`&if`) see the same gate set in a different order and pick a
different one of several functionally equivalent LUT mappings: `and=18454 ch=2653` against
`and=18612 ch=2854`, `LUT=3801` against `LUT=3831`. Decisive test: patching **only** `` `line ``
directives inside yosys's pre-`abc9_ops` intermediate dump -- no source file in play at all --
reproduces both digests on demand: #291's text told to report base's line numbers gives base's
digest (6,266); base's text told to report #291's numbers gives #291's (6,289). Text held constant,
numbers varied, the netlist follows the numbers, not the text.

**This is a property of the flow, not of `rtl/csrs.v` alone.** Putting `rtl/csrs.v` first on the
`read_verilog` command line -- which renumbers yosys's global `$autoidx` in every generated name --
produced two more, different digests (6,231 and 6,258) from the same two source trees. Renaming a
file does not reproduce it on its own: `flatten` prefixes the instance name onto every cell, so a
renamed file changes sort order only through where it happens to push `$autoidx`, not by the filename
appearing in a compared string.

**+23 cells is 0.37% of 6,266 -- inside this repo's own ±50-cell churn band**, and the logic behind
it is provably identical. This is a placement-relevant reshuffle of equivalent mappings, not a
functional regression.

## Decision

**`soc/netlist_determinism.sh`'s comment class now lands in a large, representative file
(`NETLIST_COMMENT_FILE`, `rtl/csrs.v` for up5k), separately from the dead-net class, which stays at
the top module (`NETLIST_MUTANT`, `rtl/littlesoc.v`) because a submodule's dead wire is optimised
away before the shipping netlist is written -- measured directly: injecting the same tie-off into
`rtl/csrs.v` leaves it absent from the shipping JSON. The same digest-equality and placed-bitstream
assertions cover both mutations at once, so the control fails loudly, for the observation and
regardless of the cause, if either class ever moves the digest again.

**The claim is narrower than ADR-0122 stated it, in the direction this measurement points.**
Digest-equal remains sound -- nothing here found a counterexample, and every case where the digest
stayed equal also placed identically. What is no longer true: digest-different does not imply a
semantic (RTL-meaning) change. It can mean a comment crossed a lexical-rank boundary in a large
file, with the logic unchanged and only which of several equivalent LUT mappings ABC9 picked
different. Either way the placement can move, so the operational answer is unchanged: digest
different, spend the sweep.

`soc/netlist_digest.py`'s header and CLAUDE.md's `netlist-digest` command entry both carry the
caveat and this measurement now, so a reader does not have to find this ADR to learn the digest's
equal-class is narrower than "comment-only edits are always forgiven."

## What was considered and rejected

**Reverting PR #291's comments to shrink the netlist back to 6,266.** Out of scope: the 12 MHz
margin question this movement raises is a requirements decision for its own ticket, and the logic
is unchanged either way -- there is nothing here to "fix" by editing comments.

**Naming the mechanism in the control's own comments or fail messages.** Declined. The control
should be right regardless of which pass turns out to be sensitive; encoding "`abc9_ops`'s sort
order" into the script would make it read like a claim about *why*, when what it asserts is only
*that* the digest stayed equal. The mechanism belongs here and in the digest's dated header, not in
a comment a future refactor of `abc9_ops` would leave stale.

**Combining the comment and dead-net classes in one file, as before.** Impossible for the dead-net
class without losing its own oracle: the tie-off has to live in the top module to survive synthesis
at all, and the file that demonstrates the comment class is a submodule.

## Consequences

- **`make netlist-determinism` and `make netlist-digest` are unaffected on this tree, today**: the
  two-line insertion this control performs on `rtl/csrs.v` does not move the digest further --
  whatever crossed the boundary already landed with PR #291. The control is now *capable* of failing
  for this reason where it structurally could not before; it is not expected to fail on every tree.
- **A future comment-only edit to a large file may legitimately fail `make netlist-determinism`.**
  That is the control working, not a bug in it: read the fail message, spend the sweep, and do not
  read "canonicalisation is broken" into a result this ADR predicts.
- **The toolchain-dependence already stated for `make fit`'s churn band gets sharper**: a change
  this small (0.37%) is not distinguishable from placement noise by area alone, so a digest
  difference of this shape is a signal to sweep, not a signal to investigate the diff.

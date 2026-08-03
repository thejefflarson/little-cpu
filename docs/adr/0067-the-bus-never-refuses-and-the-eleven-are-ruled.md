# ADR-0067: The bus never refuses a transaction, and the eleven declined checks are ruled

**Status:** Accepted · 2026-08-02 · *Pays off
[ADR-0044](0044-what-the-memory-system-has-to-be.md)'s closing obligation — a ruling on each of its
eleven declined ladder checks by name. Adopts its option 1 as a permanent commitment. Reads
`formal/imemcheck.sv` as [ADR-0065](0065-the-ladder-speaks-the-writable-text-bus.md) left it.
Settles invariant 2's open question. No `rtl/` file changed and no gate changed verdict.*

## Context

ADR-0044 ended with a debt, stated precisely: the eventual memory ADR owes a ruling on each of
eleven checks **by name**, with each `#omit` line either deleted, re-tagged `[DESIGN]`, or left
`[BLOCKED]` with what it is still blocked on.

ADR-0054 built the memory system and deliberately did not take that ruling. It was right not to: at
that point the SoC had two disjoint buses, one initialisable ROM, no way to load a program, and no
address map a real product would have. Declaring the bus permanently non-faulting from there would
have been a guess dressed as a decision — the thing ADR-0038 refused to do about the regfile and
ADR-0042 then did properly, with data.

The map is real now. Text is writable and the data bus reaches it (ADR-0059), the stolen fetch
window reaches decode (ADR-0060), and every formal harness speaks the same bus the RTL does
(ADR-0065). The question the ruling turns on — can this system's memory say *no* to an access? — has
a settled answer for the first time.

## Decision 1 — the bus never refuses a transaction, and that is permanent

`rtl/imemory.v` and `rtl/memory.v` answer every address the core can present. Read from the RTL
rather than from intent:

- An **in-range** load or store is served.
- An **out-of-range load** reads zero. `rtl/memory.v` drives `32'b0` when `in_range` is false, and
  `rtl/imemory.v` drives `32'b0` unless the previous cycle was a text-range load. The two are joined
  with an OR (ADR-0059 decision 2), so an address in neither region reads zero.
- An **out-of-range store** is dropped. Every write arm in both files is gated on its own range
  test, and `rtl/memory.v` indexes off a subtracted offset rather than truncated address bits, so a
  dropped store cannot alias a mapped word either. `test/mem_tb.v` checks that at the boundary and
  far away.
- An **out-of-range fetch** reads zero. Zero is an illegal instruction, so a PC that runs off the
  end of text traps in decode instead of wrapping round.

There is no fault line, no handshake and no way to signal refusal on either bus. **This ADR adopts
ADR-0044 option 1 as a commitment, not as a description of what happens to be built.** A future
kernel that wants access faults needs its own ADR, and that ADR has to answer invariant 2, not
merely note it.

Option 2 (faults resolved in decode from a static range decode) and option 3 (change invariant 2)
are both declined. Option 2 buys reachable `ifault` at the cost of a second copy of the memory map
inside `rtl/decoder.v`, on the loop `make soc-timing` reports as critical, for a fault no program in
this repo can raise. Option 3 trades away the property that makes CSR commit precise with no reorder
buffer, and buys nothing until there is protection to enforce.

## Decision 2 — invariant 2 is settled, not merely unpressured

ADR-0044's sharpest paragraph is that invariant 2 and an access fault only coexist because nothing
on this core can raise one — the two propositions were not reconciled, they were *unreached*, and
the memory system would force a ruling.

It is reached now and the ruling is the conservative one: **nothing faults after decode, because
nothing after decode can fault.** That is a property of the memory system as designed, not an
accident of what is not built yet, and the five checks that put the question are declined
permanently rather than parked.

The two words are not interchangeable. "Unpressured" leaves someone free to add a faulting
peripheral and meet the conflict at synthesis time; "settled" says the non-faulting bus is part of
the design, and widening it is an ADR-sized change.

## Decision 3 — the eleven, by name

| check | ruling | why |
|---|---|---|
| `fault_ch0` | `[DESIGN]` | no access fault can be raised |
| `bus_imem_fault_ch0` | `[DESIGN]` | no fault line on the fetch bus |
| `bus_dmem_fault_ch0` | `[DESIGN]` | no fault line on the data bus |
| `bus_dmem_io_read_fault_ch0` | `[DESIGN]` | same, and the fault half is what settles it |
| `bus_dmem_io_write_fault_ch0` | `[DESIGN]` | same |
| `bus_imem_ch0` | `[DESIGN]` | `formal/imemcheck.sv` holds this against the real fetch ports |
| `bus_dmem_ch0` | `[DESIGN]` | `formal/dmemcheck.sv` holds this against the real data port |
| `bus_dmem_io_read_ch0` | `[BLOCKED]` | no MMIO region exists |
| `bus_dmem_io_write_ch0` | `[BLOCKED]` | no MMIO region exists |
| `causal_io_ch0` | `[BLOCKED]` | no MMIO region exists |
| `bus_dmem_io_order_ch0` | `[BLOCKED]` **on the core** | one access is outstanding at a time |

The two `io_*_fault` forms need both a faulting bus and an MMIO region. They are `[DESIGN]` because
one of their two conditions is now permanently unmeetable; an MMIO region would not reopen them.

`bus_dmem_io_order_ch0` is the one whose blocker moved rather than closed. ADR-0044 already said its
real obstacle is the core and not the memory, and its `#omit` line said "no ordering to check" while
its stated need was `RISCV_FORMAL_BUS` and `rvformal_addr_io`. The line now leads with the core:
`rtl/accessor.v` has one access outstanding at a time, so a second one is a pipeline change with
everything the stall protocol would have to say about it, and no memory design touches that.

**Unification unblocks none of the eleven.** That is the correct outcome and not a disappointment.
The whole point of one address space over two memories was to make text reachable from the data bus,
and none of these checks is about reachability — five are about refusal, four about a region this
map does not have, one about ordering the pipeline cannot produce, and two about restating
properties two hand-authored tasks already hold. A change that unblocked some of them would have
been a change that added a fault line or an MMIO window, and neither was in scope.

## Decision 4 — the `imemcheck` argument survives, and it is stronger than ADR-0044's version

ADR-0044 tagged `bus_imem_ch0` and `bus_dmem_ch0` "expect `[DESIGN]` on review", on the argument
that `formal/imemcheck.sv` and `formal/dmemcheck.sv` already hold the property. That argument was
written before ADR-0065 replaced `rvfi_imem_check` with this repo's own shadow-halfword assertion,
and ADR-0065 records a real narrowing: **the retire assertion stops at the first store to the
watched halfword and does not resume.** So the argument had to be re-read rather than inherited.

It holds, and the reason is worth writing down because it points the other way from the obvious
worry. `checks/rvfi_bus_imem_check.sv` at the pin pins a `rand_const` halfword for the whole trace
and **has no write path at all**: it walks the instruction-fetch bus channels and assumes each read
of the watched address returns the fixed value. On a core whose text is writable, that assumption
does not preserve the stored-then-fetched case — it constrains it out of the model entirely, and
silently.

So adopting `bus_imem_ch0` would not recover what `formal/imemcheck.sv` gave up. It never had it.
What this repo's version does instead is model the store explicitly and stop asserting where the
answer is genuinely ambiguous, which is the same coverage stated honestly rather than assumed away.

`bus_dmem_ch0` is the easier half: `checks/rvfi_bus_dmem_check.sv` carries a shadow and a write path,
and it re-derives against `rvfi_bus_*` what `formal/dmemcheck.sv` already derives against the real
`mem_addr`/`mem_wstrb`/`mem_rdata` port. Both would additionally cost driving nine `rvfi_bus_*`
outputs from `rtl/`, which is new instrumentation for `make -C formal nonperturbation` to prove
unread, for a property already held.

### What no check on this ladder asserts

**That a store to text becomes visible to a later fetch.** `formal/imemcheck.sv` assumes it — the
fetch data is a free input its shadow constrains, because that task models no memory — and no other
task on the ladder looks at fetch data against a memory model at all. Adopting either bus check
would not change that.

`test/asm/selfmod.S`, `test/asm/textload.S` and `test/asm/contend.S` (ADR-0063) are the only things
in the tree that test it, under `make test` and `make cosim-suite`. That is a simulation-only
guarantee about the property the whole writable-text theme exists to create, and it should be read
as one.

## The consequence ADR-0008 gave up: address 0 is writable text

`test/asm/sections.lds` puts `.text` at `0x0000_0000` and `_start` is its first bytes.
`rtl/imemory.v` treats every word below `ROM_WORDS` as writable from the data bus. **So a store
through a null pointer overwrites the first instructions of `_start`.**

`rtl/memory.v`'s `BASE` parameter is `0x0001_0000` and its own comment says why: RAM sits at a
non-zero base so a null store lands outside it rather than silently on real data. That protection
still holds for RAM and is now beside the point, because the null store lands on text instead.

This is inherent to the theme and not avoidable within it. Making text unreachable from the data bus
is exactly what ADR-0059 undid; moving text off zero costs the reset vector, the linker script and
every `OBSERVED_FLOOR` line, for a guard against one bug class in a repo with no memory protection of
any kind. It is recorded here so it is found in a decision record rather than on hardware, and so
the eventual protection ADR starts from a known address rather than a discovery.

Two things follow that are cheap to state and worth stating:

- The failure is **loud, not silent**, in the common case. A store of zero over `_start` leaves an
  illegal instruction, which traps to `mtvec`, and `test/run_tests.sh` already labels a trap
  redirected to address 0 as `TRAP-TO-ZERO` (ADR-0029) rather than letting it time out.
- It arrived with ADR-0059, not here. Before text was reachable from the data bus the same store was
  dropped by an address decode that no longer exists, and nothing in `test/asm` does it either way.

## What did not change

**Nothing generated, and nothing ran differently.** The `#omit` lines are read by
`formal/genchecks-audit.py`, which captures only the check name; `genchecks`' own cfg parser drops
every `#` line before it sees a section. The generated ladder was captured before the edit and
regenerated after it: **`diff -r` over the two directories is empty — 85 `.sby` files plus the
emitted makefile, byte-identical.** This change alters reasons and class tags and nothing else.

The declined count is derived rather than asserted. `formal/genchecks-audit.py` reports
`85 generated, 14 declined for want of a [depth] line` and set-equalities that set against the
`#omit` declarations in both directions, so the eleven above plus the three that were already
`[DESIGN]` are counted by the generator, not by this ADR.

The gate that could have moved was run rather than reasoned about. Machine: 10 cores, two sibling
agents live, load 13–43 across the run, `JOBS=4`, so the wall time is not comparable with a quiet
box.

| gate | result |
|---|---|
| `make -C formal check JOBS=4` | **85 checks, 85 pass**, 11m39s; both set equalities exact in both directions |
| `formal/EXPECTED_FAIL` | empty, matched by name and status |
| `formal/checks/*.sby` before vs after | byte-identical, all 85, compared twice |

The `#omit` region is a graded comparison, so both of its red directions were forced rather than
assumed. Deleting the `fault_ch0` line gives exit 1 and
`in dropped but not #omit: fault_ch0`; adding a line for a check that *is* generated gives exit 1
and `in #omit but not dropped: insn_add_ch0`. Both took about a second, and `checks.cfg` was
restored and re-run green after each.

## Consequences

- **Every `#omit` line now reads as a decision or as a burn-down item, and the two are
  distinguishable.** That was the whole reason the `[BLOCKED]`/`[DESIGN]` tags exist; until now
  eleven of the fourteen carried the tag that means "come back to this" and nobody could tell which
  ones ever would.
- **The remaining burn-down is four checks, and it turns on two things.** An MMIO region and a
  second outstanding access are both out of scope and both need their own ADR, so the honest reading
  of `checks.cfg` is that the declined set is now stable rather than shrinking.
- **A pin bump that adds a check still fails generation until someone rules on it.** That mechanism
  is untouched, and it is what keeps this list from going stale the way ADR-0044's did.
- **`formal/EXPECTED_CHECKS`, `formal/EXPECTED_FAIL` and every depth line are untouched.** The
  ladder is 85 checks, 85 pass, both set equalities exact in both directions, re-run on this branch.
- **The next memory-shaped change inherits a contract instead of a question.** SPI flash boot, a
  bigger text region, a bootloader — each of them has to keep the bus non-faulting or replace this
  ADR, and the second option now has a name and a cost rather than being an open option nobody
  chose.

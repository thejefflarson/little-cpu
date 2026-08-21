# 0125 — The dual top runs, and the grant is permission to publish

Status: accepted

## Context

Four pieces landed separately and none of them was a machine: a bus arbiter proved over its own
whole state with `request` and `mem_lock` free inputs; the core's shared-bus surface tied off in
every integrator; one text storage with a fetch window per hart and a timer with an `mtimecmp` per
hart; and `mhartid` as a parameter. Assembling them needed one signal nobody had built and one
protocol nobody had written down.

**The signal.** The arbiter's `request` is free in its own proof, so nothing was owed formally, but
a top has to drive it and the core published nothing that could. The loose end was named when the
surface landed and this is the first thing to need it.

**The protocol.** The grant is registered — that is what makes the arbiter two flops — so it has to
be asked a cycle before the cycle it answers. Which cycle it answers was never stated.

## Decision

**`bus_request` is a new output of `rtl/littlecpu.v`, driven from decode.** It is the decoder's own
stall conjunction with `bus_wait` left out of it, ANDed with the nine encodings that reach the data
bus. `bus_wait` is deliberately not a term: the platform ANDs the two, and a term here would close
the loop through the arbiter.

The alternative was deriving it in the top from ports that already exist, and it does not work.
Everything on the boundary that says "this hart is using the bus" — `mem_ren`, `mem_wstrb`,
`mem_lock` — is a *transaction*, which is a cycle later than the arbiter needs and, worse, is
suppressed on exactly the cycles a waiting hart is waiting. A hart that publishes nothing while it
waits asks for nothing while it waits, and two harts that both stop asking are never granted
anything again. The request has to precede the transaction and survive the wait, and only decode has
both properties.

**THE GRANT IS PERMISSION TO PUBLISH, NOT OWNERSHIP OF THE BUS.** A hart's transaction goes out one
cycle after decode publishes the instruction, from the decoder's `out`. So the invariant the top
maintains is

> at most one hart publishes a memory instruction per cycle

which gives at most one transaction one cycle later. `grant` is one-hot-or-zero and
`bus_wait[h] = bus_request[h] && !grant[h]` raises the wait for every hart that asked and lost, so
that much follows from the arbiter alone and needs nothing else.

**An AMO needs the second term, and `mem_lock` arrives exactly on time for it and one cycle late for
the arbiter.** An AMO publishes once and puts two transactions on the bus. Nothing anywhere may
publish on the cycle its read is out — its own decode is already spending that cycle, and
`mem_lock` is high on exactly it. So the lock goes into the *other hart's wait*:

    bus_wait[h] = bus_request[h] && (!grant[h] || mem_lock[OTHER])

The lock also reaches the arbiter, where it holds the grant; there it covers the cycle after the one
this term covers, for the same reason the grant is registered. Both readings are true and they are
the same fact from two ends — the lock says "the bus is busy next cycle too", the grant says "you
may publish".

**`rtl/littledual.v` is the complex and `rtl/littledualsoc.v` is the pinned top**, split so that the
harness instantiates the same arbitration the placer places, with a bigger ROM, no init files and
the two resets driven apart. Two harts is a localparam and not a parameter: the arbiter is two
request bits, two lock bits and a two-bit grant, and its wait bound is proved for exactly that.

**`rtl/memory.v` takes `NHARTS` for the atomic range test and for nothing else.** The bus-side ports
stay scalar because the bus is shared and one master drives it per cycle; that one pair is asked in
*decode*, where every hart is asking about its own instruction at the same time.

**The dual harness and its runner are separate files, not a configuration axis.** `test/cxxrtl.cc`
is a merge gate: it must not grow a flag, must not get slower, and reads every signal by flat
debug-item name — two of everything has two hierarchical names. Two monitor instances roughly double
a 7000-line generated module, so nothing dual is on `make test`'s path.

**`test/crt0.S` branches on `mhartid` before touching a register the C ABI cares about or a word of
memory**, and every hart but the first parks polling a RAM mailbox — this core decodes no `wfi`.
The mailbox is one nothing writes, deliberately: a release protocol needs somewhere to release a
hart *to*, and the second hart's stack, its `.data` and its entry point are decisions no program
here has made.

## What was measured

### The port is not free, and it is smaller than the standing estimate

The prior for an unread output on `rtl/littlecpu.v` is **+44 `SB_LUT4`** on the SoC. This change,
which is that port plus `rtl/memory.v`'s `NHARTS`, measures on one tree with one toolchain:

| | base | with the change |
|---|---|---|
| `littlesoc`, `SB_LUT4` | 4219 | **4239** (+20) |
| `make fit`, packed `ICESTORM_LC` | 3929 | **3916** (−13) |

Both are inside the churn band, and the two tops disagree in sign, which is the band's own
signature. The decomposition was not isolated and is not claimed: an inert generate loop measured
−18 on this same top when the fetch windows landed, and `rtl/memory.v` gains one here, so the two
halves plausibly cancel. **What is claimed is the total, on this tree, with this toolchain.**

The digest moved, so the paired sixteen-seed sweep is owed and has not been spent. `make fit` and
`make soc-timing` at their defaults are green.

### The dual configuration places on ECP5

`littledualsoc` on the LFE5U-25F-6CABGA381, one placement, constrained at the pinned 200 MHz that
the design is meant to miss:

| | |
|---|---|
| Fmax | **35.36 MHz** |
| `DP16KD` | 40 |
| `TRELLIS_DPR16X4` | 64 |
| `MULT18X18D` | 8 |
| `TRELLIS_COMB` | 10016 of 24288 (41.2%) |

One placement is a sample and there is no ratchet here. **The three censuses are the result worth
having**: they were declared before the run and all three matched first time. 40 is 32 for the one
data RAM plus 4 for *each* of two ROM copies, and the LUT RAM and DSP counts are exactly twice the
single-hart ones — which is what says two fetch windows are two copies of one storage and two harts
are two whole cores. **No up5k number appears above and none may be subtracted from these.**

### `test/monitor.sim.v` really does work unmodified, and that was a prediction

Two `monitor` instances and two `monitor_isa_spec` instances read the sanitized oracle with no
change to it and no change to `test/sanitize_monitor.py`. The file has no hierarchical reference in
it and its per-retire spec model is a function of one hart's `rvfi_*` stream alone. It does not
predict load data, so a load that observes the other hart's store is not something it has an opinion
about. Measured on the smoke run: hart 0 spec-checked 114 of 115 retires, hart 1 139 of 140 — the
same ratio the single-hart suite reports, with the shortfall being the instructions riscv-formal
ships no model for.

### There are TWO ROM banks in this RTL and not four

The dual harness zeroes every word of both banks before poking its program, for the reason the
single-hart one does. But `rtl/imemory.v` holds **one** storage however many windows read it: the
copies two windows need are made by the mapper, and no simulation of this RTL can see them or fail
on their absence. The place that grades the replication is the mapped netlist, and the ECP5 census
above is where it shows.

### One bus port cannot be ORed, and the reason generalises

Three of the four shared-bus ports join with an OR the way the read bus does, because
`rtl/accessor.v` drives them to zero on a cycle it is not requesting. **`mem_wdata` is not one of
them.** That file publishes rs2 on it for *every issuing instruction* and not only for a store,
because with one bus master `mem_wstrb` is the only gate that matters and nothing there ever had to
drive it to zero.

ORed across two masters, one hart's rs2 becomes part of the other hart's store on any cycle a
non-memory instruction issues beside one. This is measured, not reasoned about: it lost **30 of the
smoke program's 32 counted increments** — hart 1's counter finished at −14 — and the program still
ran to completion on both harts with every other check green. It is also invisible to a
bus-exclusivity check, because the hart doing the damage has neither a read enable nor a strobe
raised. The port is muxed on the write strobe instead, and the general form is the finding: **a
signal a single-master design never had to drive to zero is not a signal two masters may OR.**

### The harness has its own red direction and it fires

`make dual-smoke` builds one program and runs it twice. With both harts the shared count is 32 and
both harts retire; with hart 1 held in reset the count is 16, hart 1 retires nothing, and the
per-hart silence gate exits 6. A dual harness that measures one hart looks exactly like a working
one unless the answer depends on the second hart having run, so the program is written so that it
does and both directions are graded on every run.

## Consequences

`G` is measured on the single-hart machine and the depths are derived from it. **Nothing in
`formal/` builds two cores, so no generated check covers this top and none of its depths apply to
it.** `formal/MULTIHART_TIE_OFF` already says so and still holds: `bus_request` stays out of that
baseline for the reason `mem_lock` does — an unread output cannot weaken a check — and the file's
own sweep agrees that no other input is held at a constant everywhere.

The arbiter's environment assumption is **still not discharged in this tree**. The core's
`mem_lock` is asserted by `components_accessor` to last exactly one cycle, which is the assumption's
content, but that proof is about one core and the arbiter's proof is about the module alone; nothing
composes them. Reading them together is an argument, not a proof.

Still open and deliberately out of scope here: the torture programs and their mutation pairings, a
`DUAL_MIN_MHZ` gate and the ECP5 seed spread this design would need before one could exist, a
hermetic probe of `test/dual_smoke.sh`'s own five comparisons, and any release protocol that lets a
parked hart run C.

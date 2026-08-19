# ADR-0126: The shared text is one storage, and what the dual milestone does not claim

**Status:** Accepted · 2026-08-19 · *Records the coherence argument for text two harts fetch from,
the software protocol the cross-patch program executes, and — at length, because this is the half
that gets lost — everything the dual-core milestone will not have established when its programs go
green. Adds four programs and a build check; changes no RTL, so no instrument reads anything.*

## Context

Two harts fetching one program raises a question the single-hart design never had to answer: when
one hart writes an instruction, what makes the other hart's fetch see it? The usual answer is a
coherence protocol. This machine's answer is that there is nothing to make coherent, and it is worth
writing down why, because the argument is short and the shape of it decides what the torture
programs can and cannot claim.

The second half of this record is the harder one. A milestone declared off green programs invites
the reading that the thing is verified, and this project has an explicit rule against that reading:
an empty baseline is necessary and not sufficient. Cross-hart execution is where that rule bites
hardest, because the oracle stack this repo has built up over a hundred decisions is, at the pinned
tools, **single-hart in every leg**.

## Decision 1 — text is one storage, so coherence is not a protocol here

`rtl/imemory.v` publishes one fetch window per hart out of replicated banks driven by a **common
write**: the same enable, the same address, the same data, the same edge. The copies differ only on
their read port. Coherence is the property of keeping *N* storages agreeing about one address;
there is one storage, wired *N* ways, so the property is structural and there is no protocol, no
directory, no invalidation traffic and nothing to get wrong at run time.

What that structure does need is the fetch stall, and it needs it for a reason that is not about
coherence at all: **both parts define a same-address write-during-read as returning invalid data on
the reading port**, not the old word. Zifencei permits an instruction fetch concurrent with a store
to that address to see the old instruction or the new one, and never garbage. So a write stalls
**every** window's fetch, not only the writing hart's. That is already how it is built, and
`fetch_stall` is already a per-core input with settled hold rules, so it adds no stall reason.

**The structural claim has a structural grader and it exists**: the mapped block RAM cells are
grouped by the nets on their write port and must come out as one group per copy-pair on a common
write, differing on their read port — forced red against a mutant whose second window reads banks
the write does not reach. That check is about the netlist. `test/dual/crosspatch.S` is the software
half of the same claim and **has not run**.

## Decision 2 — the cross-patch program executes the specification's protocol and nothing else

Zifencei is explicit that `fence.i` orders the *local* hart's fetch against the local hart's stores,
and says what to do across harts: the writing hart executes a data `FENCE`, then asks the remote
harts to execute `FENCE.I`. `test/dual/crosspatch.S` is that sequence and no other:

1. hart 0 calls the stub and observes the **old** instruction, then sets a flag — so an image that
   already held the patched word is caught rather than passed;
2. hart 1 waits for the flag, writes the new word, executes `fence`, sets a second flag;
3. hart 0 waits for that, executes **its own** `fence.i`, and calls the stub again.

The data fence orders nothing extra on this machine — one bus, no store buffer, one outstanding
access per hart — and the program executes it because the protocol is the specification's, not this
machine's. A machine that acquired a store buffer would need it and would not gain a program.

**Which hart patches is deliberate.** Hart 1 does, and it reads the replacement encoding out of
`.text` with an ordinary load. A load is answered from one window's read port, so a load from the
hart that does not own that window has to steal it; putting the template load on hart 1 is what puts
that steal on the program's path.

## Decision 3 — `fence.i`'s serialization wait is still unobservable, and this program does not change that

ADR-0105 measured that no program in the suite can see `fence.i`'s half of the serialization
commitment: a text store takes the instruction memory's write cycle, so the earliest fetch the wait
could still be covering is one the memory already holds back, and deleting `instr_fencei` from
`serialize` leaves the whole suite green.

**That finding survives two harts unchanged, and the reason is the same one.** The write reaches
every copy on one edge and stalls every window, so on both harts the earliest fetch the wait could
cover is one the memory already held back. `test/dual/crosspatch.S` exercises the software protocol
and the storage being one storage; it is **not** a grader for the wait, and must not be read as one.
`test/decoder_tb.v` remains the only grader, and the pairing that deletes both mechanisms together
remains the only one a program sees.

## Decision 4 — the milestone's claim, stated with its boundary

When the four programs in `test/dual/` run green, this is what will and will not have been
established.

### There is no independent oracle for cross-hart behaviour at the pinned tools

Every leg of the three-leg stack is single-hart where it matters, and the A extension was already
the thinnest part of it:

- **Sail** is configured here as a single-hart emulator, and `test/cosim.py` drives it that way.
  There is no interleaving for it to reproduce and no second register file for it to hold. It is
  already baselined `INCONCLUSIVE SAIL-LIMIT` for the one interrupt this core takes; cross-hart
  execution is a second such class and a larger one.
- **`test/cosim.cc`** reads one core's `regs_a`. That is deliberate and is exactly the property that
  lets it catch architectural writes the self-reporting oracles structurally miss — and it **never
  compares memory**, which is why every program in `test/dual/` reads its memory result back into a
  register.
- **riscv-formal** describes one hart. It ships no spec model for any AMO encoding at the pinned SHA,
  so **the eleven A instructions already lacked an upstream oracle single-hart** — their semantics
  are checked against assertions this repo wrote. Two harts do not weaken that; they widen what it
  fails to cover, because the object to compare is now an interleaving and there is nothing to
  compare it to.

So every claim these four programs make is **an in-band assertion this repo wrote**, graded by the
program against itself.

### The dual configuration has no generated riscv-formal checks, and its G is unmeasured

Every generated check is `mode bmc`, so a PASS means no counterexample within that check's depth.
The depths are derived from two measured numbers, F and G, and the derivation is machine-checked
against the figures `formal/checks.cfg` declares. **Both numbers describe the single-hart core.**

A hart that loses arbitration retires later than one that does not, so the dual machine's worst-case
retire gap is at least the single-hart G plus the arbiter's wait bound for that hart — 2 cycles for
hart 0 and 3 for hart 1 — and plausibly more, since the reasons compose. **Nothing has measured it**,
no check has been generated against it, and the standing rule that a change adding a stall reason
must re-measure F and G before it lands applies to the dual top when the dual top exists. A depth
carried over from the single-hart table would go green having stopped asking, which is the exact
failure the machine-checked derivation exists to prevent.

### A floor line for one of these programs would be a retire count

`test/monitor.sim.v` value-checks nothing in an A retire, because the pin ships no spec model for any
of the eleven. That is already recorded for the single-hart A programs and it is unchanged here: when
these programs are wired into a runner and given floor lines, **those numbers say the monitor did not
go quiet and say nothing about anything being compared.**

### Starvation is caught by the arbiter's proof alone

No torture program will see it, and `test/dual/MUTATION_PAIRINGS` says so on the line rather than
leaving the list to imply otherwise. Starvation is not a wrong answer; it is an answer that does not
arrive. A fixed-priority arbiter passes every program in that directory whose harts contend lightly,
and hangs the ones whose harts contend hard — where the bounded waits in `test/dual/dual_test.h` turn
it into "hart 1 never got there", which is not the same report as "the arbiter is unfair". The
wait-bound proof is a safety property with a constant per hart, each covered as well as asserted.

### The mutation pairings are predictions

`test/MUTATION_DETECTORS`' own instruction is *measure it, do not assert it*, and every line in it
was produced by applying a patch and writing down what went red. **Not one line in
`test/dual/MUTATION_PAIRINGS` has been through that**, because nothing in this tree instantiates two
harts. The file says so at the top, states the three steps that turn it into a graded set, and names
the two outcomes a prediction that fails to reproduce has: a gap to close with a new program, or a
fact about the machine to record.

### None of the four programs has executed

`make dual-build` assembles and links each one, checks the pairings against the directory in both
directions, and prints that nothing has run. That is the whole of what is graded about them today.

### The frequency requirement is a requirement, and its band does not exist

`DUAL_MIN_MHZ` is 25.0 — the Colorlight i5's oscillator — graded worst-of-sixteen on nextpnr's own
report, on the same ground `SOC_MIN_MHZ` stands on: a board clock is not a threshold to relax. **The
ECP5 edit-churn band and placement spread it would be read against have not been derived** — that
work is separate and partial — so what is uncertain is the instrument and not the number. The answer
to an uncertain instrument is a pinned pessimistic corner, which the ECP5 flow already declares and
grades off the configuration Trellis emits, rather than a softened floor.

## What the four programs do claim

Each is written so its failure is a value and not a hang, because a hang is indistinguishable from a
harness that never released the second hart. Every wait is bounded and fails at a named test number.

| program | the property | the shape of the failure |
|---|---|---|
| `amocount.S` | two harts add 1 to one word N times each; the word ends at exactly 2N **and** the values the AMOs returned sum to N(2N−1) | a lost update short-counts; a duplicated return keeps the count and breaks the sum |
| `lockwitness.S` | an LR/SC lock excludes; the counter incremented **non-atomically** inside it ends at 2N | both harts inside at once lose updates, and the occupancy flag fails at the breach |
| `snoopcross.S` | one hart's write ends the other's reservation, with an ordered case, a negative control, and a **cycle sweep** | the ordered case catches a missing snoop; only the sweep catches a snoop one cycle late |
| `crosspatch.S` | hart 1 patches an instruction, hart 0 fences and runs it | private text, or a write reaching one copy, returns the old word |

**The sweep is the one to understand.** A snoop that arrives one cycle late passes the ordered case
with tens of cycles to spare, and goes wrong only when the store-conditional takes the bus on the
cycle after the foreign write took it. So hart 0 hands over and then delays a computed number of
cycles before its `sc.w`, walking the attempt one cycle at a time across hart 1's write. What it
asserts is not who won — either order is legal — but that **hart 1's value is what survives**, which
is false exactly when a store-conditional stores after a write it should have been told about. And
the sweep grades itself: it requires at least one round each side of the crossing, so a range that
has drifted off the case goes red instead of passing having covered nothing.

## Consequences

- The claim to make when these go green is "two harts share memory and text without losing an update
  in the cases these four programs exercise", and not "cross-hart execution is verified". The
  distinction is the whole of this ADR.
- **Wiring the programs into a dual runner is owed**, with a floor file, a cycle budget large enough
  for `snoopcross.S`'s thirty-four rounds, and a re-measurement of that program's sweep range against
  a real waveform. Its range and its two coverage counts are sized from the handshake's shape, not
  from an observation.
- **Writing the mutation patches and measuring the pairings is owed**, and is the step that turns
  `test/dual/MUTATION_PAIRINGS` from a design record into a graded set.
- **F and G for the dual configuration are owed** before any generated check describes it.
- The `.aq`/`.rl` derivation for two harts is ADR-0125's and is not repeated here.

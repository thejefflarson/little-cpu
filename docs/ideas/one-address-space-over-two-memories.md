# One address space over two memories

**Status:** brief, ready to plan · 2026-08-02 · follows ADR-0054

Make instruction memory writable and put it in the same address space as data, so a
kernel can load and run programs — and so a trap handler can read the instruction that
faulted.

## Why

Three problems, all the same absence: there is no path from a store to something the
fetcher can later see.

1. Program text lives in a ROM built into the bitstream. Nothing can be loaded at runtime.
2. The trap handler cannot read the faulting instruction. That is why every trapping
   instruction in a test has to be wrapped in `.option norvc`, and that rule is what hid
   the `c.ebreak` bug in ADR-0048 — a blind spot exactly one instruction wide.
3. ADR-0054 records that the SoC cannot run a program that reads its own `.data`. The
   SPRAM is not initialised by the bitstream and nothing copies an image into it.

ADR-0008 picked separate address spaces because the RTL already had separate buses. Its
own rationale says so. It was inherited, not chosen.

## The idea

**Do not merge the memories. Merge the address space.**

Every benefit above needs one address space. None of them needs one memory. The two
physical memories stay exactly where ADR-0054 put them.

The enabling fact: an ice40 EBR has **one read port and one write port**, and the ROM
banks only use the read side. The write port needed to make text writable is already
there, unused.

```
0x0000_0000  ┌──────────────────────────┐
             │ BRAM  8 KB               │  text: readable, writable, executable
             │ 2 word-interleaved banks │  initialised from the bitstream
0x0000_2000  ├──────────────────────────┤
             │ SPRAM  64 KB             │  data only
             │ read/write, not executable│
0x0001_0000  └──────────────────────────┘
```

Fetch reaches only the BRAM region. A PC in SPRAM fetches zero, which decodes as an
illegal instruction — the out-of-range behaviour that already exists.

## Arbitration

A data access that lands in the text region uses the BRAM banks. Fetch is using them too.
Something has to give.

**Data always wins.** It belongs to an instruction that already issued, and the accessor
issues a store exactly once. A fetch is repeatable — the PC holds and asks again next
cycle. This is a ruling, not a tuning knob.

Two pieces of hardware:

- **`mem_ren`**, a new core output: `!reset && in_valid && is_load`. Needed because the
  idle bus presents `mem_addr = 0`, which is inside the text region. Without a read
  enable, every idle cycle would steal a fetch and the core would never run.
- **`fetch_stall`**, a new core input, registered from
  `(mem_ren || |mem_wstrb) && text_range(mem_addr)`.

A store steals the fetch edge as well as taking the write port, on purpose: a fetch read
and a store write of the same word should never meet on one edge.

`fetch_stall` is the **sixth stall reason** on the existing two mechanisms. It bubbles,
like `operand_stall` — `pc` holds, `out <= '0`. **No flush.** Nothing was issued, because
`issuing = !reset && !stall` already gates trap commit, CSR writes and `minstret`, so a
stolen window cannot issue or trap.

SPRAM accesses steal nothing. Different memory, different ports. That is most of why this
is nearly free.

## `fence.i` has to change

This is the one semantic change, and it is a real bug rather than a tidy-up.

The fetch address is published one cycle early (ADR-0054). So a store to text at cycle D
writes the array at edge D+2, but the instruction after a `fence.i` at D+1 was fetched at
edge D+1 — before the write. It would execute stale text after `fence.i` retired. That
breaks Zifencei.

Today's NOP is only correct because separate buses make the store impossible. ADR-0002's
"correct and for free" was a property of the split, not of the core.

Fix: `fence.i` joins the serialization drain that CSR instructions already use. One OR
term on the existing predicate. Costs 3-4 cycles when executed, which is nothing.

Instructions *between* a text store and `fence.i` may see old or new text. That is legal.
They cannot see a torn value, because the steal forbids a same-edge read and write.

## Cost

**Zero on the current suite.** No program does a text-region data access today — separate
buses make it impossible, which is the whole point.

Each future text access costs 2 cycles: one steal bubble, and one `operand_stall` refetch
because the garbage window perturbs `prev_rs1`/`prev_rs2`. That is per trap handled or per
word loaded. Invisible in aggregate.

For reference, the measured issue rate on the current design. Every instruction passes
through all four registered stages — decode, execute, access, writeback — so the *latency*
of any one instruction is about 4 cycles. The stages overlap, so what sets runtime is how
often decode can start a new instruction, and that is what this table measures.

| instruction | cycles between issues |
|---|---|
| `lui`, `auipc`, `jal` | 1 |
| ALU, branches, `jalr`, stores | 2 |
| `mul` family | 2 |
| loads | 3 |
| `div`, `rem` family | ~34 |
| `csr*`, `mret` | 2 + drain |

The limiter is decode, not the stage count. `operand_stall` makes it present the register
addresses one cycle and issue the next, so it emits at most one instruction every two
cycles. `lui`/`auipc`/`jal` use no registers, skip the present cycle, and issue back to
back — which is why whole-program CPI comes in under 2.

Whole-program CPI runs 1.78 (`beq.S`) to 4.57 (`div.S`), baseline around 1.8. Almost all
of that baseline is `operand_stall` — the synchronous regfile from ADR-0042. This proposal
does not move any of it.

The rejected alternative — one physically merged BRAM pool — costs +2.0% measured on the
suite and 8-15% on compiled code, and throws away 64 KB of SPRAM to get there.

## The `.data` gap closes for free

Link `.data`'s load address into the BRAM image and have crt0 copy it to SPRAM through the
new text-read path. No new hardware. This is the ADR-0054 gap that would otherwise need a
flash controller.

## Invariant 2 and the formal ladder

**The bus stays non-faulting, permanently.** Every in-range access is answered. Out-of-range
loads read zero, out-of-range writes are dropped, out-of-range fetch reads zero and traps
as an illegal instruction in decode. No access fault exists, so nothing faults after
decode. This adopts ADR-0044 option 1 as a commitment. A kernel that wants access faults
needs its own ADR.

That settles the eleven declined checks ADR-0054 owed ADR-0044:

- Five fault checks (`fault_ch0`, `bus_imem_fault_ch0`, `bus_dmem_fault_ch0`,
  `bus_dmem_io_read_fault_ch0`, `bus_dmem_io_write_fault_ch0`) become `[DESIGN]`.
- `bus_imem_ch0` and `bus_dmem_ch0` become `[DESIGN]`, on ADR-0044's existing argument
  that `imemcheck`/`dmemcheck` already hold the property.
- Four io/ordering checks stay `[BLOCKED]` — still no MMIO region, and `io_order`'s real
  blocker is that only one access is ever outstanding.

**Unification unblocks none of them.** That is the right answer, not a disappointment.

What does change:

- `formal/wrapper.v`'s assumption that imem is a function of its address gains an
  exception for the cycle after a text store. `fetch_stall` must be constrained to the
  arbiter's real equation, never left free — a free stall input starves `hang` and
  `liveness`, which is what ADR-0042 found.
- `formal/imemcheck.sv` needs the write path, so it observes `mem_addr`/`mem_wstrb`/
  `mem_wdata`. Assuming store-free traces instead would be a weakening; don't.
- `dmemcheck` should be unaffected. Verify rather than assume.
- **F and G must be re-measured before anything lands.** A sixth stall reason moves the
  worst-case first retire and retire gap that every ladder depth was derived from
  (ADR-0046), and `insn 15` and `reg 15 20` clear their floors by one cycle. M2 term 1
  says re-measure. This is step 1, not a follow-up.

## Tests

- `sections.lds`: regions unchanged. For the hardware flow only, `.data` gains a load
  address in the text region and `riscv_test.h` gains a crt0 copy loop.
- Both sim runners keep loading two images, so the 56 programs and `OBSERVED_FLOOR` do not
  move. Its floors are retire counts; CPI does not touch them.
- `test/sail/rv32imc_zicsr.json`: the text region must be readable and writable to the
  data bus in the reference model, or Sail faults where the core does not — a false
  divergence of the ADR-0043 kind.
- New programs, each with an `OBSERVED_FLOOR` line: `selfmod.S` (store, `fence.i`,
  execute, including the D+2 case above), `textload.S` (handler reads the encoding at
  `mepc`, works out 2 vs 4 bytes, resumes past a **compressed** trapping instruction), and
  a contention test.
- `test/imem_tb.v` extends for the write port; the arbiter gets a bench. `UNIT_BENCHES`
  goes 7 → 8.

## Risks

**This section is measured now, and two of its estimates were wrong** — see
[ADR-0057](../adr/0056-what-writable-text-costs-in-ladder-depth-and-in-nanoseconds.md). The steal mux
costs 8.5% of the critical path, not 2-3%; `SOC_MIN_MHZ` is 10.0 rather than the 10.5 quoted below
and still holds; and the load-free fallback offered here is the *slower* half of the change, because
the write port is where both the area and the delay go. F and G were re-measured too: F is unchanged
at 6, G goes 4 to 6, and three `[depth]` lines have to move.

**Timing is the real one.** The steal mux sits on the bank address input, at the tail of
the 88.51 ns fetch → decode → `next_pc` loop, where each logic level costs about 2-2.5 ns
of local interconnect. Estimate one added level, 2-3%. `SOC_MIN_MHZ = 10.5` should hold,
but ADR-0054 measured a 3.6% churn band on `make soc-timing`, so only a run on the branch
settles it. If it does not hold, the fallback is decoding the steal from the accessor's
registered inputs a cycle early — more state, and its own ADR.

**Ladder depth** may grow with the steal cycles reachable inside each check's window.
Unknown until F and G are re-run.

**8 KB is a tight ceiling** for a kernel plus a loaded program. Widening text to 12 KB
costs 24 EBRs and 203 LUT4s (ADR-0054 measured it). Recorded, not taken.

## Sequence

1. Re-measure F and G with a modelled `fetch_stall`; confirm the depth budget. Gating.
2. `rtl/imemory.v` gains the write port, the data-read path and the steal output. SoC and
   testbench gain the range decode and `mem_rdata` mux. `imem_tb` extends; arbiter bench.
3. Core: `mem_ren` out, `fetch_stall` in, `fence.i` serialization. `pcloop` and the
   decoder's `FORMAL` block updated in the same change.
4. Formal: wrapper assumption, `imemcheck` write path, the eleven `#omit` rulings,
   re-derived depths, full ladder.
5. Tests: the three new programs, Sail config, crt0 and the load address for hardware.
   `make soc-timing` re-measured, quoted with its toolchain.

## Not in scope

- SPI flash boot and a bigger text region in SPRAM. The hard IP SPI core makes this
  cheaper than ADR-0044 assumed, and up5k_riscv is a working reference, but it is new
  unverified surface and M2 is still open. Every arbitration rule here carries over
  unchanged, so this is the rehearsal.
- Any MMIO region, and with it the four blocked io checks.
- Protection of any kind.
- Recovering the second stall cycle with shadow fetch-hold registers. That is the first
  half of a fetch buffer and it saves a cycle nobody will notice.

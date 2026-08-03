# ADR-0063: The suite runs programs that use writable text

**Status:** Accepted · 2026-08-02 · *Builds step 5 of
[`docs/ideas/one-address-space-over-two-memories.md`](../ideas/one-address-space-over-two-memories.md),
minus its `.data` half. Discharges
[ADR-0061](0061-fence-i-has-to-serialize.md)'s "argued fix with a unit test" and
[ADR-0060](0060-the-steal-reaches-decode-as-the-sixth-stall-reason.md)'s "the suite does not yet
exercise this change end to end". Closes the blind spot
[ADR-0048](0048-what-an-independent-read-of-the-no-oracle-rtl-found.md) named.*

## Context

ADR-0059 made text writable, ADR-0060 routed the stolen fetch window into decode as the sixth stall
reason, and ADR-0061 made `fence.i` serialize. Both of the latter two recorded the same gap: no
program in `test/asm` does a text-region data access, so every retire count in the suite was
identical program by program and the whole feature was reachable from nothing the merge gate runs.
ADR-0061 was blunter still — what covers its decision is a decoder bench and a component proof, and
what would cover the *behaviour* is a program that stores to text, fences, and executes the result.

Separately, the shared trap handler in `test/asm/riscv_test.h` resumes at `mepc + 4` and cannot read
the faulting instruction, so every trapping instruction in a resuming test is wrapped in
`.option norvc`. That discipline is what hid the `c.ebreak` cause defect ADR-0048 found.

## Decision 1 — three programs, and the suite goes 56 to 59

- **`test/asm/selfmod.S`** stores into `.text`, fences, and runs the stored word. Three cases: the
  instruction immediately after `fence.i`, a stub patched and then called, and the original word
  written back and the stub called again — the third so a ROM image that already held the patched
  word could not pass the second.
- **`test/asm/textload.S`** carries its own handler, which reads the faulting instruction out of
  `.text`, decides two bytes or four from its low two bits, and resumes past it. None of its four
  trapping instructions is wrapped in `.option norvc`, and two of them are `c.ebreak`.
- **`test/asm/contend.S`** puts loads and stores in `.text` at both word-address parities, with byte
  and halfword strobes, a store and a load of the same word back to back, and a text load
  immediately behind a RAM store.

Each has a `test/OBSERVED_FLOOR` line, so `test/check_suite_shape.sh` matches in both directions
from both runners. `make test` is 59/59 and `make cosim-suite` is 59/59, both baselines empty.

Every text address any of them touches is under `0x200`, far inside the 8 KB the SoC's ROM covers.
Past that boundary the simulation's 16 KB ROM and the part's 8 KB one disagree about which addresses
are text at all, and a program that straddled it would mean different things on the two legs.

## Decision 2 — the store before `fence.i` uses x0 as its base, and that is what makes the test able to fail

This is the finding, and it is the reason this change has an ADR rather than being three test files.

`operand_stall` makes decode present rs1/rs2 one cycle and issue the next, and `fence.i` reads
neither register — but `uses_rs1` is true for it, so it still waits for its own rs1 field, which is
zero, to have been presented. It can therefore only issue one cycle behind a store that presented
`rs1 = x0` as well. Behind `la t0, site; sw t1, 0(t0)` it issues one cycle later than that, by which
time the store's write edge has already passed, and the stale fetch ADR-0061 describes is
unreachable.

Measured, with `instr_fencei` dropped from `rtl/decoder.v`'s serialization term:

| test 2's store | verdict |
|---|---|
| `la t0, patch_adjacent; sw t1, 0(t0)` | **PASS**, 49 retires |
| `sw t1, %lo(patch_adjacent)(x0)` | **FAIL 2**, 17 retires |

The first is the form anyone would write and it checks nothing. The second is what ships, with the
reasoning at the top of the file, because rewriting it back is a one-line tidy-up that leaves the
program passing.

The `%lo` form needs the patch site inside the first 2 KB of `.text`. Past that the immediate's sign
bit is set, the store address comes out negative, the write is dropped and test 2 fails — loud, not
silent.

## Decision 3 — `test/asm/riscv_test.h` is not touched

`textload.S` defines its own `trap_handler` and installs it with the existing
`RVTEST_INSTALL_TRAP_HANDLER`, reusing `RVTEST_TRAP_DATA`'s three words in RAM. One program needs the
length-decoding handler, so a shared macro would be a second surface for one caller.

That also settles the byte-identity question the brief raised: no shared header, linker script or
macro file changed, so the other 56 programs assemble to exactly the bytes they did before. The
suite tables confirm it — every one of their retire and spec-checked counts is unchanged.

## Decision 4 — `test/sail/rv32imc_zicsr.json` needs no change, and this is the confirmation

Its single `MainMemory` region is `0x0000_0000` for `0x0010_0000`, with `executable`, `readable` and
`writable` all true, so the reference model already answers a load or a store in the text region the
way this core does. Verified by running rather than by reading: `make cosim-suite` is 59/59 with
`test/COSIM_EXPECTED_FAIL` empty, and `selfmod.S` — where the two machines would have to disagree
about what instruction is at an address — agrees. The file is a complete `--config` and is rejected
outright if a key is missing, so editing it on a guess is the one thing that cannot be done cheaply
here.

## The measurement

Steals counted with a throwaway `fetch_stall` counter in `test/testbench.v`, read out through
`test/cxxrtl.cc`; the instrumentation is not part of this change.

| program | retires | steals |
|---|---|---|
| `selfmod.S` | 47 | **6** |
| `textload.S` | 197 | **4** |
| `contend.S` | 113 | **16** |
| `add.S`, `sw.S`, `trap.S`, `rvc.S` | unchanged | **0** |

Twenty-six stolen fetch windows, against zero anywhere in the tree before this change.

Two mutations, each against the whole 59-program table:

- **`fetch_stall` tied low in `rtl/imemory.v`.** `selfmod.S` goes 47 retires to 6 and `contend.S`
  113 to 25, both `TRAP-TO-ZERO`. Every other line of the table is byte-identical, including
  `textload.S`.
- **`instr_fencei` dropped from the serialization term.** `selfmod.S` alone goes red, `FAIL 2`, 47
  retires to 17. `fencenop.S` — the only other program that executes a `fence.i` — does not move,
  which is the measurement behind saying it checks that the instruction retires and nothing about
  ordering.

## Consequences

- **`textload.S` does not go red when the steal is tied low**, and that is worth knowing rather than
  hiding. Its four text loads all sit in the handler, where the corrupted fetch window lands on a
  cycle decode was going to bubble anyway. It exercises the text *read* path and the compressed
  trap; `contend.S` and `selfmod.S` are what the steal itself hangs on.
- **The `.data` half of the brief's step 5 is not here.** A load address in the text region for
  `.data` plus a copy loop in `riscv_test.h` changes shared infrastructure every program reads, and
  it cannot be checked without the SoC synthesis flow. It is its own change. Until it lands the SoC
  still cannot run a program that reads its own `.data`, which is what the deferred-bootloader entry
  already says.
- **No test may jump into the data region, and none does.** The core fetches zero there and traps in
  decode; the reference model's single region is executable across the whole megabyte. Such a test
  would diverge by construction, and the divergence would be the model's permissiveness.
- **`c.ebreak`'s cause is now asserted by a test that lets the assembler compress freely.**
  `cebreak.S` already covered the cause at both alignments, but only by padding each faulting
  instruction with a `c.nop` so the shared `mepc + 4` handler landed on a boundary. `textload.S`
  needs no padding and no `.option norvc`, so the discipline that hid the defect is no longer the
  only way to write a resuming trap test here.
- **`fence.i` costs a drain and now has a program that says so.** ADR-0061's own closing description
  of itself — an argued fix with a unit test — no longer applies.

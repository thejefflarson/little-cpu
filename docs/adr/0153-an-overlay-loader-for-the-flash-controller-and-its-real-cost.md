# ADR-0153: An overlay loader for the flash controller, and its real cost

**Status:** Accepted · 2026-09-01

## Context

ADR-0135 built `rtl/spiflash.v`, a byte-at-a-time SPI controller reachable in simulation, and closed
with two things still open: nothing used it, and the two payoffs the controller was meant to enable
— growing what a program can do past the 8 KB ROM ceiling, and reading persistent data — were both
still unbuilt. ADR-0135 also found that neither payoff is a bigger ROM: `rtl/imemory.v` refuses a
`ROM_WORDS` that is not a power of two, so the next legal size above 8 KB is 16 KB, and a fetch window
needs all four of the part's free `SB_SPRAM256KA` for one bank of one parity, leaving nothing for the
64 KB data RAM. So growing what a program can do after boot has to be a run-time copy into the 8 KB
that already exists, not a bigger image.

This ticket is the software half that needs no board: a driver, a loader, and a program that proves an
overlay actually ran. The pins stay exactly where ADR-0135 left them — tied off on `soc/board_upduino.v`,
`rtl/spiflash.v` reachable only from the bus. Wiring them to real pins is still a future ticket's, and
it still owes the real-board contention measurement ADR-0135 describes.

## The loader

`test/asm/spioverlay.S` adds `spi_read_bytes`: `a0` = a 24-bit flash byte address, `a1` = a
destination pointer, `a2` = a byte count. It selects the flash, sends a sequential-read command
(`0x03`) and the three address bytes, clocks `a2` bytes into `(a1)`, and releases the chip select.
It does not know or care whether its destination is code or data — the two call sites in the same
file use it both ways.

**The loader is a subroutine in the test program, not a library and not part of `test/crt0.S`.**
`test/crt0.S` copies `.data` out of ROM before `main` runs on every `.c` program; a flash read is a
distinct operation only one program in the suite needs, and every program in `test/asm` is a
standalone translation unit — `test/run_tests.sh`, `test/cosim.py`'s `assemble()` and the Makefile's
`soc-rom` each assemble one file at a time, with no cross-file linking between programs. A shared
assembly library would be a fifth thing a program-shape change touches, for one caller. The natural
home for one caller is beside it.

**A named label, not a numeric one, is what makes the loader's own read loop correct.** GAS's `SPI_WAIT_IDLE`
and `SPI_XFER_REG` macros — the second one added here, next to `SPI_XFER` in `test/asm/riscv_test.h`, to
carry a source and destination register instead of `SPI_XFER`'s hardcoded immediate and `t1`/`t2` — each
define their own numeric local label `1:` on every expansion, and `1b` resolves to the nearest prior
one. A per-byte loop spelled with its own `1:`/`1b` around a call to either macro compiles clean and
runs wrong: once the loop has executed once, the nearest prior `1:` is the macro's own internal wait
loop, not the loop's own top, so the backward branch lands inside that wait loop instead — skipping the
`sw` that starts each new exchange and re-reading the first byte forever. This was not a hypothetical:
it is exactly the shape the first version of `spi_read_bytes` shipped with, and it was caught by running
the program, not by reading it — every byte the loop stored came back identical, all four bytes of the
overlay's own instruction read back as the flash's true first byte repeated four times. `.Lspi_read_next_byte`
is the fix, and the comment beside it is the tripwire this discipline calls for.

## What proves an overlay ran

`test/asm/spioverlay.S`'s `patch_stub` is loaded three times, the same slot each time: once from a
template in the file's own text (test 2), once from the flash (test 3), and once from a second,
different template (test 4). All three write the same instruction shape — `addi s8, s10, imm` — so
the only thing that can move the answer between tests is which bytes actually landed in the slot. A
loader that silently failed to copy, or a `fence.i` that did not order the write ahead of the fetch,
would leave test 2's 111 in place for test 3 to read back, not 224; deleting either mechanism reads
exactly the way it does in `test/asm/selfmod.S`, which this file's structure is deliberately borrowed
from. The instruction itself is not chosen to make a point — it is what `test/spiflash_model.v`'s
formula (`a[7:0] ^ a[15:8] ^ 0x5a`) gives at address `0x00064f`, the one address in the low 16 bits of
address space this ticket's own search found whose four bytes decode as `addi s8, s10, 224` with no
literal picked to match. `rd`/`rs1` came out of that same search, which is why the two local templates
use them rather than the other way round.

Test 5 is the same loader used for data instead of code: sixteen bytes read into a RAM buffer and
checked one at a time against the model's own formula, recomputed in the program the way
`test/asm/spiflash.S` recomputes the JEDEC id rather than quoting bytes it read once. That is the
persistent-storage and firmware-update shape the ticket asks for, next to test 3's execute-in-place
one, on the same primitive.

## The co-simulation decision

The program is baselined in `test/COSIM_EXPECTED_FAIL` as `DISAGREE AT 22`, not structured to avoid
the divergence. Sail has plain memory where the flash controller sits — `busy` is never set, and a
load reads back whatever was last stored. Every byte this loader's read loop clocks out as its dummy
is `0x00`, so every byte it reads back on Sail is `0x00` too: the four bytes patched into `patch_stub`
come back all zero, an illegal instruction, where the real controller delivers the four bytes the
header above derives. That is unavoidable by construction — the whole point of the program is to run
bytes that came from a device the model does not have — so "structure it to agree" is not on the
table the way it might be for a device a program can read without branching on.

**The trap handler installed here, where `test/asm/selfmod.S` needs none, is what turns an opaque
budget exhaustion into a real divergence.** Left untrapped, the corrupted word is `c.illegal 0x0` on
the model, `mtvec` is still zero, and the fault restarts the whole program at `_start` — forever, since
the same corruption recurs every pass. That reproduces as `INCONCLUSIVE SAIL-LIMIT`, measured directly
(`make cosim-run PROG=spioverlay.S` against the untrapped version hit Sail's 20000-instruction budget
having made 7196 register-file changes against the real core's 1823 to a clean `PASS`), and compares
nothing — the same shape as the three existing timer-related entries. Installing the handler is not a
workaround for the gate: it is what stops a genuine authoring mistake in this program — an unrelated
fault anywhere in it — from looking identical to the divergence this file already expects. With it,
Sail resumes past the fault holding test 2's un-overwritten 111, test 3's own in-band check takes the
branch a broken loader would also take, and the run reaches an HTIF verdict (`FAIL 3`) instead of
running out the clock. `make cosim-suite` grades the exact point named (architectural change 22, the
busy-bit read inside the corrupted word's own read loop) in both directions.

## The overlay's real cost

`rtl/spiflash.v`'s `sck` is the bus clock halved, so one byte is 16 core-clock cycles at 12 MHz — the
same "eight clocks a byte" ADR-0135 states, doubled for the two edges each SCK period takes. That
sets the wire's own ceiling:

| | |
|---|---|
| wire rate | 12 MHz / 16 cycles/byte = **750 000 bytes/s** |
| 8 KB at the wire rate | 8192 / 750 000 = **10.92 ms** |

That is the floor, not what this loader gets. **Measured directly** — an isolated 8192-byte
`spi_read_bytes` call, the identical routine `test/asm/spioverlay.S` ships, run under the cxxrtl
runner with `--stalls` — costs **428 270 cycles**, not the wire's 131 072:

| | |
|---|---|
| this loader's 8 KB | 428 270 cycles / 12 MHz = **35.69 ms** |
| effective throughput | 8192 / 0.03569 s = **≈ 229 500 bytes/s**, 30.6% of the wire rate |
| slowdown vs. the wire | **≈ 3.27×** |

The gap is the byte-at-a-time polling loop, not the protocol: every byte's exchange is bracketed by
two `SPI_WAIT_IDLE` spins, each at least one `lw`/`andi`/`bnez` past the exchange's own 16 cycles, plus
the store and the two pointer/counter increments that move the loop forward. None of that is a
defect to fix here — this ticket is the mechanism, not its optimisation, and a byte-at-a-time
controller was ADR-0135's own deliberate choice over a command engine — but the number is the one
this ticket exists to produce: **loading an 8 KB overlay costs about 36 ms of the board's 12 MHz
clock**, roughly 430 000 cycles that would otherwise retire instructions. That is cheap enough for a
boot-time or update-time load — once, before the program that needs the overlay starts running — and
too expensive to hide inside a hot path or a context switch. Overlays are usable at boot; they are not
free enough to reach for casually.

## Tests

`test/asm/spioverlay.S` is in the suite with a `test/OBSERVED_FLOOR` entry (786 retires, 782
spec-checked, both sim legs green under `make test`). `test/COSIM_EXPECTED_FAIL` carries the entry
above. No new graded comparison was added to `test/probe_gates.sh` — the program's own `TEST_CASE`
sequence (baseline, overlay, revert, bulk check) is the grader, and its red direction was demonstrated
directly during development: the local-label bug above made every one of the four copied bytes read
back as the flash's first byte, and test 3 failed on it before the fix.

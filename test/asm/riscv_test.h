// Local, minimal riscv_test.h (ADR-0008).
//
// Unlike upstream riscv-tests' env/p/riscv_test.h, this core has no privilege
// modes and no PMP, so none of that setup belongs here. RVTEST_CODE_BEGIN just
// establishes `_start` and zeroes the test-number register;
// RVTEST_PASS/RVTEST_FAIL write the riscv-tests `tohost` encoding (ADR-0008) to
// a fixed doubleword at the base of RAM and spin. The cxxrtl runner
// (test/cxxrtl.cc) and the iverilog bench watch that address for the magic
// write instead of relying on ecall/mtvec; the Sail model (ADR-0032) speaks
// the same HTIF protocol and terminates on it. See RVTEST_DATA_BEGIN for why
// the doubleword width is not cosmetic.
//
// The trap-handler macros at the bottom are strictly opt-in and
// RVTEST_CODE_BEGIN is deliberately untouched by them: the suite's other tests
// must keep assembling to the same bytes, and must not start executing CSR
// instructions before the CSR RTL exists (M3). A test opts in by invoking
// RVTEST_TRAP_HANDLER (in .text) and RVTEST_TRAP_DATA (in .data) and then
// installing the handler with RVTEST_INSTALL_TRAP_HANDLER.
//
// Three constraints on trap tests, none of them optional, all of them
// consequences of decisions recorded elsewhere:
//
//   1. **The handler cannot read the faulting instruction.** This core is
//      Harvard with physically disjoint buses (ADR-0008): no load can reach
//      ROM. So a handler cannot inspect insn[1:0] to decide whether to resume
//      at mepc+2 or mepc+4. The resume path here adds 4 unconditionally, which
//      means **every trapping instruction in a resuming test must be
//      uncompressed** -- wrap it in `.option norvc` / `.option pop`, because
//      the assembler compresses freely at -march=rv32imc_zicsr (ADR-0014's
//      sunset note) and would otherwise silently turn a 4-byte `lw` into a
//      2-byte `c.lw`, after which the resume skips a whole instruction. Tests
//      that instead terminate from inside the handler are unconstrained.
//   2. **Install the handler before faulting.** `mtvec` resets to 0, which is
//      `_start` (ADR-0029), so a trap taken before installation silently
//      restarts the program. Both sim legs are supposed to make that loud, but
//      the discipline is the actual fix.
//   3. **Trap tests must self-check in band.** riscv-formal ships no spec model
//      for csrr*/ecall/ebreak/mret at the pinned SHA, so `spec_valid` is 0 for
//      all of them and the per-retire monitor -- the thing that makes every
//      other test self-checking (ADR-0019) -- has nothing to say about the
//      instructions these tests exist to exercise. Hence the handler records
//      mcause/mepc/a trap count into RAM and the test body asserts on them
//      with the ordinary TEST_CASE machinery. Do not assume a passing trap
//      test was cross-checked against an oracle; it was checked against
//      whatever the test itself asserts.
//
// The handler clobbers t0 and t1 and nothing else, so a test body must not
// hold live values in those registers across an instruction that may trap.

#ifndef __RISCV_TEST_H
#define __RISCV_TEST_H

// riscv-tests' test_macros.h expects TESTNUM to already be defined.
#define TESTNUM gp

// All 51 riscv-tests-derived `.S` files in this suite include RVTEST_RV64U
// to mark "this is a plain unprivileged integer test" — a leftover of
// upstream's env selection, not an XLEN claim. It expands to nothing here:
// this core is RV32 only (ADR-0002) and there is no privilege setup to do.
#define RVTEST_RV64U

#define RVTEST_CODE_BEGIN                                                    \
        .text;                                                              \
        .align  2;                                                          \
        .globl  _start;                                                     \
_start:                                                                      \
        li      TESTNUM, 0;

// Nothing should ever reach here — RVTEST_PASS/RVTEST_FAIL each end in an
// infinite loop after the tohost write — but spin rather than fall off the
// end of ROM if a test is malformed.
#define RVTEST_CODE_END                                                      \
1:      j       1b

// Both verdict macros write the doubleword's UPPER word first and the verdict
// itself last, and the order is deliberate (ADR-0039). This core has no 64-bit
// store, so the doubleword `tohost` is written as two `sw`s; Sail's HTIF fires
// on each half-write once the other half has been seen, so writing the upper
// (always-zero) word first makes the verdict store the single event that
// terminates the reference model -- the same store test/cxxrtl.cc and
// test/cosim.cc already stop on when RAM_BASE's low word goes non-zero. Both
// sides then end on the same instruction. Writing them the other way round
// works too, but leaves the reference model running one store longer than the
// core, for no gain.
//
// Neither store writes a register, so the sequence of architectural
// register-file states -- which is what ADR-0032's co-simulation compares --
// is unchanged by this pair.
#define RVTEST_PASS                                                          \
        li      TESTNUM, 1;                                                 \
        la      t0, tohost;                                                 \
        sw      x0, 4(t0);                                                  \
        sw      TESTNUM, 0(t0);                                             \
1:      j       1b

#define RVTEST_FAIL                                                          \
        sll     TESTNUM, TESTNUM, 1;                                        \
        or      TESTNUM, TESTNUM, 1;                                        \
        la      t0, tohost;                                                 \
        sw      x0, 4(t0);                                                  \
        sw      TESTNUM, 0(t0);                                             \
1:      j       1b

// `tohost` is a full DOUBLEWORD, 8-byte aligned, and both of those are
// load-bearing rather than cosmetic (ADR-0039, amending ADR-0008).
//
// The riscv-tests HTIF protocol this encoding is borrowed from defines
// `tohost` as a 64-bit location, and every consumer that speaks HTIF -- the
// Sail RISC-V model among them -- claims the whole doubleword at the symbol
// as an IO window the moment it sees the symbol in the ELF. It was a 32-bit
// `.word` here, so `.data` (every load/store test's TEST_DATA, ADR-0008 puts
// it immediately after) began four bytes INSIDE that window: the reference
// model answered every `lw` from RAM_BASE+4 with zero. Eight programs
// "diverged" for a reason that was entirely this file's fault.
//
// Padding it is what makes the co-simulation leg (ADR-0032) able to hand Sail
// the same ELF the other two legs run, unmodified, and to terminate on the
// verdict rather than on an instruction budget.
//
// Nothing else moves. `tohost` is still the first thing in the RAM region and
// still the low word at RAM_BASE, so test/cxxrtl.cc's `ram_data[0]`,
// test/cosim.cc's, and test/testbench.v's RAM decode are all unchanged; the
// `.data` that follows simply starts four bytes later. RVTEST_PASS/FAIL still
// write it with a 32-bit `sw` -- this core has no 64-bit store -- and the
// upper word stays zero, which is what makes the low word alone a faithful
// read of the riscv-tests encoding on a little-endian machine.
#define RVTEST_DATA_BEGIN                                                    \
        .pushsection .tohost,"aw",@progbits;                                \
        .align  3;                                                          \
        .global tohost;                                                     \
tohost:                                                                      \
        .dword  0;                                                          \
        .popsection;                                                        \
        .data;                                                              \
        .align  2;

#define RVTEST_DATA_END

// ---------------------------------------------------------------------------
// Opt-in trap support (M3). Nothing above this line references any of it, so a
// test that does not invoke these macros assembles to exactly the bytes it did
// before they existed. Read the three constraints at the top of this file
// before writing a trap test.
// ---------------------------------------------------------------------------

// Where the handler records what it saw. Invoke inside the data section (after
// RVTEST_DATA_BEGIN) so it lands in RAM, which is the only memory a load can
// reach on this Harvard core (ADR-0008).
#define RVTEST_TRAP_DATA                                                     \
        .align  2;                                                          \
        .global trap_count;                                                 \
trap_count:                                                                  \
        .word   0;                                                          \
        .global trap_cause;                                                 \
trap_cause:                                                                  \
        .word   0;                                                          \
        .global trap_epc;                                                   \
trap_epc:                                                                    \
        .word   0;

// The recording handler: bump `trap_count`, store `mcause` and `mepc`, then
// resume at mepc+4 via `mret`.
//
// +4 is unconditional because the handler cannot look at the faulting
// instruction to find out whether it was 2 or 4 bytes (constraint 1 at the top
// of this file) — so the trapping instruction must be uncompressed. Clobbers
// t0 and t1.
//
// Invoke it in .text somewhere control cannot fall into — after TEST_PASSFAIL
// is the convention, since both of those paths end in an infinite loop.
// `.align 2` satisfies mtvec's 4-byte-aligned WARL base (ADR-0005).
//
// The body is `.option norvc` and straight-line, so every instruction in it is
// exactly 4 bytes and `(trap_handler_end - trap_handler) / 4` is the number of
// instructions one trap entry retires. A test that reasons about `minstret`
// across a trap (ADR-0027) should compute that expression rather than hardcode
// a count, so editing this macro cannot silently invalidate it.
#define RVTEST_TRAP_HANDLER                                                  \
        .align  2;                                                          \
trap_handler:                                                                \
        .option push;                                                       \
        .option norvc;                                                      \
        la      t0, trap_count;                                             \
        lw      t1, 0(t0);                                                  \
        addi    t1, t1, 1;                                                  \
        sw      t1, 0(t0);                                                  \
        csrr    t1, mcause;                                                 \
        la      t0, trap_cause;                                             \
        sw      t1, 0(t0);                                                  \
        csrr    t1, mepc;                                                   \
        la      t0, trap_epc;                                               \
        sw      t1, 0(t0);                                                  \
        addi    t1, t1, 4;                                                  \
        csrw    mepc, t1;                                                   \
        mret;                                                               \
trap_handler_end:                                                            \
        .option pop;

// The alternative handler: any trap is a test failure, reported against
// whatever test number was live when it happened. Terminates from inside the
// handler, so it is not subject to the uncompressed-instruction constraint.
// Define at most one of RVTEST_TRAP_HANDLER / RVTEST_TRAP_HANDLER_FATAL per
// test — they share the `trap_handler` symbol.
#define RVTEST_TRAP_HANDLER_FATAL                                            \
        .align  2;                                                          \
trap_handler:                                                                \
        RVTEST_FAIL

// Point mtvec at the handler. Direct mode: mtvec[1:0] = 0, which the handler's
// `.align 2` guarantees. Clobbers t0.
#define RVTEST_INSTALL_TRAP_HANDLER                                          \
        la      t0, trap_handler;                                           \
        csrw    mtvec, t0;

#endif

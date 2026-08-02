// Local, minimal riscv_test.h (ADR-0008). This core has no privilege modes and
// no PMP, so none of upstream riscv-tests' env/p setup belongs here.
// RVTEST_PASS/RVTEST_FAIL write the riscv-tests `tohost` encoding to a fixed
// doubleword at the base of RAM and spin; both cxxrtl runners, the iverilog
// bench and the Sail model watch that address and terminate on it, so nothing
// depends on ecall or mtvec. See RVTEST_DATA_BEGIN for why the width matters.
//
// The trap macros at the bottom are opt-in, and RVTEST_CODE_BEGIN must stay
// untouched by them so the other tests keep assembling to the same bytes. A
// test opts in with RVTEST_TRAP_DATA (in .data), RVTEST_TRAP_HANDLER (in .text)
// and RVTEST_INSTALL_TRAP_HANDLER.
//
// Three constraints on trap tests:
//
//   1. The handler cannot read the faulting instruction -- this core is Harvard
//      with disjoint buses (ADR-0008), so no load reaches ROM and nothing can
//      inspect insn[1:0] to choose between mepc+2 and mepc+4. The resume path
//      adds 4 unconditionally, so every trapping instruction in a resuming test
//      must be wrapped in `.option norvc` / `.option pop`: the assembler
//      compresses freely at -march=rv32imc_zicsr, and a `lw` that became a
//      `c.lw` would make the resume skip a whole instruction. Tests that
//      terminate from inside the handler are unconstrained.
//   2. Install the handler before faulting. `mtvec` resets to 0, which is
//      `_start` (ADR-0029), so a trap taken before installation restarts the
//      program.
//   3. Trap tests must self-check in band. riscv-formal ships no spec model for
//      csrr*/ecall/ebreak/mret at the pinned SHA, so `spec_valid` is 0 for all
//      of them and the per-retire monitor has nothing to say about the very
//      instructions these tests exercise. A passing trap test was checked
//      against its own assertions, not against an oracle.
//
// The handler clobbers t0 and t1, so a test body must not hold live values
// there across an instruction that may trap.

#ifndef __RISCV_TEST_H
#define __RISCV_TEST_H

// riscv-tests' test_macros.h expects TESTNUM to already be defined.
#define TESTNUM gp

// A leftover of upstream's env selection that the `.S` files still invoke, not
// an XLEN claim. It expands to nothing: this core is RV32 only (ADR-0002) and
// has no privilege setup to do.
#define RVTEST_RV64U

#define RVTEST_CODE_BEGIN                                                    \
        .text;                                                              \
        .align  2;                                                          \
        .globl  _start;                                                     \
_start:                                                                      \
        li      TESTNUM, 0;

// Unreachable in a well-formed test, since RVTEST_PASS/RVTEST_FAIL each end in
// an infinite loop -- but spin rather than run off the end of ROM.
#define RVTEST_CODE_END                                                      \
1:      j       1b

// The upper word is written first and the verdict last, and the order matters:
// this core has no 64-bit store, so the doubleword goes out as two `sw`s, and
// Sail's HTIF fires on whichever half-write completes the pair. Writing the
// always-zero upper word first makes the verdict store the single event that
// stops the reference model -- the same store the cxxrtl runners stop on when
// RAM_BASE's low word goes non-zero -- so both sides end on the same
// instruction (ADR-0039).
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

// `tohost` must stay a full doubleword, 8-byte aligned. HTIF defines it as a
// 64-bit location, and every consumer that speaks the protocol -- Sail among
// them -- claims the whole doubleword at the symbol as an IO window as soon as
// it sees the symbol in the ELF. As a 32-bit `.word` it put the start of `.data`
// four bytes inside that window, and Sail answered every load from there with
// zero (ADR-0039, amending ADR-0008).
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

// Where the handler records what it saw. Invoke it after RVTEST_DATA_BEGIN so
// it lands in RAM, the only memory a load can reach on this Harvard core.
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

// The recording handler: bump `trap_count`, store `mcause` and `mepc`, resume
// at mepc+4. The +4 is unconditional (constraint 1 at the top of this file), so
// the trapping instruction must be uncompressed. Clobbers t0 and t1.
//
// Invoke it in .text somewhere control cannot fall into -- after TEST_PASSFAIL
// by convention, since both of those paths end in an infinite loop. The
// `.align 2` satisfies mtvec's 4-byte-aligned WARL base (ADR-0005).
//
// The body is `.option norvc` and straight-line, so every instruction is 4
// bytes and `(trap_handler_end - trap_handler) / 4` is how many instructions
// one trap entry retires. A test reasoning about `minstret` across a trap
// should compute that rather than hardcode a count, so editing this macro
// cannot silently invalidate it.
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

// Any trap is a test failure, reported against whatever test number was live.
// It terminates from inside the handler, so the uncompressed-instruction
// constraint does not apply. Use at most one of the two handlers per test --
// they share the `trap_handler` symbol.
#define RVTEST_TRAP_HANDLER_FATAL                                            \
        .align  2;                                                          \
trap_handler:                                                                \
        RVTEST_FAIL

// Point mtvec at the handler, direct mode -- mtvec[1:0] = 0, which the
// handler's `.align 2` guarantees. Clobbers t0.
#define RVTEST_INSTALL_TRAP_HANDLER                                          \
        la      t0, trap_handler;                                           \
        csrw    mtvec, t0;

#endif

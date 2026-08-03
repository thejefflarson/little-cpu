// Local, minimal riscv_test.h. Every consumer -- both cxxrtl
// runners, the iverilog bench and the Sail model -- terminates on the `tohost`
// write below, so nothing here depends on ecall or mtvec.
//
// The trap macros at the bottom are opt-in, and RVTEST_CODE_BEGIN must stay
// untouched by them so the other tests keep assembling to the same bytes.
// Three constraints on a test that uses them:
//
//   1. Wrap every trapping instruction in `.option norvc` / `.option pop`. The
//      handler resumes at mepc+4 unconditionally, because Harvard buses mean it
//      cannot read the faulting instruction to tell 2 bytes from 4, and the
//      assembler compresses freely at -march=rv32imc_zicsr. Tests that
//      terminate from inside the handler are unconstrained.
//   2. Install the handler before faulting. `mtvec` resets to 0, which is
//      `_start`, so a trap before installation restarts the program.
//   3. Assert in band. riscv-formal has no spec model for csrr*/ecall/ebreak/
//      mret at the pin, so the per-retire monitor says nothing about the
//      instructions these tests exist to exercise.
//
// The handler clobbers t0 and t1.

#ifndef __RISCV_TEST_H
#define __RISCV_TEST_H

// riscv-tests' test_macros.h expects TESTNUM to already be defined.
#define TESTNUM gp

// A leftover of upstream's env selection, not an XLEN claim.
#define RVTEST_RV64U

#define RVTEST_CODE_BEGIN                                                    \
        .text;                                                              \
        .align  2;                                                          \
        .globl  _start;                                                     \
_start:                                                                      \
        li      TESTNUM, 0;

#define RVTEST_CODE_END                                                      \
1:      j       1b

// The upper word first and the verdict last, because there is no 64-bit store
// and Sail's HTIF fires on whichever half-write completes the pair: this way
// the verdict store is what stops the reference model, on the same instruction
// the cxxrtl runners stop on.
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

// `tohost` must stay a full doubleword, 8-byte aligned: HTIF defines it as a
// 64-bit location and every consumer claims the whole doubleword at the symbol
// as an IO window. As a 32-bit `.word` it put the start of `.data` four bytes
// inside that window, and Sail answered every load from there with zero.
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

// Invoke after RVTEST_DATA_BEGIN so it lands in RAM, the only memory a load can
// reach on this Harvard core.
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

// Invoke in .text somewhere control cannot fall into -- after TEST_PASSFAIL by
// convention, since both of those paths end in an infinite loop. The `.align 2`
// satisfies mtvec's 4-byte-aligned WARL base.
//
// The body is straight-line and uncompressed, so
// `(trap_handler_end - trap_handler) / 4` is how many instructions one trap
// entry retires. A test reasoning about `minstret` across a trap should compute
// that rather than hardcode a count.
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

// Terminates from inside the handler, so constraint 1 does not apply.
#define RVTEST_TRAP_HANDLER_FATAL                                            \
        .align  2;                                                          \
trap_handler:                                                                \
        RVTEST_FAIL

// Direct mode: mtvec[1:0] = 0, which the handler's `.align 2` guarantees.
#define RVTEST_INSTALL_TRAP_HANDLER                                          \
        la      t0, trap_handler;                                           \
        csrw    mtvec, t0;

#endif

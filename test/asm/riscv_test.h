// Local, minimal riscv_test.h (ADR-0008).
//
// Unlike upstream riscv-tests' env/p/riscv_test.h, this core has no CSRs, no
// privilege modes, no PMP, and no trap handling wired up yet (M3 work), so
// none of that setup belongs here. RVTEST_CODE_BEGIN just establishes
// `_start` and zeroes the test-number register; RVTEST_PASS/RVTEST_FAIL
// write the riscv-tests `tohost` encoding (ADR-0008) to a fixed word in RAM
// and spin. The cxxrtl runner (test/cxxrtl.cc) and the iverilog bench watch
// that address for the magic write instead of relying on ecall/mtvec.

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

#define RVTEST_PASS                                                          \
        li      TESTNUM, 1;                                                 \
        la      t0, tohost;                                                 \
        sw      TESTNUM, 0(t0);                                             \
1:      j       1b

#define RVTEST_FAIL                                                          \
        sll     TESTNUM, TESTNUM, 1;                                        \
        or      TESTNUM, TESTNUM, 1;                                        \
        la      t0, tohost;                                                 \
        sw      TESTNUM, 0(t0);                                             \
1:      j       1b

#define RVTEST_DATA_BEGIN                                                    \
        .pushsection .tohost,"aw",@progbits;                                \
        .align  2;                                                          \
        .global tohost;                                                     \
tohost:                                                                      \
        .word   0;                                                          \
        .popsection;                                                        \
        .data;                                                              \
        .align  2;

#define RVTEST_DATA_END

#endif

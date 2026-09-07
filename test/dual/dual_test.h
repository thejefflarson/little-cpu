// Macros for the two-hart torture programs in this directory.

#ifndef __DUAL_TEST_H
#define __DUAL_TEST_H

#include "riscv_test.h"
#include "test_macros.h"

#define DUALTEST_SPIN_LIMIT 256

// The one test number a macro fails at on its own.
#define DUALTEST_BAD_HARTID 1

#define DUALTEST_CODE_BEGIN                                                  \
        .text;                                                              \
        .align  2;                                                          \
        .globl  _start;                                                     \
_start:                                                                      \
        li      TESTNUM, 0;                                                 \
        csrr    t0, mhartid;                                                \
        beqz    t0, 8f;                                                     \
        li      t1, 1;                                                      \
        beq     t0, t1, hart1;                                              \
        li      TESTNUM, DUALTEST_BAD_HARTID;                               \
        j       fail;                                                       \
8:

#define DUALTEST_WAIT_REG( testnum, sym, valreg )                            \
        li      TESTNUM, testnum;                                           \
        la      t0, sym;                                                    \
        li      t2, DUALTEST_SPIN_LIMIT;                                    \
9:      lw      t1, 0(t0);                                                  \
        beq     t1, valreg, 8f;                                             \
        addi    t2, t2, -1;                                                 \
        bnez    t2, 9b;                                                     \
        j       fail;                                                       \
8:

#define DUALTEST_WAIT( testnum, sym, val )                                   \
        li      t3, val;                                                    \
        DUALTEST_WAIT_REG( testnum, sym, t3 )

#define DUALTEST_HART1_DONE                                                  \
        li      t1, 1;                                                      \
        la      t0, hart1_done;                                             \
        sw      t1, 0(t0);                                                  \
1:      j       1b

#define DUALTEST_JOIN( testnum )                                             \
        DUALTEST_WAIT( testnum, hart1_done, 1 )

#define DUALTEST_DATA                                                        \
        .align  2;                                                          \
        .global hart1_done;                                                 \
hart1_done:                                                                  \
        .word   0;

#endif

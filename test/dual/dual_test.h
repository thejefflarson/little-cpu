// Macros for the two-hart torture programs in this directory. They are the
// suite's `.S` shape -- test/asm/sections.lds, `.data` poked into RAM by a
// harness -- with one image running on both harts.
//
// THESE PROGRAMS DO NOT RUN YET. Nothing in this tree instantiates two harts,
// so the dual runner is owed and every program here is source waiting for it.
// test/dual_build.sh is the only thing that reads them today: it assembles and
// links each one and refuses a warning, which says they are syntactically alive
// and says nothing whatever about the hardware.
//
// WHAT THE RUNNER OWES THEM, and the whole of it:
//
//   1. Two harts out of ONE text image, both released from reset at address 0.
//      `mhartid` reads 0 on one and 1 on the other -- that is the only thing
//      that tells them apart, and the dispatch below is the only place it is
//      read.
//   2. One shared data RAM at test/asm/sections.lds's `ram` origin, with
//      `.data` poked in before release, exactly as the single-hart runner does
//      for a `.S` program.
//   3. Termination on the FIRST `tohost` write from either hart, the way the
//      single-hart runner terminates. A failing hart writes it immediately and
//      stops the machine; the passing path writes it only after the join below,
//      so a PASS means both harts finished.
//
// A hart that is not 0 or 1 fails at test 1 rather than running hart 1's half,
// so a runner that brings up three harts is reported instead of half-tested.
//
// THE SPIN LIMIT IS THE POINT OF THE BOUNDED WAIT. A lock that is never
// released, a flag that never arrives and a hart that never started all produce
// a hang, and a hang is indistinguishable from a harness that never released
// the second hart. Every wait here is bounded, so those become `FAIL <n>` at a
// named test number instead.

#ifndef __DUAL_TEST_H
#define __DUAL_TEST_H

#include "riscv_test.h"
#include "test_macros.h"

// Iterations, not cycles. The longest legitimate wait in this directory is one
// hart sitting out the other's critical section -- about twenty instructions,
// so tens of cycles, so under twenty times round a four-instruction spin. This
// is an order of magnitude above that and still exhausts inside any cycle
// budget a runner would give these programs.
#define DUALTEST_SPIN_LIMIT 256

// The one test number a macro fails at on its own. Every other wait below takes
// its number from the program, so a `FAIL <n>` names which wait gave up.
#define DUALTEST_BAD_HARTID 1

// `_start` for both harts. Hart 0 falls through to the code that follows;
// hart 1 jumps to `hart1`, which every program in this directory defines. There
// is no matching end: a program here closes with RVTEST_CODE_END like every
// other program in the suite.
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

// Spins until the word at `sym` reads what `valreg` holds, and fails at
// `testnum` if it never does. Clobbers t0, t1, t2 and TESTNUM, so `valreg` may
// not be one of those. A handshake that counts rounds up instead of setting and
// clearing a flag needs this form: there is no round in which a stale flag
// reads as this round's.
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

// The same against a constant. Clobbers t3 as well.
#define DUALTEST_WAIT( testnum, sym, val )                                   \
        li      t3, val;                                                    \
        DUALTEST_WAIT_REG( testnum, sym, t3 )

// Hart 1's last act: publish that it finished, then park. It must NOT write
// `tohost` -- that would stop the machine with hart 0 still running.
#define DUALTEST_HART1_DONE                                                  \
        li      t1, 1;                                                      \
        la      t0, hart1_done;                                             \
        sw      t1, 0(t0);                                                  \
1:      j       1b

// Hart 0's join. Everything after it in a program runs with hart 1 finished, so
// a final read-back sees the whole execution rather than a prefix of it.
#define DUALTEST_JOIN( testnum )                                             \
        DUALTEST_WAIT( testnum, hart1_done, 1 )

// The flag `DUALTEST_HART1_DONE` and `DUALTEST_JOIN` share. Invoke once, after
// RVTEST_DATA_BEGIN, so it lands in RAM.
#define DUALTEST_DATA                                                        \
        .align  2;                                                          \
        .global hart1_done;                                                 \
hart1_done:                                                                  \
        .word   0;

#endif

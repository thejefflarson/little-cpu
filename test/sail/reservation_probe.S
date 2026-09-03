// An LR/SC reservation probe for the Sail model ALONE. No core runs this, and
// it is deliberately not in test/asm: both sim legs glob that directory and
// would grade this against a core, which is exactly what it must not be. The
// graded programs that DO exercise lr.w/sc.w against the core are
// test/asm/lrsc.S and its neighbours. Built and run by
// test/sail/reservation_probe.sh.
//
// WHAT IT ASKS. The reservation policy is the one part of LR/SC the spec leaves
// to the implementation, and co-simulation compares register values -- an SC's
// result IS a register value, 0 for success and nonzero for failure. So a
// reference model that drops the reservation where this core keeps it turns
// every LR/SC program into a divergence on a correct core. The six cases below
// ask the model directly, one architectural event per case.
//
// HOW IT ANSWERS. Each case leaves a bit in s0: 1 when the SC SUCCEEDED, 0 when
// it failed. Bit 6 is set unconditionally so the word is never zero -- HTIF
// reads a bare 1 as the pass encoding, and a probe whose every case failed must
// not be able to print SUCCESS. The word is reported as the HTIF failure
// number, so one run prints one line: `FAILURE: <mask>`.
//
//   bit 0  control: lr.w then sc.w, same word, nothing between
//   bit 1  trap ENTRY alone: the sc.w runs inside the ecall handler, no mret
//   bit 2  trap entry and mret: the sc.w runs after the handler returns
//   bit 3  mret alone: mepc is set by hand and mret executed, no trap
//   bit 4  a same-hart sw to the reserved word between the lr.w and the sc.w
//   bit 5  the sc.w addresses the next word up, not the reserved one
//
// Bit 0 is the anti-vacuity control. With it clear the model failed an SC that
// nothing could have invalidated, and no other bit means anything.
//
// The handler touches no memory, which is what keeps cases 1 and 2 about trap
// entry rather than about the handler's own stores.

#include "riscv_test.h"

// s0 accumulates the mask; folding a case in consumes t2, the sc.w result.
#define RECORD(bit)                                                          \
        seqz    t2, t2;                                                      \
        slli    t2, t2, bit;                                                 \
        or      s0, s0, t2;

RVTEST_RV64U
RVTEST_CODE_BEGIN

        li      s0, 0
        li      s2, 0
        la      s1, lockword

        la      t0, trap_handler
        csrw    mtvec, t0

        // 0 -- control.
        lr.w    t1, (s1)
        sc.w    t2, t1, (s1)
        RECORD(0)

        // 1 -- trap entry alone. The handler runs the sc.w and leaves its
        // result in t2, so nothing about mret is in this answer.
        li      s2, 1
        lr.w    t1, (s1)
        .option push
        .option norvc
        ecall
        .option pop
        RECORD(1)

        // 2 -- trap entry and mret together.
        lr.w    t1, (s1)
        .option push
        .option norvc
        ecall
        .option pop
        sc.w    t2, t1, (s1)
        RECORD(2)

        // 3 -- mret alone. Returning to the next instruction with no trap
        // behind it is legal in M-mode and is the only way to separate the two.
        lr.w    t1, (s1)
        la      t0, 1f
        csrw    mepc, t0
        mret
1:      sc.w    t2, t1, (s1)
        RECORD(3)

        // 4 -- a plain store from this hart to the reserved word.
        lr.w    t1, (s1)
        sw      t1, 0(s1)
        sc.w    t2, t1, (s1)
        RECORD(4)

        // 5 -- an sc.w one word up from the reservation.
        lr.w    t1, (s1)
        addi    t0, s1, 4
        sc.w    t2, t1, (t0)
        RECORD(5)

        ori     s0, s0, 0x40
        slli    s0, s0, 1
        ori     s0, s0, 1
        la      t0, tohost
        sw      x0, 4(t0)
        sw      s0, 0(t0)
1:      j       1b

        .align  2
trap_handler:
        .option push
        .option norvc
        beqz    s2, 2f
        sc.w    t2, t1, (s1)
        li      s2, 0
2:      csrr    t0, mepc
        addi    t0, t0, 4
        csrw    mepc, t0
        mret
        .option pop

        .data
RVTEST_DATA_BEGIN

        .align  2
        .global lockword
lockword:
        .word   0
        .word   0

RVTEST_DATA_END

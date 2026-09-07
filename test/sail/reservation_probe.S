// An LR/SC reservation probe for the Sail model ALONE. No core runs this, and it is
// deliberately not in test/asm: both sim legs glob that directory and would grade this
// against a core, which is exactly what it must not be.

#include "riscv_test.h"

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

        lr.w    t1, (s1)
        sc.w    t2, t1, (s1)
        RECORD(0)

        li      s2, 1
        lr.w    t1, (s1)
        .option push
        .option norvc
        ecall
        .option pop
        RECORD(1)

        lr.w    t1, (s1)
        .option push
        .option norvc
        ecall
        .option pop
        sc.w    t2, t1, (s1)
        RECORD(2)

        lr.w    t1, (s1)
        la      t0, 1f
        csrw    mepc, t0
        mret
1:      sc.w    t2, t1, (s1)
        RECORD(3)

        lr.w    t1, (s1)
        sw      t1, 0(s1)
        sc.w    t2, t1, (s1)
        RECORD(4)

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

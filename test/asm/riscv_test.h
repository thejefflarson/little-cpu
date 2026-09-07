// Local, minimal riscv_test.h.

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

// The upper word first and the verdict last, because there is no 64-bit store and Sail's
// HTIF fires on whichever half-write completes the pair: this way the verdict store is
// what stops the reference model, on the same instruction the cxxrtl runners stop on.
#ifdef BOARD_SUITE
// The HTIF store goes too, not just the spin.
#define RVTEST_PASS                                                          \
        li      TESTNUM, 1;                                                 \
        j       board_next

#define RVTEST_FAIL                                                          \
        sll     TESTNUM, TESTNUM, 1;                                        \
        or      TESTNUM, TESTNUM, 1;                                        \
        j       board_next
#else
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
#endif

// `tohost` must stay a full doubleword, 8-byte aligned: HTIF defines it as a 64-bit
// location and every consumer claims the whole doubleword at the symbol as an IO window.
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

#define TEST_AMO( testnum, inst, memval, rs2val, oldval, newval )            \
test_ ## testnum:                                                            \
        li      TESTNUM, testnum;                                           \
        la      x1, amodat;                                                 \
        li      x2, memval;                                                 \
        sw      x2, 0(x1);                                                  \
        li      x4, rs2val;                                                 \
        inst    x5, x4, (x1);                                               \
        li      x29, oldval;                                                \
        bne     x5, x29, fail;                                              \
        lw      x6, 0(x1);                                                  \
        li      x29, newval;                                                \
        bne     x6, x29, fail;

// Invoke after RVTEST_DATA_BEGIN so it lands in RAM, the only memory a load can reach on
// this Harvard core.
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
        .word   0;                                                          \
        .global trap_tval;                                                  \
trap_tval:                                                                   \
        .word   0;

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
        csrr    t1, mtval;                                                  \
        la      t0, trap_tval;                                              \
        sw      t1, 0(t0);                                                  \
        csrr    t1, mepc;                                                   \
        la      t0, trap_epc;                                               \
        sw      t1, 0(t0);                                                  \
        addi    t1, t1, 4;                                                  \
        csrw    mepc, t1;                                                   \
        mret;                                                               \
trap_handler_end:                                                            \
        .option pop;

#define RVTEST_TRAP_HANDLER_FATAL                                            \
        .align  2;                                                          \
trap_handler:                                                                \
        RVTEST_FAIL

#define RVTEST_INSTALL_TRAP_HANDLER                                          \
        la      t0, trap_handler;                                           \
        csrw    mtvec, t0;

#define MTIMER_BASE      0x00020000
#define MTIMECMP_OFFSET  8
#define MTIMECMPH_OFFSET 12

#define UART_BASE          0x00020020
#define UART_STATUS_OFFSET 4

#define UART_WAIT_IDLE(base)                                                 \
1:      lw      t1, UART_STATUS_OFFSET(base);                               \
        bnez    t1, 1b;

#define SPI_BASE            0x00020028
#define SPI_CONTROL_OFFSET  4
#define SPI_BUSY_BIT        0x100

#define MAP_TOP            0x00020030

#define SPI_WAIT_IDLE(base)                                                  \
1:      lw      t1, 0(base);                                                \
        andi    t1, t1, SPI_BUSY_BIT;                                       \
        bnez    t1, 1b;

#define SPI_SELECT(base, level)                                              \
        SPI_WAIT_IDLE(base);                                                 \
        li      t1, level;                                                   \
        sw      t1, SPI_CONTROL_OFFSET(base);

#define SPI_XFER(base, byte)                                                 \
        SPI_WAIT_IDLE(base);                                                 \
        li      t1, byte;                                                    \
        sw      t1, 0(base);                                                 \
        SPI_WAIT_IDLE(base);                                                 \
        lw      t2, 0(base);                                                \
        andi    t2, t2, 0xff;

#define SPI_XFER_REG(base, srcreg, dstreg)                                   \
        SPI_WAIT_IDLE(base);                                                 \
        sw      srcreg, 0(base);                                            \
        SPI_WAIT_IDLE(base);                                                 \
        lw      dstreg, 0(base);                                            \
        andi    dstreg, dstreg, 0xff;

#define RVTEST_TIMER_HANDLER                                                 \
        .align  2;                                                          \
timer_handler:                                                               \
        .option push;                                                       \
        .option norvc;                                                      \
        la      t0, irq_count;                                              \
        lw      t1, 0(t0);                                                  \
        addi    t1, t1, 1;                                                  \
        sw      t1, 0(t0);                                                  \
        csrr    t1, mcause;                                                 \
        la      t0, irq_cause;                                              \
        sw      t1, 0(t0);                                                  \
        csrr    t1, mepc;                                                   \
        la      t0, irq_epc;                                                \
        sw      t1, 0(t0);                                                  \
        csrr    t1, mstatus;                                                \
        la      t0, irq_mstatus;                                            \
        sw      t1, 0(t0);                                                  \
        li      t0, MTIMER_BASE;                                            \
        li      t1, -1;                                                     \
        sw      t1, MTIMECMP_OFFSET(t0);                                    \
        sw      t1, MTIMECMPH_OFFSET(t0);                                   \
        mret;                                                               \
timer_handler_end:                                                           \
        .option pop;

#define RVTEST_STICKY_TIMER_HANDLER                                          \
        .align  2;                                                          \
sticky_timer_handler:                                                        \
        .option push;                                                       \
        .option norvc;                                                      \
        la      t0, irq_count;                                              \
        lw      t1, 0(t0);                                                  \
        addi    t1, t1, 1;                                                  \
        sw      t1, 0(t0);                                                  \
        la      t0, irq_limit;                                              \
        lw      t0, 0(t0);                                                  \
        blt     t1, t0, 1f;                                                 \
        li      t0, MTIMER_BASE;                                            \
        li      t1, -1;                                                     \
        sw      t1, MTIMECMP_OFFSET(t0);                                    \
        sw      t1, MTIMECMPH_OFFSET(t0);                                   \
1:      mret;                                                               \
        .option pop;

#define RVTEST_TIMER_DATA                                                    \
        .align  2;                                                          \
        .global irq_count;                                                  \
irq_count:                                                                   \
        .word   0;                                                          \
        .global irq_cause;                                                  \
irq_cause:                                                                   \
        .word   0;                                                          \
        .global irq_epc;                                                    \
irq_epc:                                                                     \
        .word   0;                                                          \
        .global irq_mstatus;                                                \
irq_mstatus:                                                                 \
        .word   0;                                                          \
        .global irq_limit;                                                  \
irq_limit:                                                                   \
        .word   0;

#define RVTEST_INSTALL_TIMER_HANDLER                                         \
        la      t0, timer_handler;                                          \
        csrw    mtvec, t0;

#define RVTEST_INSTALL_STICKY_TIMER_HANDLER                                  \
        la      t0, sticky_timer_handler;                                   \
        csrw    mtvec, t0;

#define RVTEST_ARM_TIMER(delay)                                              \
        li      t0, MTIMER_BASE;                                            \
        lw      t1, 0(t0);                                                  \
        addi    t1, t1, delay;                                              \
        li      t2, -1;                                                     \
        sw      t2, MTIMECMP_OFFSET(t0);                                    \
        sw      x0, MTIMECMPH_OFFSET(t0);                                   \
        sw      t1, MTIMECMP_OFFSET(t0);

#define RVTEST_DISARM_TIMER                                                  \
        li      t0, MTIMER_BASE;                                            \
        li      t1, -1;                                                     \
        sw      t1, MTIMECMP_OFFSET(t0);                                    \
        sw      t1, MTIMECMPH_OFFSET(t0);

#define RVTEST_ENABLE_MTIE                                                   \
        li      t0, 0x80;                                                   \
        csrs    mie, t0;

#define RVTEST_DISABLE_MTIE                                                  \
        li      t0, 0x80;                                                   \
        csrc    mie, t0;

#define RVTEST_ENABLE_MIE                                                    \
        li      t0, 0x8;                                                    \
        csrs    mstatus, t0;

#define RVTEST_DISABLE_MIE                                                   \
        li      t0, 0x8;                                                    \
        csrc    mstatus, t0;

#endif

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

// The machine timer, four words: mtime, mtimeh, mtimecmp, mtimecmph. Assembly
// cannot read rtl/timer.v's `BASE`, so this is a copy of it and
// test/memmap_test.sh compares the two.
#define MTIMER_BASE      0x00020000
#define MTIMECMP_OFFSET  8
#define MTIMECMPH_OFFSET 12

// Constraint 1 does NOT apply to the handler below and its opposite does: an
// interrupt is taken BETWEEN instructions, so `mepc` holds an instruction that
// has not run and the handler must resume AT it, not past it. Advancing mepc
// here would skip a real instruction, which is the mistake this macro exists to
// stop each test making on its own.
//
// It clobbers t0 and t1, and it disarms the timer before returning: `mtip` is a
// level, so a handler that returns without moving `mtimecmp` is re-entered
// immediately and forever.
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

// The same, minus the disarm, until `irq_count` reaches `irq_limit`. MTIP is a
// LEVEL: the spec says the interrupt remains posted until mtimecmp becomes
// greater than mtime, and taking the trap does not clear it. So a handler that
// returns without moving mtimecmp is re-entered on the first cycle after `mret`
// that would otherwise have issued -- before the instruction at mepc runs. The
// limit is what stops that being a livelock in a test.
//
// Clobbers t0 and t1.
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

// Invoke after RVTEST_DATA_BEGIN, like RVTEST_TRAP_DATA, so it lands in RAM.
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

// Arm the timer to assert `delay` ticks from now, in the order the privileged
// spec's own sample code uses for RV32:
//
//     low half = all ones     -- no smaller than the OLD value
//     high half = new high    -- no smaller than the NEW value
//     low half = new low      -- the new value
//
// A 32-bit store touches one half, so the pair is briefly a mix of old and new.
// This order makes every intermediate at least as large as the smaller of the
// two, which is what stops a spurious interrupt being manufactured in the
// middle of the update. Writing the high half first does NOT: with the old low
// half small, the pair passes through {new high, old low}, and mtime may
// already be past it. test/asm/mtimer.S fires an interrupt that way on purpose.
//
// mtime's high half is zero for the whole of a program here unless the program
// writes it, so the arithmetic can be 32-bit. Clobbers t0, t1 and t2.
#define RVTEST_ARM_TIMER(delay)                                              \
        li      t0, MTIMER_BASE;                                            \
        lw      t1, 0(t0);                                                  \
        addi    t1, t1, delay;                                              \
        li      t2, -1;                                                     \
        sw      t2, MTIMECMP_OFFSET(t0);                                    \
        sw      x0, MTIMECMPH_OFFSET(t0);                                   \
        sw      t1, MTIMECMP_OFFSET(t0);

// mtimecmp resets to zero, so mtip is asserted out of reset -- harmless while
// both enables are clear, and the first thing any boot path has to undo.
// The low half first, per the sequence below; the third store would write the
// low half a second time with the same value, so it is left out.
// Clobbers t0 and t1.
#define RVTEST_DISARM_TIMER                                                  \
        li      t0, MTIMER_BASE;                                            \
        li      t1, -1;                                                     \
        sw      t1, MTIMECMP_OFFSET(t0);                                    \
        sw      t1, MTIMECMPH_OFFSET(t0);

// MTIE is bit 7 of mie; MIE is bit 3 of mstatus. Separately, because a test
// that masks one of them has to leave the other on.
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

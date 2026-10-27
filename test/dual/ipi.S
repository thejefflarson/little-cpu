// Hart 0 interrupts hart 1 through hart 1's msip word, and hart 1 answers through hart 0's:
// each handler records mcause in its own slot and clears its own msip.

#include "riscv_test.h"

#define POLL_LIMIT 256

RVTEST_RV64U
RVTEST_CODE_BEGIN

  la      t0, ipi_handler
  csrw    mtvec, t0
  li      t0, 8
  csrs    mie, t0
  csrs    mstatus, t0

  csrr    t0, mhartid
  bnez    t0, hart1

  li      a0, MSIP_BASE
  li      a1, 1
  sw      a1, 4(a0)

  la      a2, causes
  li      a3, POLL_LIMIT
0:
  lw      a4, 4(a2)
  bnez    a4, 1f
  addi    a3, a3, -1
  bnez    a3, 0b
1:
  li      a3, POLL_LIMIT
2:
  lw      a4, 0(a2)
  bnez    a4, 3f
  addi    a3, a3, -1
  bnez    a3, 2b
3:
  li      a6, 0x80000003
  li      a5, 0
  lw      a4, 0(a2)
  bne     a4, a6, 4f
  addi    a5, a5, 1
4:
  lw      a4, 4(a2)
  bne     a4, a6, 5f
  addi    a5, a5, 1
5:
  la      a0, total
  sw      a5, 0(a0)

  li      TESTNUM, 2
  li      a1, 2
  bne     a5, a1, fail
  RVTEST_PASS

fail:
  RVTEST_FAIL

hart1:
  la      a2, causes
  li      a3, POLL_LIMIT
6:
  lw      a4, 4(a2)
  bnez    a4, 7f
  addi    a3, a3, -1
  bnez    a3, 6b
7:
  li      a0, MSIP_BASE
  li      a1, 1
  sw      a1, 0(a0)
8:
  j       8b

  .option push
  .option norvc
  .align  2
ipi_handler:
  csrr    t0, mhartid
  slli    t0, t0, 2
  li      t1, MSIP_BASE
  add     t1, t1, t0
  sw      x0, 0(t1)
  csrr    t3, mcause
  la      t4, causes
  add     t4, t4, t0
  sw      t3, 0(t4)
  mret
  .option pop

RVTEST_CODE_END

RVTEST_DATA_BEGIN
  .align 2
  .global causes
causes:
  .word 0, 0
  .global total
total:
  .word 0
RVTEST_DATA_END

// KIND=0: straight-line code. KIND=1: a REPS-times loop inside one 16-byte block. KIND=2: a
// REPS-times loop with a load whose 32-bit addi straddles the block's end.
#include "riscv_test.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

#if KIND == 0
  .rept 80
  addi a0, a0, 1
  .endr
#endif
#if KIND == 1
  li a0, REPS
  .balign 16
loop:
  addi a0, a0, -1
  bnez a0, loop
#endif
#if KIND == 2
  li a0, REPS
  la a3, scratch
  .balign 16
  .rept 4
  c.nop
  .endr
loop:
  c.addi a0, -1
  c.nop
  c.nop
  .option push
  .option norvc
  addi a1, a1, 1
  .option pop
  c.lw a2, 0(a3)
  c.bnez a0, loop
#endif

  RVTEST_PASS

RVTEST_CODE_END

  .data
scratch: .word 0
RVTEST_DATA_BEGIN
RVTEST_DATA_END

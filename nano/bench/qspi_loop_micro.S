// KIND 0: straight-line code. 1: a REPS-times loop in one 16-byte block. 2: that loop with a load
// and a 32-bit addi straddling the block's end. 3: a fourteen-parcel loop across the block edge.
// 4: a loop of two 32-bit instructions in one block.
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
#if KIND == 3
  li a0, REPS
  .balign 16
loop:
  c.addi a0, -1
  .rept 12
  c.nop
  .endr
  c.bnez a0, loop
#endif
#if KIND == 4
  li a0, REPS
  .balign 16
  .option push
  .option norvc
loop:
  addi a0, a0, -1
  bnez a0, loop
  .option pop
#endif

  RVTEST_PASS

RVTEST_CODE_END

  .data
scratch: .word 0
RVTEST_DATA_BEGIN
RVTEST_DATA_END

// KIND=0 is branch-free straight-line code; KIND=1 is a REPS-times loop aligned so its
// whole body sits in one 16-byte block.
#include "riscv_test.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

#if KIND == 0
  .rept 80
  addi a0, a0, 1
  .endr
#else
  li a0, REPS
  .balign 16
loop:
  addi a0, a0, -1
  bnez a0, loop
#endif

  RVTEST_PASS

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN
RVTEST_DATA_END

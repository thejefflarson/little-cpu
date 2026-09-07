# See LICENSE for license details.

# A misaligned atomic, for all eleven encodings and at all three misalignments.

#include "riscv_test.h"
#include "test_macros.h"

# A misaligned AMO or sc.w: the cause, then that rd and the word are untouched.
#define TEST_MIS_AMO( testnum, inst, off, cause ) \
test_ ## testnum: \
    li  TESTNUM, testnum; \
    la  x1, trap_cause; \
    sw  x0, 0(x1); \
    la  x10, mdat; \
    addi x10, x10, off; \
    li  x11, 0x5a5; \
    li  x12, 0x3c3; \
    .option push; \
    .option norvc; \
    inst x11, x12, (x10); \
    .option pop; \
    la  x1, trap_cause; \
    lw  x13, 0(x1); \
    li  x29, cause; \
    bne x13, x29, fail; \
    li  x29, 0x5a5; \
    bne x11, x29, fail; \
    la  x1, mdat; \
    lw  x13, 0(x1); \
    li  x29, 0x0ff00ff0; \
    bne x13, x29, fail;

# The same for lr.w, which takes no rs2.
#define TEST_MIS_LR( testnum, off ) \
test_ ## testnum: \
    li  TESTNUM, testnum; \
    la  x1, trap_cause; \
    sw  x0, 0(x1); \
    la  x10, mdat; \
    addi x10, x10, off; \
    li  x11, 0x5a5; \
    .option push; \
    .option norvc; \
    lr.w x11, (x10); \
    .option pop; \
    la  x1, trap_cause; \
    lw  x13, 0(x1); \
    li  x29, 4; \
    bne x13, x29, fail; \
    li  x29, 0x5a5; \
    bne x11, x29, fail; \
    la  x1, mdat; \
    lw  x13, 0(x1); \
    li  x29, 0x0ff00ff0; \
    bne x13, x29, fail;

RVTEST_RV64U
RVTEST_CODE_BEGIN

  # mtvec resets to 0, which is _start -- install before faulting.
  RVTEST_INSTALL_TRAP_HANDLER

  # lr.w: cause 4, at each of the three misalignments a word address can have.

  TEST_MIS_LR(  2, 1 );
  TEST_MIS_LR(  3, 2 );
  TEST_MIS_LR(  4, 3 );

  TEST_MIS_AMO(  5, sc.w, 1, 6 );
  TEST_MIS_AMO(  6, sc.w, 2, 6 );
  TEST_MIS_AMO(  7, sc.w, 3, 6 );

  TEST_MIS_AMO(  8, amoswap.w, 2, 6 );
  TEST_MIS_AMO(  9, amoadd.w,  2, 6 );
  TEST_MIS_AMO( 10, amoxor.w,  2, 6 );
  TEST_MIS_AMO( 11, amoand.w,  2, 6 );
  TEST_MIS_AMO( 12, amoor.w,   2, 6 );
  TEST_MIS_AMO( 13, amomin.w,  2, 6 );
  TEST_MIS_AMO( 14, amomax.w,  2, 6 );
  TEST_MIS_AMO( 15, amominu.w, 1, 6 );
  TEST_MIS_AMO( 16, amomaxu.w, 3, 6 );

test_17:
  la    x10, mdat
  addi  x10, x10, 2
  li    x12, 0x3c3
  .option push
  .option norvc
mis_amo:
  amoadd.w x11, x12, (x10)
  .option pop

  la    x1, trap_epc
  lw    x13, 0(x1)
  la    x29, mis_amo
  li    TESTNUM, 17
  bne   x13, x29, fail

  TEST_CASE( 18, x13, 16, la x1, trap_count; lw x13, 0(x1); )

  TEST_CASE( 19, x11, 0x0ff00ff0, \
    la  x10, mdat; \
    li  x12, 0x00000001; \
    amoadd.w x11, x12, (x10); \
  )
  TEST_CASE( 20, x13, 0x0ff00ff1, la x1, mdat; lw x13, 0(x1); )
  TEST_CASE( 21, x13, 16, la x1, trap_count; lw x13, 0(x1); )

  TEST_PASSFAIL

  RVTEST_TRAP_HANDLER

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  RVTEST_TRAP_DATA

  .align 2
mdat:
  .word 0x0ff00ff0
  .word 0x00000000

RVTEST_DATA_END

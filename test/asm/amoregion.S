# See LICENSE for license details.

# An atomic outside the data RAM, which is the one region this platform answers one at.

#include "riscv_test.h"
#include "test_macros.h"

# Above the machine timer's four words and below the 1 MB the linker script may grow
# into.
#define UNMAPPED 0x00040000

# One refused atomic with an rs2: the cause, then that rd was not written.
#define TEST_REFUSED_AMO( testnum, inst, addr, cause ) \
test_ ## testnum: \
    li  TESTNUM, testnum; \
    la  x1, trap_cause; \
    sw  x0, 0(x1); \
    li  x10, addr; \
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
    bne x11, x29, fail;

#define TEST_REFUSED_LR( testnum, addr ) \
test_ ## testnum: \
    li  TESTNUM, testnum; \
    la  x1, trap_cause; \
    sw  x0, 0(x1); \
    li  x10, addr; \
    li  x11, 0x5a5; \
    .option push; \
    .option norvc; \
    lr.w x11, (x10); \
    .option pop; \
    la  x1, trap_cause; \
    lw  x13, 0(x1); \
    li  x29, 5; \
    bne x13, x29, fail; \
    li  x29, 0x5a5; \
    bne x11, x29, fail;

RVTEST_RV64U
RVTEST_CODE_BEGIN

  RVTEST_INSTALL_TRAP_HANDLER

  TEST_CASE(  2, a2, 0, \
    la  a0, rdat; \
    lr.w a1, (a0); \
    li  a5, 0x12345678; \
    sc.w a2, a5, (a0); \
  )
  TEST_CASE(  3, a4, 0x12345678, la a0, rdat; lw a4, 0(a0); )

  TEST_REFUSED_LR(   4, UNMAPPED );
  TEST_REFUSED_AMO(  5, sc.w,      UNMAPPED, 7 );
  TEST_REFUSED_AMO(  6, amoswap.w, UNMAPPED, 7 );
  TEST_REFUSED_AMO(  7, amoadd.w,  UNMAPPED, 7 );
  TEST_REFUSED_AMO(  8, amoxor.w,  UNMAPPED, 7 );
  TEST_REFUSED_AMO(  9, amoand.w,  UNMAPPED, 7 );
  TEST_REFUSED_AMO( 10, amoor.w,   UNMAPPED, 7 );
  TEST_REFUSED_AMO( 11, amomin.w,  UNMAPPED, 7 );
  TEST_REFUSED_AMO( 12, amomax.w,  UNMAPPED, 7 );
  TEST_REFUSED_AMO( 13, amominu.w, UNMAPPED, 7 );
  TEST_REFUSED_AMO( 14, amomaxu.w, UNMAPPED, 7 );

  TEST_CASE( 15, a4, 11, la a0, trap_count; lw a4, 0(a0); )

test_16:
  li    TESTNUM, 16
  li    x10, UNMAPPED
  li    x12, 0x3c3
  .option push
  .option norvc
refused_amo:
  amoadd.w x11, x12, (x10)
  .option pop
  la    x1, trap_epc
  lw    x13, 0(x1)
  la    x29, refused_amo
  bne   x13, x29, fail

  TEST_CASE( 17, a4, 12, la a0, trap_count; lw a4, 0(a0); )
  TEST_CASE( 18, a4, 12, la a0, trap_count; lw a4, 0(a0); )

  RVTEST_DISARM_TIMER

  TEST_REFUSED_LR(  19, MTIMER_BASE + MTIMECMP_OFFSET );
  TEST_REFUSED_AMO( 20, sc.w,     MTIMER_BASE + MTIMECMP_OFFSET, 7 );
  TEST_REFUSED_AMO( 21, amoadd.w, MTIMER_BASE + MTIMECMP_OFFSET, 7 );

  TEST_CASE( 22, a4, -1, \
    li  a0, MTIMER_BASE + MTIMECMP_OFFSET; \
    lw  a4, 0(a0); \
  )

  TEST_REFUSED_LR(  23, 0x00000000 );
  TEST_REFUSED_AMO( 24, amoadd.w, 0x00000000, 7 );

  TEST_CASE( 25, a4, 17, \
    li  a0, 0x00000000; \
    lw  a1, 0(a0); \
    la  a0, trap_count; \
    lw  a4, 0(a0); \
  )

  TEST_CASE( 26, a2, 0, \
    la  a0, rdat; \
    lr.w a1, (a0); \
    li  a5, 0x0000c0de; \
    sc.w a2, a5, (a0); \
  )
  TEST_CASE( 27, a4, 0x0000c0de, la a0, rdat; lw a4, 0(a0); )

  TEST_CASE( 28, a4, 17, la a0, trap_count; lw a4, 0(a0); )

  TEST_PASSFAIL

  RVTEST_TRAP_HANDLER

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  RVTEST_TRAP_DATA

  .align 2
rdat:
  .word 0x0ff00ff0

RVTEST_DATA_END

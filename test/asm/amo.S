# See LICENSE for license details.

# The nine AMO functions, each against the word it must leave behind as well as the word
# it must return.

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  # The nine functions.

  TEST_AMO(  2, amoswap.w, 0x12345678, 0xdeadbeef, 0x12345678, 0xdeadbeef );
  TEST_AMO(  3, amoadd.w,  0x00000010, 0x00000022, 0x00000010, 0x00000032 );
  TEST_AMO(  4, amoxor.w,  0x0ff00ff0, 0x00ff00ff, 0x0ff00ff0, 0x0f0f0f0f );
  TEST_AMO(  5, amoand.w,  0x0ff00ff0, 0x00ff00ff, 0x0ff00ff0, 0x00f000f0 );
  TEST_AMO(  6, amoor.w,   0x0ff00ff0, 0x00ff00ff, 0x0ff00ff0, 0x0fff0fff );
  TEST_AMO(  7, amomin.w,  0x00000005, 0x00000003, 0x00000005, 0x00000003 );
  TEST_AMO(  8, amomax.w,  0x00000005, 0x00000003, 0x00000005, 0x00000005 );
  TEST_AMO(  9, amominu.w, 0x00000005, 0x00000003, 0x00000005, 0x00000003 );
  TEST_AMO( 10, amomaxu.w, 0x00000005, 0x00000003, 0x00000005, 0x00000005 );

  TEST_AMO( 11, amoadd.w,  0xffffffff, 0x00000001, 0xffffffff, 0x00000000 );
  TEST_AMO( 12, amoadd.w,  0x7fffffff, 0x00000001, 0x7fffffff, 0x80000000 );

  TEST_AMO( 13, amoadd.w.aq,   0x00000010, 0x00000022, 0x00000010, 0x00000032 );
  TEST_AMO( 14, amoadd.w.rl,   0x00000010, 0x00000022, 0x00000010, 0x00000032 );
  TEST_AMO( 15, amoadd.w.aqrl, 0x00000010, 0x00000022, 0x00000010, 0x00000032 );

  # rd == rs2: the returned word overwrites the operand that was added to it.
  TEST_CASE( 16, x4, 0x00000010, \
    la  x1, amodat; \
    li  x2, 0x00000010; \
    sw  x2, 0(x1); \
    li  x4, 0x00000022; \
    amoadd.w x4, x4, (x1); \
  )
  TEST_CASE( 17, x6, 0x00000032, la x1, amodat; lw x6, 0(x1); )

  TEST_CASE( 18, x1, 0x00000010, \
    la  x1, amodat; \
    li  x2, 0x00000010; \
    sw  x2, 0(x1); \
    li  x4, 0x00000022; \
    amoadd.w x1, x4, (x1); \
  )
  TEST_CASE( 19, x6, 0x00000032, la x1, amodat; lw x6, 0(x1); )

  TEST_CASE( 20, x0, 0, \
    la  x1, amodat; \
    li  x2, 0x00000010; \
    sw  x2, 0(x1); \
    li  x4, 0x00000022; \
    amoadd.w x0, x4, (x1); \
  )
  TEST_CASE( 21, x6, 0x00000032, la x1, amodat; lw x6, 0(x1); )

  TEST_CASE( 22, x7, 0x00000042, \
    la  x1, amodat; \
    li  x2, 0x00000010; \
    sw  x2, 0(x1); \
    li  x4, 0x00000022; \
    amoadd.w x5, x4, (x1); \
    addi x7, x5, 0x32; \
  )

  TEST_CASE( 23, x7, 0x00000032, \
    la  x1, amodat; \
    li  x2, 0x00000010; \
    sw  x2, 0(x1); \
    li  x4, 0x00000022; \
    amoadd.w x5, x4, (x1); \
    lw  x7, 0(x1); \
  )

  TEST_CASE( 24, x8, 0x00000021, \
    la  x1, amodat; \
    li  x2, 0x00000010; \
    sw  x2, 0(x1); \
    li  x4, 0x00000011; \
    amoadd.w x5, x4, (x1); \
    amoadd.w x8, x4, (x1); \
  )
  TEST_CASE( 25, x6, 0x00000032, la x1, amodat; lw x6, 0(x1); )

  TEST_CASE( 26, x11, 0x00000007, \
    la  x1, amodat; \
    li  x2, 0x00000010; \
    sw  x2, 0(x1); \
    li  x9, 63; \
    li  x10, 9; \
    div x11, x9, x10; \
    li  x4, 0x00000022; \
    amoadd.w x8, x4, (x1); \
  )
  TEST_CASE( 27, x8, 0x00000010, nop; )
  TEST_CASE( 28, x6, 0x00000032, la x1, amodat; lw x6, 0(x1); )

  TEST_PASSFAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

  .align 2
amodat:
  .word 0x00000000

RVTEST_DATA_END

# See LICENSE for license details.

# lr.w and sc.w, and the five events that decide whether a store-conditional stores: no
# reservation at all, this hart writing the reserved word, another hart writing it, a
# second reservation replacing the first, and the store-conditional itself.

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  // A store-conditional with no reservation behind it.

  TEST_CASE(  2, a2, 1, \
    la  a0, ldat; \
    li  a5, 0xbad0bad0; \
    sc.w a2, a5, (a0); \
  )
  TEST_CASE(  3, a4, 0x0ff00ff0, la a0, ldat; lw a4, 0(a0); )

  // lr.w reads the word, and the sc.w behind it stores rs2.

  TEST_CASE(  4, a1, 0x0ff00ff0, la a0, ldat; lr.w a1, (a0); )

  TEST_CASE(  5, a2, 0, li a5, 0x12345678; sc.w a2, a5, (a0); )
  TEST_CASE(  6, a4, 0x12345678, la a0, ldat; lw a4, 0(a0); )

  // The store-conditional consumed the reservation, so the next one fails with no other
  // event in between.
  TEST_CASE(  7, a2, 1, li a5, 0xbad0bad0; sc.w a2, a5, (a0); )
  TEST_CASE(  8, a4, 0x12345678, la a0, ldat; lw a4, 0(a0); )

  TEST_CASE(  9, a2, 1, \
    la  a0, ldat; \
    lr.w a1, (a0); \
    li  a5, 0x0000f00d; \
    sw  a5, 0(a0); \
    li  a6, 0xbad0bad0; \
    sc.w a2, a6, (a0); \
  )
  TEST_CASE( 10, a4, 0x0000f00d, la a0, ldat; lw a4, 0(a0); )

  TEST_CASE( 11, a2, 1, \
    la  a0, ldat; \
    lr.w a1, (a0); \
    li  a5, 0x77; \
    sb  a5, 0(a0); \
    li  a6, 0xbad0bad0; \
    sc.w a2, a6, (a0); \
  )
  TEST_CASE( 12, a4, 0x0000f077, la a0, ldat; lw a4, 0(a0); )

  TEST_CASE( 13, a2, 1, \
    la  a0, ldat; \
    lr.w a1, (a0); \
    li  a5, 0x1234; \
    sh  a5, 0(a0); \
    li  a6, 0xbad0bad0; \
    sc.w a2, a6, (a0); \
  )
  TEST_CASE( 14, a4, 0x00001234, la a0, ldat; lw a4, 0(a0); )

  TEST_CASE( 15, a2, 1, \
    la  a0, ldat; \
    lr.w a1, (a0); \
    li  a5, 0x1000; \
    amoadd.w a7, a5, (a0); \
    li  a6, 0xbad0bad0; \
    sc.w a2, a6, (a0); \
  )
  TEST_CASE( 16, a4, 0x00002234, la a0, ldat; lw a4, 0(a0); )

  TEST_CASE( 17, a2, 0, \
    la  a0, ldat; \
    la  a3, ldat2; \
    lr.w a1, (a0); \
    li  a5, 0x5a5a5a5a; \
    sw  a5, 0(a3); \
    li  a6, 0x0000cafe; \
    sc.w a2, a6, (a0); \
  )
  TEST_CASE( 18, a4, 0x0000cafe, la a0, ldat; lw a4, 0(a0); )

  TEST_CASE( 19, a2, 0, \
    la  a0, ldat; \
    lr.w a1, (a0); \
    lw  a5, 0(a0); \
    li  a6, 0x0000beef; \
    sc.w a2, a6, (a0); \
  )
  TEST_CASE( 20, a4, 0x0000beef, la a0, ldat; lw a4, 0(a0); )

  TEST_CASE( 21, a2, 1, \
    la  a0, ldat; \
    la  a3, ldat2; \
    li  a5, 0x11112222; \
    sw  a5, 0(a3); \
    lr.w a1, (a0); \
    li  a6, 0xbad0bad0; \
    sc.w a2, a6, (a3); \
  )
  TEST_CASE( 22, a4, 0x11112222, la a3, ldat2; lw a4, 0(a3); )
  TEST_CASE( 23, a4, 0x0000beef, la a0, ldat; lw a4, 0(a0); )

  TEST_CASE( 24, a2, 0, \
    la  a0, ldat; \
    la  a3, ldat2; \
    lr.w a1, (a0); \
    lr.w a1, (a3); \
    li  a6, 0x0000d00d; \
    sc.w a2, a6, (a3); \
  )
  TEST_CASE( 25, a4, 0x0000d00d, la a3, ldat2; lw a4, 0(a3); )
  TEST_CASE( 26, a2, 1, \
    la  a0, ldat; \
    li  a6, 0xbad0bad0; \
    sc.w a2, a6, (a0); \
  )
  TEST_CASE( 27, a4, 0x0000beef, la a0, ldat; lw a4, 0(a0); )

  TEST_CASE( 28, a7, 1, \
    la  a0, ldat; \
    lr.w a1, (a0); \
    li  a6, 0x0000abcd; \
    sc.w a2, a6, (a0); \
    addi a7, a2, 1; \
  )
  TEST_CASE( 29, a4, 0x0000abcd, la a0, ldat; lw a4, 0(a0); )

  TEST_CASE( 30, a7, 2, \
    la  a0, ldat; \
    li  a6, 0xbad0bad0; \
    sc.w a2, a6, (a0); \
    addi a7, a2, 1; \
  )
  TEST_CASE( 31, a4, 0x0000abcd, la a0, ldat; lw a4, 0(a0); )

  TEST_CASE( 32, a0, 0, \
    la  a0, ldat; \
    lr.w a1, (a0); \
    li  a6, 0x00005555; \
    sc.w a0, a6, (a0); \
  )
  TEST_CASE( 33, a4, 0x00005555, la a0, ldat; lw a4, 0(a0); )

  TEST_CASE( 34, a6, 0, \
    la  a0, ldat; \
    lr.w a1, (a0); \
    li  a6, 0x00006666; \
    sc.w a6, a6, (a0); \
  )
  TEST_CASE( 35, a4, 0x00006666, la a0, ldat; lw a4, 0(a0); )

  TEST_CASE( 36, a2, 0, \
    la  a0, ldat; \
    li  a5, 63; \
    li  a6, 9; \
    div a7, a5, a6; \
    lr.w a1, (a0); \
    li  a6, 0x00007777; \
    sc.w a2, a6, (a0); \
  )
  TEST_CASE( 37, a7, 7, nop; )
  TEST_CASE( 38, a4, 0x00007777, la a0, ldat; lw a4, 0(a0); )

  TEST_PASSFAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

  .align 2
ldat:
  .word 0x0ff00ff0
ldat2:
  .word 0x00000000

RVTEST_DATA_END

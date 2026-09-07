#include "riscv_test.h"
#include "test_macros.h"

// Loads and stores that land in .text, where the fetch is reading.

RVTEST_CODE_BEGIN

  // Reads, both parities.

  TEST_CASE( 2, a4, 0x0000babe, la a3, tw_even; lw a4, 0(a3); )
  TEST_CASE( 3, a4, 0x0000cafe, la a3, tw_odd;  lw a4, 0(a3); )

  TEST_CASE( 4, a4, 0xcafe, la a3, tw_odd; lhu a4, 0(a3); )
  TEST_CASE( 5, a4, 0xbe,   la a3, tw_even; lbu a4, 0(a3); )

  la    a3, tw_scratch_even
  li    a4, 0x11112222
  sw    a4, 0(a3)
  lw    a5, 0(a3)
  TEST_CASE( 6, a5, 0x11112222, nop; )

  la    a3, tw_scratch_odd
  li    a4, 0x33334444
  sw    a4, 0(a3)
  lw    a5, 0(a3)
  TEST_CASE( 7, a5, 0x33334444, nop; )

  la    a3, tw_scratch_odd
  li    a4, 0xa5
  sb    a4, 1(a3)
  lw    a5, 0(a3)
  TEST_CASE( 8, a5, 0x3333a544, nop; )

  la    a3, tw_scratch_even
  li    a4, 0xbeef
  sh    a4, 2(a3)
  lw    a5, 0(a3)
  TEST_CASE( 9, a5, 0xbeef2222, nop; )

  la    a3, tw_scratch_even
  li    a4, 0x0f0f0f0f
  sw    a4, 0(a3)
  lw    a5, 4(a3)
  TEST_CASE( 10, a5, 0x3333a544, nop; )
  TEST_CASE( 11, a5, 0x0f0f0f0f, la a3, tw_scratch_even; lw a5, 0(a3); )

  la    a3, rdat
  lw    a4, 0(a3)
  li    a5, -1
  sw    a5, 0(a3)
  la    a3, tw_even
  lw    a4, 0(a3)
  TEST_CASE( 12, a4, 0x0000babe, nop; )

  TEST_CASE( 13, a4, -1, la a3, rdat; lw a4, 0(a3); )

  TEST_PASSFAIL

  .align 3
tw_even:
  .word 0x0000babe
tw_odd:
  .word 0x0000cafe
tw_scratch_even:
  .word 0
tw_scratch_odd:
  .word 0

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

  .align 2
rdat:
  .word 0x0000f00d

RVTEST_DATA_END

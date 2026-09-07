#include "riscv_test.h"
#include "test_macros.h"

// Loads and stores that land in .text, where the fetch is reading. The
// instruction memory has one read port per bank, so a data access there takes
// the port and the fetch behind it is dropped and asked for again.
//
// Neighbouring words live in different banks, so every access below is done at
// both word-address parities: the cells start at an 8-byte boundary, which
// makes tw_even and tw_scratch_even even word addresses and the two `_odd`
// cells odd ones. A bank select that ignored the low bit would answer the
// wrong cell rather than fail to answer.
//
// The cells sit after TEST_PASSFAIL's infinite loop, so nothing ever executes
// them, and they stay well under 8 KB -- the size of the ROM on the part,
// which is smaller than the one the simulation builds. Past that boundary the
// two would disagree about which addresses are text at all.

RVTEST_CODE_BEGIN

  # Reads, both parities.

  TEST_CASE( 2, a4, 0x0000babe, la a3, tw_even; lw a4, 0(a3); )
  TEST_CASE( 3, a4, 0x0000cafe, la a3, tw_odd;  lw a4, 0(a3); )

  # Narrower than a word, so the accessor picks bytes out of a text answer the same way
  # it does out of a RAM one.
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

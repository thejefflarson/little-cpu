# See LICENSE for license details.

#*****************************************************************************
# lrsc.S
#-----------------------------------------------------------------------------
#
# lr.w and sc.w, and the five events that decide whether a store-conditional
# stores: no reservation at all, this hart writing the reserved word, a write
# somewhere else, a second reservation replacing the first, and the
# store-conditional itself.
#
# A FAILED sc.w MUST LEAVE THE WORD ALONE, and that is the half no register
# value can show on its own -- rd is 1 either way a wrong design might reach it.
# So every case here loads the word back afterwards. test/cosim.cc compares
# architectural registers and never compares memory, and riscv-formal ships no
# spec model for lr.w/sc.w at the pinned SHA, so a store that happened and
# should not have is invisible to every oracle unless the program reads it.
#
# The value this core writes for a failure is 1. The specification asks only for
# a nonzero code, so an assertion of exactly 1 is an assertion about THIS
# implementation; it is written that way on purpose, because a core that started
# returning some other nonzero value has changed something nobody decided.
#

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  #-------------------------------------------------------------
  # A store-conditional with no reservation behind it. This is first in the
  # program on purpose: nothing before it has executed an lr.w, so there is no
  # stale reservation for it to be passing against.
  #-------------------------------------------------------------

  TEST_CASE(  2, a2, 1, \
    la  a0, ldat; \
    li  a5, 0xbad0bad0; \
    sc.w a2, a5, (a0); \
  )
  TEST_CASE(  3, a4, 0x0ff00ff0, la a0, ldat; lw a4, 0(a0); )

  #-------------------------------------------------------------
  # lr.w reads the word, and the sc.w behind it stores rs2.
  #-------------------------------------------------------------

  TEST_CASE(  4, a1, 0x0ff00ff0, la a0, ldat; lr.w a1, (a0); )

  TEST_CASE(  5, a2, 0, li a5, 0x12345678; sc.w a2, a5, (a0); )
  TEST_CASE(  6, a4, 0x12345678, la a0, ldat; lw a4, 0(a0); )

  # The store-conditional consumed the reservation, so the next one fails with
  # no other event in between.
  TEST_CASE(  7, a2, 1, li a5, 0xbad0bad0; sc.w a2, a5, (a0); )
  TEST_CASE(  8, a4, 0x12345678, la a0, ldat; lw a4, 0(a0); )

  #-------------------------------------------------------------
  # A plain store from this hart to the reserved word clears the reservation.
  # One hart is all there is, so this is the only invalidation source that can
  # be exercised at all.
  #-------------------------------------------------------------

  TEST_CASE(  9, a2, 1, \
    la  a0, ldat; \
    lr.w a1, (a0); \
    li  a5, 0x0000f00d; \
    sw  a5, 0(a0); \
    li  a6, 0xbad0bad0; \
    sc.w a2, a6, (a0); \
  )
  # The word holds what the plain store put there, not what the sc.w carried.
  TEST_CASE( 10, a4, 0x0000f00d, la a0, ldat; lw a4, 0(a0); )

  # A byte store to the reserved word counts: the set is the whole word.
  TEST_CASE( 11, a2, 1, \
    la  a0, ldat; \
    lr.w a1, (a0); \
    li  a5, 0x77; \
    sb  a5, 0(a0); \
    li  a6, 0xbad0bad0; \
    sc.w a2, a6, (a0); \
  )
  TEST_CASE( 12, a4, 0x0000f077, la a0, ldat; lw a4, 0(a0); )

  # A halfword store, for the same reason.
  TEST_CASE( 13, a2, 1, \
    la  a0, ldat; \
    lr.w a1, (a0); \
    li  a5, 0x1234; \
    sh  a5, 0(a0); \
    li  a6, 0xbad0bad0; \
    sc.w a2, a6, (a0); \
  )
  TEST_CASE( 14, a4, 0x00001234, la a0, ldat; lw a4, 0(a0); )

  # An AMO to the reserved word is a write of this hart's too.
  TEST_CASE( 15, a2, 1, \
    la  a0, ldat; \
    lr.w a1, (a0); \
    li  a5, 0x1000; \
    amoadd.w a7, a5, (a0); \
    li  a6, 0xbad0bad0; \
    sc.w a2, a6, (a0); \
  )
  TEST_CASE( 16, a4, 0x00002234, la a0, ldat; lw a4, 0(a0); )

  #-------------------------------------------------------------
  # ...and the other direction, which is the half a design clearing on EVERY
  # store would pass the cases above while failing.
  #-------------------------------------------------------------

  # A store to a different word leaves the reservation alone.
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

  # A LOAD from the reserved word leaves it alone as well.
  TEST_CASE( 19, a2, 0, \
    la  a0, ldat; \
    lr.w a1, (a0); \
    lw  a5, 0(a0); \
    li  a6, 0x0000beef; \
    sc.w a2, a6, (a0); \
  )
  TEST_CASE( 20, a4, 0x0000beef, la a0, ldat; lw a4, 0(a0); )

  #-------------------------------------------------------------
  # The address the reservation is held at. This core reserves ONE aligned word
  # and compares word addresses, which is the specification's minimum set.
  #-------------------------------------------------------------

  # A store-conditional at the next word up finds nothing reserved there.
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
  # ...and the reserved word is untouched by that attempt too.
  TEST_CASE( 23, a4, 0x0000beef, la a0, ldat; lw a4, 0(a0); )

  # A second lr.w moves the reservation: the store-conditional that matches the
  # SECOND one succeeds.
  TEST_CASE( 24, a2, 0, \
    la  a0, ldat; \
    la  a3, ldat2; \
    lr.w a1, (a0); \
    lr.w a1, (a3); \
    li  a6, 0x0000d00d; \
    sc.w a2, a6, (a3); \
  )
  TEST_CASE( 25, a4, 0x0000d00d, la a3, ldat2; lw a4, 0(a3); )
  # ...and the first word is no longer reserved.
  TEST_CASE( 26, a2, 1, \
    la  a0, ldat; \
    li  a6, 0xbad0bad0; \
    sc.w a2, a6, (a0); \
  )
  TEST_CASE( 27, a4, 0x0000beef, la a0, ldat; lw a4, 0(a0); )

  #-------------------------------------------------------------
  # The pair through the pipeline. A store-conditional is the one store that
  # also writes a register, so its rd has to be visible to the scoreboard for
  # exactly as long as a load's.
  #-------------------------------------------------------------

  # rd consumed by the very next instruction.
  TEST_CASE( 28, a7, 1, \
    la  a0, ldat; \
    lr.w a1, (a0); \
    li  a6, 0x0000abcd; \
    sc.w a2, a6, (a0); \
    addi a7, a2, 1; \
  )
  TEST_CASE( 29, a4, 0x0000abcd, la a0, ldat; lw a4, 0(a0); )

  # rd of a FAILED one, consumed the same way.
  TEST_CASE( 30, a7, 2, \
    la  a0, ldat; \
    li  a6, 0xbad0bad0; \
    sc.w a2, a6, (a0); \
    addi a7, a2, 1; \
  )
  TEST_CASE( 31, a4, 0x0000abcd, la a0, ldat; lw a4, 0(a0); )

  # sc.w naming the address register as its destination.
  TEST_CASE( 32, a0, 0, \
    la  a0, ldat; \
    lr.w a1, (a0); \
    li  a6, 0x00005555; \
    sc.w a0, a6, (a0); \
  )
  TEST_CASE( 33, a4, 0x00005555, la a0, ldat; lw a4, 0(a0); )

  # sc.w naming its own data register as its destination.
  TEST_CASE( 34, a6, 0, \
    la  a0, ldat; \
    lr.w a1, (a0); \
    li  a6, 0x00006666; \
    sc.w a6, a6, (a0); \
  )
  TEST_CASE( 35, a4, 0x00006666, la a0, ldat; lw a4, 0(a0); )

  # A divide in flight holds the pair in decode before the executor takes it.
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

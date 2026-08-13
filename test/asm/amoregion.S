# See LICENSE for license details.

#*****************************************************************************
# amoregion.S
#-----------------------------------------------------------------------------
#
# An atomic outside the data RAM, where the platform refuses the reservation.
#
# WHAT THE HARDWARE DOES AND WHY. rtl/memory.v exports the range test it already
# computes, rtl/littlecpu.v routes it in, and rtl/accessor.v sets no reservation
# on an lr.w where it is low. So a store-conditional anywhere but the data RAM
# always fails and issues no transaction at all. A spurious failure is
# architecturally permitted anywhere, and the eventual-success guarantee
# attaches only to a region whose reservability attribute claims it -- which is
# the one region here that does. The outcome the specification has no room for
# is the other one: an sc.w reporting 0 for a store that went nowhere.
#
# THE REST OF THE DEVIATION IS UNCHANGED AND IS ASSERTED HERE AS WHAT IT IS. An
# address no memory answers still reads as zero and is still written nowhere,
# silently, for loads, stores and AMOs alike -- the decode-time region test that
# would raise causes 5 and 7 instead was measured at four logic levels in the
# fetch loop and declined. So the AMO cases below assert a documented deviation
# rather than a specification requirement, and they are here so that a change to
# it is a change to a graded program rather than a discovery.
#
# THIS PROGRAM IS THE SUITE'S ONE CO-SIMULATION DIVERGENCE FOR THE A EXTENSION.
# The Sail model has no third setting: a region is RsrvEventual, where the sc.w
# below SUCCEEDS, or RsrvNone, where the lr.w raises a load access fault the
# core does not raise. Measured both ways; test/COSIM_EXPECTED_FAIL carries the
# entry and the paragraph. Everything about the A extension that IS
# co-simulable was kept out of this file for that reason.
#

#include "riscv_test.h"
#include "test_macros.h"

# Above the machine timer's four words and below the 1 MB the linker script may
# grow into. Nothing on this bus answers it.
#define UNMAPPED 0x00040000

RVTEST_RV64U
RVTEST_CODE_BEGIN

  #-------------------------------------------------------------
  # The control. The same sequence inside the data RAM succeeds, so a failure
  # below is the region attribute and not the mechanism.
  #-------------------------------------------------------------

  TEST_CASE(  2, a2, 0, \
    la  a0, rdat; \
    lr.w a1, (a0); \
    li  a5, 0x12345678; \
    sc.w a2, a5, (a0); \
  )
  TEST_CASE(  3, a4, 0x12345678, la a0, rdat; lw a4, 0(a0); )

  #-------------------------------------------------------------
  # An address no memory answers. The lr.w reads zero and takes no reservation,
  # so the sc.w behind it fails and stores nothing.
  #-------------------------------------------------------------

  TEST_CASE(  4, a1, 0, li a0, UNMAPPED; lr.w a1, (a0); )

  TEST_CASE(  5, a2, 1, li a5, 0x5a5a5a5a; sc.w a2, a5, (a0); )
  TEST_CASE(  6, a4, 0, li a0, UNMAPPED; lw a4, 0(a0); )

  # A second lr.w there does not accumulate a reservation either.
  TEST_CASE(  7, a2, 1, \
    li  a0, UNMAPPED; \
    lr.w a1, (a0); \
    lr.w a1, (a0); \
    li  a5, 0x5a5a5a5a; \
    sc.w a2, a5, (a0); \
  )
  TEST_CASE(  8, a4, 0, li a0, UNMAPPED; lw a4, 0(a0); )

  # ...and a reservation taken in RAM is not honoured at an address outside it,
  # which is the same word-address comparison test/asm/lrsc.S makes inside RAM.
  TEST_CASE(  9, a2, 1, \
    la  a3, rdat; \
    lr.w a1, (a3); \
    li  a0, UNMAPPED; \
    li  a5, 0x5a5a5a5a; \
    sc.w a2, a5, (a0); \
  )
  TEST_CASE( 10, a4, 0, li a0, UNMAPPED; lw a4, 0(a0); )
  TEST_CASE( 11, a4, 0x12345678, la a0, rdat; lw a4, 0(a0); )

  #-------------------------------------------------------------
  # An AMO out there is not refused -- it executes, reads zero, and writes its
  # result nowhere. That is the recorded deviation, one shape wider than a plain
  # load and store, and it is bounded by the measurement that declined the
  # decode-time region test rather than by an argument.
  #-------------------------------------------------------------

  TEST_CASE( 12, a4, 0, \
    li  a0, UNMAPPED; \
    li  a3, 0x00000007; \
    amoadd.w a4, a3, (a0); \
  )
  TEST_CASE( 13, a4, 0, li a0, UNMAPPED; lw a4, 0(a0); )

  #-------------------------------------------------------------
  # The machine timer's four words get a refused reservation for free: they are
  # not the data RAM, so rtl/memory.v's range test is low for them. The sc.w
  # below therefore leaves mtimecmp exactly as the disarm left it, which is a
  # readable device register rather than a hole in the map.
  #-------------------------------------------------------------

  RVTEST_DISARM_TIMER

  TEST_CASE( 14, a1, -1, \
    li  a0, MTIMER_BASE + MTIMECMP_OFFSET; \
    lr.w a1, (a0); \
  )
  TEST_CASE( 15, a2, 1, sc.w a2, x0, (a0); )
  TEST_CASE( 16, a4, -1, \
    li  a0, MTIMER_BASE + MTIMECMP_OFFSET; \
    lw  a4, 0(a0); \
  )

  #-------------------------------------------------------------
  # ...and the data RAM is still reservable afterwards, so nothing above left
  # the reservation state stuck.
  #-------------------------------------------------------------

  TEST_CASE( 17, a2, 0, \
    la  a0, rdat; \
    lr.w a1, (a0); \
    li  a5, 0x0000c0de; \
    sc.w a2, a5, (a0); \
  )
  TEST_CASE( 18, a4, 0x0000c0de, la a0, rdat; lw a4, 0(a0); )

  TEST_PASSFAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

  .align 2
rdat:
  .word 0x0ff00ff0

RVTEST_DATA_END

# See LICENSE for license details.

#*****************************************************************************
# hpm.S
#-----------------------------------------------------------------------------
#
# The 87 hardware performance monitor addresses: mhpmcounter3-31 (0xB03-0xB1F),
# mhpmcounter3h-31h (0xB83-0xB9F) and mhpmevent3-31 (0x323-0x33F). The RV32
# privileged specification's machine-mode performance-monitor section asks for
# all 29 counters and their event selectors -- "All counters should be
# implemented" -- and in the same breath permits the cheap way out: "a legal
# implementation is to make both the counter and its corresponding event
# selector be read-only 0". That is what rtl/csrs.v does. There is no counter
# and no event logic behind any of these; the whole implementation is three
# address ranges answering zero.
#
# So the only architectural fact a program can observe is that they read zero
# and DO NOT TRAP, which is what this file asserts. All three ranges are
# recognised by an address compare, so both ends of each range are read here --
# a range that stops one address short is exactly the mistake an interior
# sample cannot see. test/csr_tb.v carries the other half, the addresses just
# outside each range that must still be illegal.
#
# THE LAST CASE IS THE RED DIRECTION FOR ALL THE OTHERS. Every "no trap"
# assertion above it is a trap_count that stayed at zero, and a handler that
# never ran would satisfy every one of them. 0xB20 is one past mhpmcounter31
# and must raise illegal instruction, so the counter is seen to move.
#

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  # mtvec resets to 0, which is _start — install before faulting. The RECORDING
  # handler, not the fatal one: what this file reports is a trap count, so a
  # trap has to be survivable.
  RVTEST_INSTALL_TRAP_HANDLER

  #-------------------------------------------------------------
  # mhpmcounter3-31: both ends of the range and one in the middle.
  #-------------------------------------------------------------

  .option push
  .option norvc

  TEST_CASE(  2, a0, 0, csrr a0, 0xB03; )
  TEST_CASE(  3, a0, 0, csrr a0, 0xB11; )
  TEST_CASE(  4, a0, 0, csrr a0, 0xB1F; )

  #-------------------------------------------------------------
  # mhpmcounter3h-31h, the upper halves of the same 64-bit counters.
  #-------------------------------------------------------------

  TEST_CASE(  5, a0, 0, csrr a0, 0xB83; )
  TEST_CASE(  6, a0, 0, csrr a0, 0xB91; )
  TEST_CASE(  7, a0, 0, csrr a0, 0xB9F; )

  #-------------------------------------------------------------
  # mhpmevent3-31, the event selectors.
  #-------------------------------------------------------------

  TEST_CASE(  8, a0, 0, csrr a0, 0x323; )
  TEST_CASE(  9, a0, 0, csrr a0, 0x331; )
  TEST_CASE( 10, a0, 0, csrr a0, 0x33F; )

  TEST_CASE( 11, a4, 0, la a3, trap_count; lw a4, 0(a3); )

  #-------------------------------------------------------------
  # All six accessing instructions, on an address from each range. None of
  # these addresses is read-only by encoding (bits [11:10] are 00 or 10), so a
  # write is a legal WARL no-op rather than a fault -- including the forms that
  # write a non-zero value and the immediate forms, which is where a core that
  # implemented only the read side would trap.
  #-------------------------------------------------------------

  li    a5, 0xffffffff

  TEST_CASE( 12, a0, 0, csrrw a0, 0xB03, a5; )
  TEST_CASE( 13, a0, 0, csrrs a0, 0xB9F, a5; )
  TEST_CASE( 14, a0, 0, csrrc a0, 0x33F, a5; )
  TEST_CASE( 15, a0, 0, csrrwi a0, 0xB1F, 31; )
  TEST_CASE( 16, a0, 0, csrrsi a0, 0xB83, 31; )
  TEST_CASE( 17, a0, 0, csrrci a0, 0x323, 31; )

  # A write that lands nowhere still has to leave the read at zero.
  TEST_CASE( 18, a0, 0, csrr a0, 0xB03; )
  TEST_CASE( 19, a0, 0, csrr a0, 0xB9F; )
  TEST_CASE( 20, a0, 0, csrr a0, 0x33F; )

  TEST_CASE( 21, a4, 0, la a3, trap_count; lw a4, 0(a3); )

  #-------------------------------------------------------------
  # One past the top of the counter range, which is implemented by nothing and
  # must raise illegal instruction. Without this the file cannot fail.
  #-------------------------------------------------------------

  csrr  a0, 0xB20

  TEST_CASE( 22, a4, 1, la a3, trap_count; lw a4, 0(a3); )
  TEST_CASE( 23, a4, 2, la a3, trap_cause; lw a4, 0(a3); )

  .option pop

  TEST_PASSFAIL

  RVTEST_TRAP_HANDLER

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  RVTEST_TRAP_DATA

RVTEST_DATA_END

# See LICENSE for license details.

#*****************************************************************************
# amominmax.S
#-----------------------------------------------------------------------------
#
# The four min/max AMOs at the only operands where a wrong compare shows:
# 0x8000_0000 and -1, against each other and against small positives.
#
# WHY THESE OPERANDS. rtl/accessor.v serves the add and all four compares from
# ONE 33-bit adder/subtractor. The borrow out is unsigned less-than; signed
# less-than is the same fact except where the sign bits disagree, and then the
# negative operand is the smaller. Every case below where the two answers differ
# is a case where the sign bits disagree, so a comparator that dropped the
# correction, or read the shared `less` term the wrong way round, is only
# visible here. Ordinary operands like 5 against 3 agree under both readings and
# are in test/asm/amo.S for that reason.
#
# BOTH OPERAND ORDERS for every pair. The memory word and rs2 are not
# interchangeable in the hardware -- one arrives on the bus and the other was
# held from the request a cycle earlier -- so a keep-the-memory-word term
# inverted answers half of these correctly.
#
# Each case reads the memory back, for the reason the TEST_AMO comment in
# riscv_test.h gives: nothing else in this repository looks at what an AMO left
# in RAM.
#

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  #-------------------------------------------------------------
  # amomin.w -- signed. 0x8000_0000 is the smallest word there is.
  #-------------------------------------------------------------

  TEST_AMO(  2, amomin.w, 0x80000000, 0xffffffff, 0x80000000, 0x80000000 );
  TEST_AMO(  3, amomin.w, 0xffffffff, 0x80000000, 0xffffffff, 0x80000000 );
  TEST_AMO(  4, amomin.w, 0x80000000, 0x7fffffff, 0x80000000, 0x80000000 );
  TEST_AMO(  5, amomin.w, 0x7fffffff, 0x80000000, 0x7fffffff, 0x80000000 );
  TEST_AMO(  6, amomin.w, 0x80000000, 0x00000001, 0x80000000, 0x80000000 );
  TEST_AMO(  7, amomin.w, 0x00000001, 0x80000000, 0x00000001, 0x80000000 );
  TEST_AMO(  8, amomin.w, 0xffffffff, 0x00000001, 0xffffffff, 0xffffffff );
  TEST_AMO(  9, amomin.w, 0x00000001, 0xffffffff, 0x00000001, 0xffffffff );
  TEST_AMO( 10, amomin.w, 0xffffffff, 0x00000000, 0xffffffff, 0xffffffff );
  TEST_AMO( 11, amomin.w, 0x00000000, 0xffffffff, 0x00000000, 0xffffffff );

  # Equal operands: neither is less, so the answer must be that value however
  # the tie is broken.
  TEST_AMO( 12, amomin.w, 0x80000000, 0x80000000, 0x80000000, 0x80000000 );
  TEST_AMO( 13, amomin.w, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff );

  #-------------------------------------------------------------
  # amomax.w -- signed, the same pairs read the other way.
  #-------------------------------------------------------------

  TEST_AMO( 14, amomax.w, 0x80000000, 0xffffffff, 0x80000000, 0xffffffff );
  TEST_AMO( 15, amomax.w, 0xffffffff, 0x80000000, 0xffffffff, 0xffffffff );
  TEST_AMO( 16, amomax.w, 0x80000000, 0x7fffffff, 0x80000000, 0x7fffffff );
  TEST_AMO( 17, amomax.w, 0x7fffffff, 0x80000000, 0x7fffffff, 0x7fffffff );
  TEST_AMO( 18, amomax.w, 0x80000000, 0x00000001, 0x80000000, 0x00000001 );
  TEST_AMO( 19, amomax.w, 0x00000001, 0x80000000, 0x00000001, 0x00000001 );
  TEST_AMO( 20, amomax.w, 0xffffffff, 0x00000001, 0xffffffff, 0x00000001 );
  TEST_AMO( 21, amomax.w, 0x00000001, 0xffffffff, 0x00000001, 0x00000001 );
  TEST_AMO( 22, amomax.w, 0x80000000, 0x80000000, 0x80000000, 0x80000000 );

  #-------------------------------------------------------------
  # amominu.w -- unsigned. Every pair below has a sign disagreement in it, so
  # each answer is the OPPOSITE of the signed one above; that is what says the
  # correction is applied to the signed pair and only to them.
  #-------------------------------------------------------------

  TEST_AMO( 23, amominu.w, 0x80000000, 0xffffffff, 0x80000000, 0x80000000 );
  TEST_AMO( 24, amominu.w, 0xffffffff, 0x80000000, 0xffffffff, 0x80000000 );
  TEST_AMO( 25, amominu.w, 0x80000000, 0x7fffffff, 0x80000000, 0x7fffffff );
  TEST_AMO( 26, amominu.w, 0x7fffffff, 0x80000000, 0x7fffffff, 0x7fffffff );
  TEST_AMO( 27, amominu.w, 0x80000000, 0x00000001, 0x80000000, 0x00000001 );
  TEST_AMO( 28, amominu.w, 0x00000001, 0x80000000, 0x00000001, 0x00000001 );
  TEST_AMO( 29, amominu.w, 0xffffffff, 0x00000001, 0xffffffff, 0x00000001 );
  TEST_AMO( 30, amominu.w, 0x00000001, 0xffffffff, 0x00000001, 0x00000001 );
  TEST_AMO( 31, amominu.w, 0xffffffff, 0x00000000, 0xffffffff, 0x00000000 );
  TEST_AMO( 32, amominu.w, 0x00000000, 0xffffffff, 0x00000000, 0x00000000 );
  TEST_AMO( 33, amominu.w, 0x80000000, 0x80000000, 0x80000000, 0x80000000 );

  #-------------------------------------------------------------
  # amomaxu.w -- unsigned, the other way again.
  #-------------------------------------------------------------

  TEST_AMO( 34, amomaxu.w, 0x80000000, 0xffffffff, 0x80000000, 0xffffffff );
  TEST_AMO( 35, amomaxu.w, 0xffffffff, 0x80000000, 0xffffffff, 0xffffffff );
  TEST_AMO( 36, amomaxu.w, 0x80000000, 0x7fffffff, 0x80000000, 0x80000000 );
  TEST_AMO( 37, amomaxu.w, 0x7fffffff, 0x80000000, 0x7fffffff, 0x80000000 );
  TEST_AMO( 38, amomaxu.w, 0x80000000, 0x00000001, 0x80000000, 0x80000000 );
  TEST_AMO( 39, amomaxu.w, 0x00000001, 0x80000000, 0x00000001, 0x80000000 );
  TEST_AMO( 40, amomaxu.w, 0xffffffff, 0x00000001, 0xffffffff, 0xffffffff );
  TEST_AMO( 41, amomaxu.w, 0x00000001, 0xffffffff, 0x00000001, 0xffffffff );
  TEST_AMO( 42, amomaxu.w, 0x00000000, 0x00000000, 0x00000000, 0x00000000 );

  TEST_PASSFAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

  .align 2
amodat:
  .word 0x00000000

RVTEST_DATA_END

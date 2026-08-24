# See LICENSE for license details.

#*****************************************************************************
# sh2add.S
#-----------------------------------------------------------------------------
#
# Zba's sh2add: rd = rs2 + (rs1 << 2), the cheapest-shape probe for what one
# more opcode costs in the fetch loop -- a constant shift ahead of the adder
# rtl/executor.v's add arm already has, one mux, no new arithmetic.
#
# riscv-formal ships no spec model for it (the pin's isa is rv32imc, and Zba
# is not in it), so this self-checks in band the way test/asm/amo.S does:
# every case compares a register against an expected value and jumps to fail
# on a mismatch, with nothing routed through the per-retire RVFI monitor.
#
# `.option arch, +zba` is scoped to this file, not to the suite's -march: the
# assembler otherwise rejects the mnemonic, and widening the flag every other
# assembly program in this suite builds under is the ISA-string question this
# probe measures the cost of, not one it answers by itself.

#include "riscv_test.h"
#include "test_macros.h"

.option arch, +zba

RVTEST_RV64U
RVTEST_CODE_BEGIN

  #-------------------------------------------------------------
  # Arithmetic tests
  #-------------------------------------------------------------

  TEST_RR_OP( 2, sh2add, 0x00000000, 0x00000000, 0x00000000 );
  TEST_RR_OP( 3, sh2add, 0x00000004, 0x00000001, 0x00000000 );
  TEST_RR_OP( 4, sh2add, 0x00000013, 0x00000003, 0x00000007 );

  # The shift drops its top two bits rather than widening: 0x40000000 << 2 is
  # 2^32, which is 0 mod 2^32.
  TEST_RR_OP( 5, sh2add, 0x00000000, 0x40000000, 0x00000000 );

  # 0x7fffffff << 2, truncated to 32 bits, is 0xfffffffc.
  TEST_RR_OP( 6, sh2add, 0xfffffffd, 0x7fffffff, 0x00000001 );
  TEST_RR_OP( 7, sh2add, 0xfffffffc, 0xffffffff, 0x00000000 );

  TEST_RR_OP( 8, sh2add, 0xdeadbeef, 0x00000000, 0xdeadbeef );

  #-------------------------------------------------------------
  # Source/Destination tests
  #-------------------------------------------------------------

  TEST_RR_SRC1_EQ_DEST( 9,  sh2add, 0x0000003f, 13, 11 );
  TEST_RR_SRC2_EQ_DEST( 10, sh2add, 0x00000043, 14, 11 );
  TEST_RR_SRC12_EQ_DEST( 11, sh2add, 65, 13 );

  #-------------------------------------------------------------
  # Zero-operand tests
  #-------------------------------------------------------------

  TEST_RR_ZEROSRC1( 12, sh2add, 15, 15 );
  TEST_RR_ZEROSRC2( 13, sh2add, 60, 15 );
  TEST_RR_ZEROSRC12( 14, sh2add, 0 );
  TEST_RR_ZERODEST( 15, sh2add, 16, 30 );

  TEST_PASSFAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

RVTEST_DATA_END

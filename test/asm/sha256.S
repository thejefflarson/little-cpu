# See LICENSE for license details.

#*****************************************************************************
# sha256.S
#-----------------------------------------------------------------------------
#
# Zknh's SHA-256 half, a cost probe: sha256sig0, sha256sig1, sha256sum0 and
# sha256sum1, each rd = f(rs1) -- an XOR of three constant rotates, no adder
# and no shift amount. All four share one funct7 row of OP-IMM's funct3 001,
# told apart by the field a normal R-type reads as rs2.
#
# riscv-formal ships no spec model for them (the pin's isa is rv32imc, and
# Zknh is not in it), so this self-checks in band the way test/asm/sh2add.S
# does: every case compares a register against an expected value computed
# from the ratified spec's own definition (ror32 and xor), with nothing
# routed through the per-retire RVFI monitor.
#
# `.option arch, +zknh` is scoped to this file, not to the suite's -march: the
# assembler otherwise rejects the mnemonics, and widening the flag every other
# assembly program in this suite builds under is the ISA-string question this
# probe measures the cost of, not one it answers by itself.

#include "riscv_test.h"
#include "test_macros.h"

.option arch, +zknh

RVTEST_RV64U
RVTEST_CODE_BEGIN

  #-------------------------------------------------------------
  # sha256sig0: rd = ROR(rs1,7) ^ ROR(rs1,18) ^ (rs1 >> 3)
  #-------------------------------------------------------------

  TEST_R_OP( 2, sha256sig0, 0x00000000, 0x00000000 );
  TEST_R_OP( 3, sha256sig0, 0x02004000, 0x00000001 );
  TEST_R_OP( 4, sha256sig0, 0x1fffffff, 0xffffffff );
  TEST_R_OP( 5, sha256sig0, 0x11002000, 0x80000000 );
  TEST_R_OP( 6, sha256sig0, 0xe7fce6ee, 0x12345678 );
  TEST_R_OP( 7, sha256sig0, 0xabd31b0b, 0xdeadbeef );

  TEST_R_SRC1_EQ_DEST( 8, sha256sig0, 0xe7fce6ee, 0x12345678 );
  TEST_R_DEST_BYPASS( 9, 0, sha256sig0, 0xabd31b0b, 0xdeadbeef );
  TEST_R_DEST_BYPASS( 10, 1, sha256sig0, 0xabd31b0b, 0xdeadbeef );

  #-------------------------------------------------------------
  # sha256sig1: rd = ROR(rs1,17) ^ ROR(rs1,19) ^ (rs1 >> 10)
  #-------------------------------------------------------------

  TEST_R_OP( 11, sha256sig1, 0x00000000, 0x00000000 );
  TEST_R_OP( 12, sha256sig1, 0x0000a000, 0x00000001 );
  TEST_R_OP( 13, sha256sig1, 0x003fffff, 0xffffffff );
  TEST_R_OP( 14, sha256sig1, 0x00205000, 0x80000000 );
  TEST_R_OP( 15, sha256sig1, 0xa1f78649, 0x12345678 );
  TEST_R_OP( 16, sha256sig1, 0x689dbfec, 0xdeadbeef );

  TEST_R_SRC1_EQ_DEST( 17, sha256sig1, 0xa1f78649, 0x12345678 );
  TEST_R_DEST_BYPASS( 18, 0, sha256sig1, 0x689dbfec, 0xdeadbeef );
  TEST_R_DEST_BYPASS( 19, 1, sha256sig1, 0x689dbfec, 0xdeadbeef );

  #-------------------------------------------------------------
  # sha256sum0: rd = ROR(rs1,2) ^ ROR(rs1,13) ^ ROR(rs1,22)
  #-------------------------------------------------------------

  TEST_R_OP( 20, sha256sum0, 0x00000000, 0x00000000 );
  TEST_R_OP( 21, sha256sum0, 0x40080400, 0x00000001 );
  TEST_R_OP( 22, sha256sum0, 0xffffffff, 0xffffffff );
  TEST_R_OP( 23, sha256sum0, 0x20040200, 0x80000000 );
  TEST_R_OP( 24, sha256sum0, 0x66146474, 0x12345678 );
  TEST_R_OP( 25, sha256sum0, 0xb62e25ac, 0xdeadbeef );

  TEST_R_SRC1_EQ_DEST( 26, sha256sum0, 0x66146474, 0x12345678 );
  TEST_R_DEST_BYPASS( 27, 0, sha256sum0, 0xb62e25ac, 0xdeadbeef );
  TEST_R_DEST_BYPASS( 28, 1, sha256sum0, 0xb62e25ac, 0xdeadbeef );

  #-------------------------------------------------------------
  # sha256sum1: rd = ROR(rs1,6) ^ ROR(rs1,11) ^ ROR(rs1,25)
  #-------------------------------------------------------------

  TEST_R_OP( 29, sha256sum1, 0x00000000, 0x00000000 );
  TEST_R_OP( 30, sha256sum1, 0x04200080, 0x00000001 );
  TEST_R_OP( 31, sha256sum1, 0xffffffff, 0xffffffff );
  TEST_R_OP( 32, sha256sum1, 0x02100040, 0x80000000 );
  TEST_R_OP( 33, sha256sum1, 0x3561abda, 0x12345678 );
  TEST_R_OP( 34, sha256sum1, 0x345e14a3, 0xdeadbeef );

  TEST_R_SRC1_EQ_DEST( 35, sha256sum1, 0x3561abda, 0x12345678 );
  TEST_R_DEST_BYPASS( 36, 0, sha256sum1, 0x345e14a3, 0xdeadbeef );
  TEST_R_DEST_BYPASS( 37, 1, sha256sum1, 0x345e14a3, 0xdeadbeef );

  #-------------------------------------------------------------
  # Zero-destination tests: rd = x0 stays 0 for all four
  #-------------------------------------------------------------

  TEST_CASE( 38, x0, 0, li x1, 0x12345678; sha256sig0 x0, x1; );
  TEST_CASE( 39, x0, 0, li x1, 0x12345678; sha256sig1 x0, x1; );
  TEST_CASE( 40, x0, 0, li x1, 0x12345678; sha256sum0 x0, x1; );
  TEST_CASE( 41, x0, 0, li x1, 0x12345678; sha256sum1 x0, x1; );

  TEST_PASSFAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

RVTEST_DATA_END

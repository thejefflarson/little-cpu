# See LICENSE for license details.

#*****************************************************************************
# amo.S
#-----------------------------------------------------------------------------
#
# The nine AMO functions, each against the word it must leave behind as well as
# the word it must return.
#
# EVERY CASE READS THE MEMORY BACK INTO A REGISTER, and that is not decoration.
# test/cosim.cc compares the core's architectural registers against the Sail
# model and never compares memory, so an AMO whose only effect stayed in RAM
# would be checked by nothing anywhere. The per-retire RVFI monitor is no help
# either: riscv-formal ships no spec model for any A encoding at the pinned SHA,
# so `spec_valid` is 0 for all eleven and test/monitor.sim.v's whole value block
# is skipped for them. This file self-checks in band for the same reason
# test/asm/trap.S does.
#
# The four min/max functions get one straightforward case each here. Their
# signed boundaries are test/asm/amominmax.S, which is where a comparator read
# the wrong way round actually shows.
#

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  #-------------------------------------------------------------
  # The nine functions.
  #-------------------------------------------------------------

  TEST_AMO(  2, amoswap.w, 0x12345678, 0xdeadbeef, 0x12345678, 0xdeadbeef );
  TEST_AMO(  3, amoadd.w,  0x00000010, 0x00000022, 0x00000010, 0x00000032 );
  TEST_AMO(  4, amoxor.w,  0x0ff00ff0, 0x00ff00ff, 0x0ff00ff0, 0x0f0f0f0f );
  TEST_AMO(  5, amoand.w,  0x0ff00ff0, 0x00ff00ff, 0x0ff00ff0, 0x00f000f0 );
  TEST_AMO(  6, amoor.w,   0x0ff00ff0, 0x00ff00ff, 0x0ff00ff0, 0x0fff0fff );
  TEST_AMO(  7, amomin.w,  0x00000005, 0x00000003, 0x00000005, 0x00000003 );
  TEST_AMO(  8, amomax.w,  0x00000005, 0x00000003, 0x00000005, 0x00000005 );
  TEST_AMO(  9, amominu.w, 0x00000005, 0x00000003, 0x00000005, 0x00000003 );
  TEST_AMO( 10, amomaxu.w, 0x00000005, 0x00000003, 0x00000005, 0x00000005 );

  # The add really is 32-bit modular arithmetic: the carry out of bit 31 is
  # dropped rather than landing in the 33rd bit the accessor's shared
  # adder/subtractor carries for the compares.
  TEST_AMO( 11, amoadd.w,  0xffffffff, 0x00000001, 0xffffffff, 0x00000000 );
  TEST_AMO( 12, amoadd.w,  0x7fffffff, 0x00000001, 0x7fffffff, 0x80000000 );

  #-------------------------------------------------------------
  # The ordering suffixes are decoded and ignored: one hart, in order, one
  # outstanding access. All four spellings of amoadd.w must therefore do the
  # same arithmetic rather than decode as an illegal instruction.
  #-------------------------------------------------------------

  TEST_AMO( 13, amoadd.w.aq,   0x00000010, 0x00000022, 0x00000010, 0x00000032 );
  TEST_AMO( 14, amoadd.w.rl,   0x00000010, 0x00000022, 0x00000010, 0x00000032 );
  TEST_AMO( 15, amoadd.w.aqrl, 0x00000010, 0x00000022, 0x00000010, 0x00000032 );

  #-------------------------------------------------------------
  # Register aliasing. rd is written from the bus a cycle after rs1 and rs2 were
  # spent on the request, so an AMO naming the same register twice is the case
  # where a held operand and a writeback could collide.
  #-------------------------------------------------------------

  # rd == rs2: the returned word overwrites the operand that was added to it.
  TEST_CASE( 16, x4, 0x00000010, \
    la  x1, amodat; \
    li  x2, 0x00000010; \
    sw  x2, 0(x1); \
    li  x4, 0x00000022; \
    amoadd.w x4, x4, (x1); \
  )
  TEST_CASE( 17, x6, 0x00000032, la x1, amodat; lw x6, 0(x1); )

  # rd == rs1: the returned word overwrites the address it came from.
  TEST_CASE( 18, x1, 0x00000010, \
    la  x1, amodat; \
    li  x2, 0x00000010; \
    sw  x2, 0(x1); \
    li  x4, 0x00000022; \
    amoadd.w x1, x4, (x1); \
  )
  TEST_CASE( 19, x6, 0x00000032, la x1, amodat; lw x6, 0(x1); )

  # rd == x0: the write to memory still happens, and nothing lands in x0.
  TEST_CASE( 20, x0, 0, \
    la  x1, amodat; \
    li  x2, 0x00000010; \
    sw  x2, 0(x1); \
    li  x4, 0x00000022; \
    amoadd.w x0, x4, (x1); \
  )
  TEST_CASE( 21, x6, 0x00000032, la x1, amodat; lw x6, 0(x1); )

  #-------------------------------------------------------------
  # An AMO spends the cycle after its read keeping everything else off the bus.
  # These interlock a dependent instruction and a second memory instruction
  # immediately behind one, which is where a wait that ended a cycle early would
  # show as a wrong answer rather than as a stall.
  #-------------------------------------------------------------

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

  # Two AMOs back to back on the same word: the second must see the first's
  # write, which is only true if the read-modify-write really landed before the
  # next read went out.
  TEST_CASE( 24, x8, 0x00000021, \
    la  x1, amodat; \
    li  x2, 0x00000010; \
    sw  x2, 0(x1); \
    li  x4, 0x00000011; \
    amoadd.w x5, x4, (x1); \
    amoadd.w x8, x4, (x1); \
  )
  TEST_CASE( 25, x6, 0x00000032, la x1, amodat; lw x6, 0(x1); )

  # A divide in flight holds the AMO in decode before the executor takes it, so
  # no read has gone out and no write cycle is next. The arithmetic on both
  # sides has to survive that.
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

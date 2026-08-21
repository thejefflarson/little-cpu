# See LICENSE for license details.

#*****************************************************************************
# amoregion.S
#-----------------------------------------------------------------------------
#
# An atomic outside the data RAM, which is the one region this platform answers
# one at.
#
# WHAT THE HARDWARE DOES AND WHY. rtl/memory.v answers the range test it already
# computes about a second address -- the one an atomic's rs1 names -- and
# rtl/decoder.v raises a fault from it before the instruction issues: cause 5,
# load access fault, for lr.w, and cause 7, store/AMO access fault, for the nine
# AMOs and sc.w. That is what the privileged specification names as the
# preferred way to report an access that fails its PMA check, and it is
# affordable here for one reason the same question about a load or a store does
# not have: an atomic's effective address is rs1 verbatim, so the region test
# reads a register output rather than the top of a 32-bit sum.
#
# THE REGION IS THE DATA RAM AND NOT "ANYTHING MAPPED". Text is readable and
# writable through the data bus and the machine timer answers four words, and
# neither supports atomics -- so an atomic at either faults, which is what their
# PMAs should say. The cases below reach all three: unmapped, the timer, and
# text.
#
# WHAT IS STILL SILENT, ASSERTED HERE AS WHAT IT IS. A plain load or store to an
# address no memory answers still reads zero and is written nowhere, with no
# fault -- the decode-time region test for those two was measured at four logic
# levels in the fetch loop and declined. The lw cases below assert that
# deviation rather than a specification requirement, and they are here so a
# change to it is a change to a graded program rather than a discovery.
#
# THE RESERVATION IS STILL REFUSED OUTSIDE THE RAM, and no program can see it
# any more: rtl/accessor.v sets no reservation where rtl/memory.v says none may
# be held, but an lr.w out there now faults before the accessor ever sees it. It
# stays as the second half of the same statement -- a platform that tied the
# fault bit high would still not let an sc.w report success for a write that
# went nowhere -- and test/accessor_tb.v and components_accessor are what check
# it now.
#
# Every trapping instruction is wrapped in `.option norvc` per riscv_test.h's
# first constraint. No atomic has a compressed form, so the directive changes no
# bytes here; the handler's unconditional mepc+4 is what it is there for.
#

#include "riscv_test.h"
#include "test_macros.h"

# Above the machine timer's four words and below the 1 MB the linker script may
# grow into. Nothing on this bus answers it.
#define UNMAPPED 0x00040000

# One refused atomic with an rs2: the cause, then that rd was not written.
# `x10` is the address, `x11` is rd, `x12` is rs2.
#define TEST_REFUSED_AMO( testnum, inst, addr, cause ) \
test_ ## testnum: \
    li  TESTNUM, testnum; \
    la  x1, trap_cause; \
    sw  x0, 0(x1); \
    li  x10, addr; \
    li  x11, 0x5a5; \
    li  x12, 0x3c3; \
    .option push; \
    .option norvc; \
    inst x11, x12, (x10); \
    .option pop; \
    la  x1, trap_cause; \
    lw  x13, 0(x1); \
    li  x29, cause; \
    bne x13, x29, fail; \
    li  x29, 0x5a5; \
    bne x11, x29, fail;

# The same for lr.w, which takes no rs2.
#define TEST_REFUSED_LR( testnum, addr ) \
test_ ## testnum: \
    li  TESTNUM, testnum; \
    la  x1, trap_cause; \
    sw  x0, 0(x1); \
    li  x10, addr; \
    li  x11, 0x5a5; \
    .option push; \
    .option norvc; \
    lr.w x11, (x10); \
    .option pop; \
    la  x1, trap_cause; \
    lw  x13, 0(x1); \
    li  x29, 5; \
    bne x13, x29, fail; \
    li  x29, 0x5a5; \
    bne x11, x29, fail;

RVTEST_RV64U
RVTEST_CODE_BEGIN

  # mtvec resets to 0, which is _start -- install before faulting.
  RVTEST_INSTALL_TRAP_HANDLER

  #-------------------------------------------------------------
  # The control. The same sequence inside the data RAM succeeds, so a fault
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
  # An address no memory answers. All eleven encodings, because the flags are
  # decoded one per function rather than from funct5, and a region test wired to
  # ten of them would pass every other check in the tree.
  #-------------------------------------------------------------

  TEST_REFUSED_LR(   4, UNMAPPED );
  TEST_REFUSED_AMO(  5, sc.w,      UNMAPPED, 7 );
  TEST_REFUSED_AMO(  6, amoswap.w, UNMAPPED, 7 );
  TEST_REFUSED_AMO(  7, amoadd.w,  UNMAPPED, 7 );
  TEST_REFUSED_AMO(  8, amoxor.w,  UNMAPPED, 7 );
  TEST_REFUSED_AMO(  9, amoand.w,  UNMAPPED, 7 );
  TEST_REFUSED_AMO( 10, amoor.w,   UNMAPPED, 7 );
  TEST_REFUSED_AMO( 11, amomin.w,  UNMAPPED, 7 );
  TEST_REFUSED_AMO( 12, amomax.w,  UNMAPPED, 7 );
  TEST_REFUSED_AMO( 13, amominu.w, UNMAPPED, 7 );
  TEST_REFUSED_AMO( 14, amomaxu.w, UNMAPPED, 7 );

  # Twelve so far, one per case above.
  TEST_CASE( 15, a4, 11, la a0, trap_count; lw a4, 0(a0); )

  # mepc is the faulting atomic's own address, so resuming at mepc+4 landed on
  # the instruction after it -- which is where this is running.
test_16:
  li    TESTNUM, 16
  li    x10, UNMAPPED
  li    x12, 0x3c3
  .option push
  .option norvc
refused_amo:
  amoadd.w x11, x12, (x10)
  .option pop
  la    x1, trap_epc
  lw    x13, 0(x1)
  la    x29, refused_amo
  bne   x13, x29, fail

  #-------------------------------------------------------------
  # A plain load there faults now too, so the deviation this file used to record
  # is closed and test/asm/loadfault.S and test/asm/storefault.S are where it is
  # covered. These two cases keep the trap count pinned where the deviation's
  # cases sat, so the arithmetic below them is unchanged.
  #-------------------------------------------------------------

  TEST_CASE( 17, a4, 12, la a0, trap_count; lw a4, 0(a0); )
  TEST_CASE( 18, a4, 12, la a0, trap_count; lw a4, 0(a0); )

  #-------------------------------------------------------------
  # The machine timer's four words. They are a device rather than main memory,
  # so an atomic there faults instead of executing a read-modify-write in two
  # transactions -- which is what the timer's PMA should say and what an AMO
  # there used to do.
  #-------------------------------------------------------------

  RVTEST_DISARM_TIMER

  TEST_REFUSED_LR(  19, MTIMER_BASE + MTIMECMP_OFFSET );
  TEST_REFUSED_AMO( 20, sc.w,     MTIMER_BASE + MTIMECMP_OFFSET, 7 );
  TEST_REFUSED_AMO( 21, amoadd.w, MTIMER_BASE + MTIMECMP_OFFSET, 7 );

  # ...and mtimecmp is exactly what the disarm left it, so none of the three
  # reached the device.
  TEST_CASE( 22, a4, -1, \
    li  a0, MTIMER_BASE + MTIMECMP_OFFSET; \
    lw  a4, 0(a0); \
  )

  #-------------------------------------------------------------
  # Text, which the data bus reaches for loads and stores and which is not the
  # data RAM either. This is the case that says the attribute is "is this the
  # RAM" and not "is this mapped".
  #-------------------------------------------------------------

  TEST_REFUSED_LR(  23, 0x00000000 );
  TEST_REFUSED_AMO( 24, amoadd.w, 0x00000000, 7 );

  # ...while a plain load from text is answered, which is what test/asm's
  # textload.S asserts in full. Seventeen refused atomics so far and this load
  # added none, so the fault is the atomic's and not the address's.
  TEST_CASE( 25, a4, 17, \
    li  a0, 0x00000000; \
    lw  a1, 0(a0); \
    la  a0, trap_count; \
    lw  a4, 0(a0); \
  )

  #-------------------------------------------------------------
  # ...and the data RAM is still reservable afterwards, so nothing above left
  # the reservation state stuck.
  #-------------------------------------------------------------

  TEST_CASE( 26, a2, 0, \
    la  a0, rdat; \
    lr.w a1, (a0); \
    li  a5, 0x0000c0de; \
    sc.w a2, a5, (a0); \
  )
  TEST_CASE( 27, a4, 0x0000c0de, la a0, rdat; lw a4, 0(a0); )

  # Seventeen refused atomics in all, and nothing else trapped.
  TEST_CASE( 28, a4, 17, la a0, trap_count; lw a4, 0(a0); )

  TEST_PASSFAIL

  RVTEST_TRAP_HANDLER

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  RVTEST_TRAP_DATA

  .align 2
rdat:
  .word 0x0ff00ff0

RVTEST_DATA_END

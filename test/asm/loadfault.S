# See LICENSE for license details.

#*****************************************************************************
# loadfault.S
#-----------------------------------------------------------------------------
#
# A plain load at an address no memory on this platform answers: cause 5, load
# access fault, committed in decode with every other cause.
#
# WHAT THE HARDWARE DOES AND WHY. The effective address is `immediate +
# reg_rs1`, and the top of that sum arrives far too late to choose the next pc,
# so it is never read. Every immediate that can reach this test is sign-extended
# above bit 11, so the sum's top twenty bits are `reg_rs1[31:12]` plus an
# adjustment of -1, 0 or +1. Each window is therefore tested three ways on the
# raw register bits, and the one late bit -- the carry into bit 12, read back
# out of the adder the address already pays for -- picks between two precomputed
# answers. The map reaches decode at elaboration, from the same four parameters
# rtl/littlecpu.v hands its load/store locality counters.
#
# WHAT THIS PROGRAM CAN AND CANNOT SEE. The per-retire monitor is blind on the
# faulting retires, deliberately: riscv-formal ships no memory map, so its spec
# model answers a load at any address at all and would call every case below a
# "mismatch in trap". test/testbench.v drops exactly those retires and no
# others -- see `ls_fault_retire` there. So every claim here is an in-band
# assertion, and the oracles behind them are formal/traps.sv, which requires
# both the trap and its cause against this same map under k-induction, and
# co-simulation, whose model carries this platform's regions in
# test/sail/rv32imac_zicsr.json.
#
# NOTHING BELOW PUTS A LOADED VALUE IN A REGISTER unless the value is one this
# program wrote. The answered cases load into x0, which makes the access happen
# and writes nothing: the map, not the data, is what they are about, and a word
# of ROM or of the machine timer is not a value two machines have to agree on.
#
# THE CARRY IS WHAT THIS FILE IS FOR. Cases 12 to 15 are the ones a region test
# on `reg_rs1` alone gets wrong, in both directions: an address that leaves its
# window through a positive offset must fault, and one that comes back into a
# window through a negative offset must not.
#
# Every faulting instruction is wrapped in `.option norvc` per riscv_test.h's
# first constraint, except case 16, which is a compressed load on purpose and
# carries the two bytes that constraint really asks for.
#

#include "riscv_test.h"
#include "test_macros.h"

# Above the machine timer's four words and below the 1 MB the linker script may
# grow into. Nothing on this bus answers it.
#define UNMAPPED 0x00040000

# The last word of the 16 KB text window, and the first word past it.
#define TEXT_TOP  0x00003FFC
#define TEXT_PAST 0x00004000

# The base of the 64 KB data RAM, and the last word below it.
#define RAM_BASE  0x00010000
#define RAM_BELOW 0x0000FFFC

# One refused load: the cause, then that rd was not written.
#define TEST_REFUSED_LOAD( testnum, inst, base, off ) \
test_ ## testnum: \
    li  TESTNUM, testnum; \
    la  x1, trap_cause; \
    sw  x0, 0(x1); \
    li  x10, base; \
    li  x11, 0x5a5; \
    .option push; \
    .option norvc; \
    inst x11, off(x10); \
    .option pop; \
    la  x1, trap_cause; \
    lw  x13, 0(x1); \
    li  x29, 5; \
    bne x13, x29, fail; \
    li  x29, 0x5a5; \
    bne x11, x29, fail;

# The other direction, at an address the map DOES answer: the trap count must
# not move. Without these the file would pass on a core that faulted every load.
#define TEST_ANSWERED_LOAD( testnum, base, off ) \
test_ ## testnum: \
    li  TESTNUM, testnum; \
    la  x1, trap_count; \
    lw  x14, 0(x1); \
    li  x10, base; \
    .option push; \
    .option norvc; \
    lw  x0, off(x10); \
    .option pop; \
    la  x1, trap_count; \
    lw  x13, 0(x1); \
    bne x13, x14, fail;

RVTEST_RV64U
RVTEST_CODE_BEGIN

  # mtvec resets to 0, which is _start -- install before faulting.
  RVTEST_INSTALL_TRAP_HANDLER

  #-------------------------------------------------------------
  # The control. The same load inside the data RAM succeeds, so a fault below
  # is the address and not the mechanism.
  #-------------------------------------------------------------

  TEST_CASE( 2, a4, 0x0ff00ff0, la a0, rdat; lw a4, 0(a0); )

  #-------------------------------------------------------------
  # All five load encodings at an address no memory answers. The flags are
  # decoded one per width, so a region test wired to four of them would pass
  # every other check in the tree.
  #-------------------------------------------------------------

  TEST_REFUSED_LOAD(  3, lw,  UNMAPPED, 0 );
  TEST_REFUSED_LOAD(  4, lh,  UNMAPPED, 0 );
  TEST_REFUSED_LOAD(  5, lhu, UNMAPPED, 0 );
  TEST_REFUSED_LOAD(  6, lb,  UNMAPPED, 0 );
  TEST_REFUSED_LOAD(  7, lbu, UNMAPPED, 1 );

  # Five so far, one per case above.
  TEST_CASE( 8, a4, 5, la a0, trap_count; lw a4, 0(a0); )

  # mepc is the faulting load's own address, so resuming at mepc+4 landed on the
  # instruction after it -- which is where this is running.
test_9:
  li    TESTNUM, 9
  li    x10, UNMAPPED
  .option push
  .option norvc
refused_load:
  lw    x11, 0(x10)
  .option pop
  la    x1, trap_epc
  lw    x13, 0(x1)
  la    x29, refused_load
  bne   x13, x29, fail

  #-------------------------------------------------------------
  # Misalignment outranks the region, which keeps the four data causes disjoint
  # and matches what the reference model reports. Drop the alignment term and
  # two arms of the decoder's cause chain match at once.
  #-------------------------------------------------------------

test_10:
  li    TESTNUM, 10
  la    x1, trap_cause
  sw    x0, 0(x1)
  li    x10, UNMAPPED + 2
  .option push
  .option norvc
  lw    x11, 0(x10)
  .option pop
  la    x1, trap_cause
  lw    x13, 0(x1)
  li    x29, 4
  bne   x13, x29, fail

test_11:
  li    TESTNUM, 11
  la    x1, trap_cause
  sw    x0, 0(x1)
  li    x10, UNMAPPED + 1
  .option push
  .option norvc
  lh    x11, 0(x10)
  .option pop
  la    x1, trap_cause
  lw    x13, 0(x1)
  li    x29, 4
  bne   x13, x29, fail

  #-------------------------------------------------------------
  # THE CARRY. Each pair straddles one window's edge in both directions, so the
  # base register names a page on the wrong side of it every time.
  #-------------------------------------------------------------

  TEST_REFUSED_LOAD(  12, lw, TEXT_TOP, 4 );
  TEST_ANSWERED_LOAD( 13, TEXT_PAST, -4 );
  TEST_REFUSED_LOAD(  14, lw, RAM_BASE + 4, -8 );
  TEST_ANSWERED_LOAD( 15, RAM_BELOW, 8 );

  #-------------------------------------------------------------
  # A compressed load faults the same way. `c.lw` sets the same decoder flag and
  # its offset is zero-extended and five bits wide, which is inside the same
  # sign extension the split stands on.
  #
  # The `c.nop` behind it is what makes riscv_test.h's handler usable here: that
  # handler resumes at mepc+4 whatever the instruction was, so a two-byte
  # faulting instruction needs two bytes of its own behind it or control comes
  # back inside the next word.
  #-------------------------------------------------------------

test_16:
  li    TESTNUM, 16
  la    x1, trap_cause
  sw    x0, 0(x1)
  li    a0, UNMAPPED
  li    a1, 0x5a5
  .option push
  .option rvc
  c.lw  a1, 0(a0)
  c.nop
  .option pop
  la    x1, trap_cause
  lw    x13, 0(x1)
  li    x29, 5
  bne   x13, x29, fail
  li    x29, 0x5a5
  bne   a1, x29, fail

  #-------------------------------------------------------------
  # The map is four windows and not one. Text is readable through the data bus,
  # the timer's window is the eight words the map reserves for one `mtimecmp`
  # per hart, and the UART's two words sit above the whole of that; the word
  # above THOSE is nobody's. Every boundary here is below bit 12 -- the only
  # part of this test that reads the sum rather than the register.
  #
  # Case 19 reads the UART's status register, which is the one window a store
  # cannot be used to probe: writing it is how a byte is sent.
  #
  # THE REFUSAL ABOVE THE UART IS NOT HERE, AND CANNOT BE. The word at
  # UART_BASE+8 is one no device answers, but the reference model's regions are
  # 4K-aligned -- it refuses a configuration that ends anywhere else -- so on
  # Sail that whole page is ordinary memory and a program refusing an address
  # inside it would take a different path there. test/decoder_tb.v vectors it
  # instead, and the generated riscv-formal checks reach it with a free base
  # register, which is more than a program can do.
  #-------------------------------------------------------------

  TEST_ANSWERED_LOAD( 17, 0x00000000, 0 );
  TEST_ANSWERED_LOAD( 18, MTIMER_BASE, 0 );
  TEST_ANSWERED_LOAD( 19, UART_BASE, UART_STATUS_OFFSET );

  # Eleven faulting loads in all -- five widths, one for mepc, two misaligned,
  # two off a window edge and one compressed. Nothing besides those trapped.
  TEST_CASE( 21, a4, 11, la a0, trap_count; lw a4, 0(a0); )

  # ...and the data RAM still reads what it held, so nothing above disturbed it.
  TEST_CASE( 22, a4, 0x0ff00ff0, la a0, rdat; lw a4, 0(a0); )

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

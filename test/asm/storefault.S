# See LICENSE for license details.

#*****************************************************************************
# storefault.S
#-----------------------------------------------------------------------------
#
# A plain store at an address no memory on this platform answers: cause 7,
# store/AMO access fault, committed in decode with every other cause. The load
# half of the same mechanism is test/asm/loadfault.S, and the header there
# describes the carry split both files exercise.
#
# WHY THE STORE HALF IS A FILE OF ITS OWN. The two causes come out of one term
# and split on the encoding, so a decoder that wired the region test to the load
# opcode alone would pass loadfault.S entirely. The split is also what keeps
# `mcause` conformant -- the privileged spec puts a store's access fault at 7
# and a load's at 5 -- and it is checked here against the trap handler's own
# reading of mcause rather than against anything the core reports about itself.
#
# WHAT A DROPPED STORE USED TO LOOK LIKE, and why this file is the one that
# closes it: an out-of-region store went nowhere with no fault at all, which is
# a deviation no program could assert without also asserting where the word
# went. Now it traps, so the assertion is the cause and the count, and the word
# is beside the point.
#
# THE SCRATCH WORD AT THE TOP OF RAM. Case 11 stores across the RAM's upper
# boundary and reads the word back, so it needs a location the linker has not
# given to anything: 0x0001FFFC is the last word of the 64 KB data RAM, and
# test/asm/sections.lds puts .data and .bss at the bottom of that region with no
# stack anywhere -- these programs never set one.
#
# Every faulting instruction is wrapped in `.option norvc` per riscv_test.h's
# first constraint, except case 12, which is a compressed store on purpose and
# carries the two bytes that constraint really asks for.
#

#include "riscv_test.h"
#include "test_macros.h"

#define UNMAPPED 0x00040000

# The last word of the 16 KB text window, and the first word past it.
#define TEXT_TOP  0x00003FFC
// One past the top of the text window -- and it is NOT the same on every
// machine this runs on. test/testbench.v gives a simulator 4096 words, the part
// has 2048, and test/memmap_test.sh prints both: "rom 2048 words on the part /
// 4096 simulated". Hardcoding the simulator's number made this program test an
// address the part does not have: the load below was answered under simulation
// and correctly REFUSED on hardware, so the check failed on a core that was
// right. Overridable, and the board build passes the part's value.
#ifndef TEXT_PAST
#define TEXT_PAST 0x00004000
#endif

# The base of the 64 KB data RAM, its last word, and the first word past it.
#define RAM_BASE  0x00010000
#define RAM_TOP   0x0001FFFC
#define RAM_PAST  0x00020000

# One refused store: the cause, and that the trap count moved by exactly one.
#define TEST_REFUSED_STORE( testnum, inst, base, off ) \
test_ ## testnum: \
    li  TESTNUM, testnum; \
    la  x1, trap_cause; \
    sw  x0, 0(x1); \
    la  x1, trap_count; \
    lw  x14, 0(x1); \
    li  x10, base; \
    li  x11, 0x5a5; \
    .option push; \
    .option norvc; \
    inst x11, off(x10); \
    .option pop; \
    la  x1, trap_cause; \
    lw  x13, 0(x1); \
    li  x29, 7; \
    bne x13, x29, fail; \
    la  x1, trap_count; \
    lw  x13, 0(x1); \
    addi x29, x14, 1; \
    bne x13, x29, fail;

RVTEST_RV64U
RVTEST_CODE_BEGIN

  # mtvec resets to 0, which is _start -- install before faulting.
  RVTEST_INSTALL_TRAP_HANDLER

  #-------------------------------------------------------------
  # The control. The same store inside the data RAM lands, so a fault below is
  # the address and not the mechanism.
  #-------------------------------------------------------------

  TEST_CASE( 2, a4, 0x12345678, \
    la  a0, sdat; \
    li  a1, 0x12345678; \
    sw  a1, 0(a0); \
    lw  a4, 0(a0); \
  )

  #-------------------------------------------------------------
  # All three store encodings at an address no memory answers.
  #-------------------------------------------------------------

  TEST_REFUSED_STORE( 3, sw, UNMAPPED, 0 );
  TEST_REFUSED_STORE( 4, sh, UNMAPPED, 0 );
  TEST_REFUSED_STORE( 5, sb, UNMAPPED, 1 );

  # mepc is the faulting store's own address, so resuming at mepc+4 landed on
  # the instruction after it -- which is where this is running.
test_6:
  li    TESTNUM, 6
  li    x10, UNMAPPED
  li    x11, 0x5a5
  .option push
  .option norvc
refused_store:
  sw    x11, 0(x10)
  .option pop
  la    x1, trap_epc
  lw    x13, 0(x1)
  la    x29, refused_store
  bne   x13, x29, fail

  #-------------------------------------------------------------
  # Misalignment outranks the region here too, so the four data causes stay
  # disjoint whichever access raised them.
  #-------------------------------------------------------------

test_7:
  li    TESTNUM, 7
  la    x1, trap_cause
  sw    x0, 0(x1)
  li    x10, UNMAPPED + 2
  li    x11, 0x5a5
  .option push
  .option norvc
  sw    x11, 0(x10)
  .option pop
  la    x1, trap_cause
  lw    x13, 0(x1)
  li    x29, 6
  bne   x13, x29, fail

test_8:
  li    TESTNUM, 8
  la    x1, trap_cause
  sw    x0, 0(x1)
  li    x10, UNMAPPED + 1
  li    x11, 0x5a5
  .option push
  .option norvc
  sh    x11, 0(x10)
  .option pop
  la    x1, trap_cause
  lw    x13, 0(x1)
  li    x29, 6
  bne   x13, x29, fail

  #-------------------------------------------------------------
  # THE CARRY, in both directions across both edges a store can reach. Case 11
  # writes the top word of the data RAM from a base register naming the page
  # ABOVE it, and reads it back through a base naming the page below -- so the
  # store and the load disagree about the page and agree about the address.
  #-------------------------------------------------------------

  TEST_REFUSED_STORE( 9, sw, TEXT_TOP, 4 );
  TEST_REFUSED_STORE( 10, sw, RAM_BASE + 4, -8 );

test_11:
  li    TESTNUM, 11
  la    x1, trap_count
  lw    x14, 0(x1)
  li    x10, RAM_PAST
  li    x11, 0x0f0fa5a5
  .option push
  .option norvc
  sw    x11, -4(x10)
  .option pop
  li    x10, RAM_TOP - 4
  lw    x13, 4(x10)
  bne   x13, x11, fail
  la    x1, trap_count
  lw    x13, 0(x1)
  bne   x13, x14, fail

  #-------------------------------------------------------------
  # A compressed store faults the same way. `c.swsp` sets the same decoder flag
  # and its offset is zero-extended.
  #
  # The `c.nop` behind it is what makes riscv_test.h's handler usable here: that
  # handler resumes at mepc+4 whatever the instruction was, so a two-byte
  # faulting instruction needs two bytes of its own behind it or control comes
  # back inside the next word.
  #-------------------------------------------------------------

test_12:
  li    TESTNUM, 12
  la    x1, trap_cause
  sw    x0, 0(x1)
  li    sp, UNMAPPED
  li    a1, 0x5a5
  .option push
  .option rvc
  c.swsp a1, 0(sp)
  c.nop
  .option pop
  la    x1, trap_cause
  lw    x13, 0(x1)
  li    x29, 7
  bne   x13, x29, fail

  # THE REFUSAL ABOVE THE UART IS NOT HERE, for the reason the header of
  # test/asm/loadfault.S gives: the reference model's regions are 4K-aligned, so
  # a sub-page refusal is one no configuration of it can express.
  # test/decoder_tb.v vectors that boundary and the generated riscv-formal
  # checks reach it with a free base register.

  # Nine faulting stores in all: three widths, one for mepc, two misaligned, two
  # off a window edge and one compressed.
  TEST_CASE( 14, a4, 9, la a0, trap_count; lw a4, 0(a0); )

  # ...and the word case 2 wrote is still there, so nothing above reached the
  # data RAM by accident.
  TEST_CASE( 15, a4, 0x12345678, la a0, sdat; lw a4, 0(a0); )

  TEST_PASSFAIL

  RVTEST_TRAP_HANDLER

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  RVTEST_TRAP_DATA

  .align 2
sdat:
  .word 0

RVTEST_DATA_END

# See LICENSE for license details.

# A plain store at an address no memory on this platform answers: cause 7, store/AMO
# access fault, committed in decode with every other cause.

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

  TEST_CASE( 2, a4, 0x12345678, \
    la  a0, sdat; \
    li  a1, 0x12345678; \
    sw  a1, 0(a0); \
    lw  a4, 0(a0); \
  )

  # All three store encodings at an address no memory answers.

  TEST_REFUSED_STORE( 3, sw, UNMAPPED, 0 );
  TEST_REFUSED_STORE( 4, sh, UNMAPPED, 0 );
  TEST_REFUSED_STORE( 5, sb, UNMAPPED, 1 );

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

  TEST_CASE( 14, a4, 9, la a0, trap_count; lw a4, 0(a0); )

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

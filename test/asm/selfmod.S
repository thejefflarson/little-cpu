#include "riscv_test.h"
#include "test_macros.h"

// A store into .text, a fence.i, and then the stored word runs as an instruction.

RVTEST_CODE_BEGIN

  li    a0, 0

  la    t0, tmpl_bump_a0
  lw    t1, 0(t0)

  .align 2
  .option push
  .option norvc
  sw    t1, %lo(patch_adjacent)(x0)
  fence.i
patch_adjacent:
  nop
  .option pop

  TEST_CASE( 2, a0, 1, nop; )

  // The same thing at a distance: patch a stub, fence, then call it.

  la    t0, tmpl_set_a1
  lw    t1, 0(t0)
  la    t0, patch_stub
  sw    t1, 0(t0)
  fence.i

  li    a1, 0
  jal   ra, patch_stub

  TEST_CASE( 3, a1, 0x2a, nop; )

  la    t0, tmpl_nop
  lw    t1, 0(t0)
  la    t0, patch_stub
  sw    t1, 0(t0)
  fence.i

  li    a1, 0
  jal   ra, patch_stub

  TEST_CASE( 4, a1, 0, nop; )

  TEST_PASSFAIL

  .align 2
  .option push
  .option norvc
patch_stub:
  nop
  ret

tmpl_bump_a0:
  addi  a0, a0, 1
tmpl_set_a1:
  li    a1, 0x2a
tmpl_nop:
  nop
  .option pop

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN
  TEST_DATA
RVTEST_DATA_END

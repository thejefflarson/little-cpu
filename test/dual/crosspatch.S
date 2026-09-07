#include "dual_test.h"

// One hart patches an instruction; the other hart runs it.

#define PATCH_OLD 0x11
#define PATCH_NEW 0x2a

// 1 belongs to test/dual/dual_test.h.
#define TN_WAIT_PATCH 3
#define TN_WAIT_READY 4
#define TN_JOIN       5

DUALTEST_CODE_BEGIN

  li    a0, 0
  jal   ra, patch_stub
  TEST_CASE( 6, a0, PATCH_OLD, nop; )

  li    t1, 1
  la    t0, bready
  sw    t1, 0(t0)

  DUALTEST_WAIT( TN_WAIT_PATCH, patched, 1 )

  fence.i

  li    a0, 0
  jal   ra, patch_stub
  TEST_CASE( 7, a0, PATCH_NEW, nop; )

  DUALTEST_JOIN( TN_JOIN )
  TEST_PASSFAIL

hart1:
  DUALTEST_WAIT( TN_WAIT_READY, bready, 1 )

  la    t0, tmpl_new
  lw    t1, 0(t0)
  la    t0, patch_stub
  sw    t1, 0(t0)

  fence

  li    t1, 1
  la    t0, patched
  sw    t1, 0(t0)
  DUALTEST_HART1_DONE

  .align 2
  .option push
  .option norvc
patch_stub:
  addi  a0, x0, PATCH_OLD
  ret

tmpl_new:
  addi  a0, x0, PATCH_NEW
  .option pop

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  DUALTEST_DATA

  .align 2
bready:
  .word 0
patched:
  .word 0

RVTEST_DATA_END

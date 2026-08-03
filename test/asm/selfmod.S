#include "riscv_test.h"
#include "test_macros.h"

// A store into .text, a fence.i, and then the stored word runs as an
// instruction. Text and data share one address space, so `sw` can reach the
// instruction memory.
//
// Test 2 is the case the fence has to cover. Decode publishes the fetch address
// a cycle early, so the word for the instruction after `fence.i` is read out of
// the banks on the edge `fence.i` issues. `fence.i` waits for the pipeline to
// empty, which puts a store still in flight ahead of that read.
//
// The store in test 2 has to use x0 as its base. Decode presents rs1 one cycle
// and issues the next, and `fence.i`'s rs1 field is zero, so it can only issue
// one cycle behind a store that presented rs1 = x0 too. Behind any other store
// it issues a cycle later, the write has already landed, and the test passes
// with the wait taken out of the decoder. Writing it as
// `la t0, patch_adjacent; sw t1, 0(t0)` reads better and checks nothing. The
// `%lo` form needs the patch site inside the first 2 KB of .text; past that the
// store address comes out negative and test 2 fails.
//
// Every patch site is `.option norvc`, so it is exactly the four bytes the
// `sw` writes, and follows an `.align 2` so the store address is word aligned.
// The replacement encodings are assembled as templates and loaded out of .text
// rather than written as hex here.

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

  #-------------------------------------------------------------
  # The same thing at a distance: patch a stub, fence, then call it.
  #-------------------------------------------------------------

  la    t0, tmpl_set_a1
  lw    t1, 0(t0)
  la    t0, patch_stub
  sw    t1, 0(t0)
  fence.i

  li    a1, 0
  jal   ra, patch_stub

  TEST_CASE( 3, a1, 0x2a, nop; )

  #-------------------------------------------------------------
  # Writing the original word back and calling the stub again. Without this a
  # ROM image that already held the patched word would pass test 3.
  #-------------------------------------------------------------

  la    t0, tmpl_nop
  lw    t1, 0(t0)
  la    t0, patch_stub
  sw    t1, 0(t0)
  fence.i

  li    a1, 0
  jal   ra, patch_stub

  TEST_CASE( 4, a1, 0, nop; )

  TEST_PASSFAIL

  # Control cannot fall in here: every path above ends in TEST_PASSFAIL's
  # infinite loop. The stub is only ever entered by the `jal` above.
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

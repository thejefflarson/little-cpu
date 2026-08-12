#include "riscv_test.h"
#include "test_macros.h"

// A store into .text, a fence.i, and then the stored word runs as an
// instruction. Text and data share one address space, so `sw` can reach the
// instruction memory.
//
// Test 2 is the tightest spelling of that: the store is the instruction right
// before the fence, so the patched word is fetched on the earliest cycle
// anything has to cover. Two mechanisms cover it, and either alone is enough --
// deleting `instr_fencei` from `serialize` in rtl/decoder.v leaves this green,
// so does dropping the store from `text_access` in rtl/imemory.v, and doing
// both fails test 2. The wait is what orders a store against a fetch on a
// memory that answers fetch and data at once; on this one the write also takes
// the fetch port for its own cycle, which holds the next fetch back by itself.
// test/decoder_tb.v is what grades the wait.
//
// The store keeps x0 as its base so the `fence.i` cannot need an operand cycle
// behind it, which is what keeps the patched word in that earliest slot. Decode
// now guesses the next instruction's register pair out of the fetch window, so
// `la t0, patch_adjacent; sw t1, 0(t0)` measures the same thing today; the x0
// form does not depend on that guess. It does need the patch site inside the
// first 2 KB of .text -- past that the `%lo` store address comes out negative
// and test 2 fails.
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

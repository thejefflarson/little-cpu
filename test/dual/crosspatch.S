#include "dual_test.h"

// One hart patches an instruction; the other hart runs it. This is the program
// private text could never have passed, so it is also the grader for the two
// harts fetching out of one storage.
//
// The sequence is the specification's own protocol for cross-hart instruction
// patching, and nothing else:
//
//   hart 0 calls the stub and observes the OLD instruction, then signals;
//   hart 1 writes the new word, executes a data FENCE, then signals;
//   hart 0 executes its own FENCE.I and calls the stub again.
//
// The old-value call comes first and is ordered before the patch by a flag, so
// an image that already held the patched word is caught rather than passed:
// without it a stub that returned the new value from power-on would look
// identical.
//
// WHICH HART PATCHES IS DELIBERATE. Hart 1 does, and it reads the replacement
// encoding out of `.text` with an ordinary load. A load has to come out of one
// window's read port and the windows all hold the same words, so a load from
// the hart that does not own that window has to steal it from the hart that
// does. Putting the template load on hart 1 is what puts that steal on this
// program's path; on hart 0 it would be an ordinary read of its own window.
//
// WHAT THIS DOES NOT TEST, and the distinction is easy to lose. `fence.i` on
// this core also waits for the pipeline to drain, and no program can observe
// that: a store into text takes the instruction memory's write cycle, which
// holds the next fetch back by itself. That finding was measured on one hart
// and nothing here changes it -- the write reaches every window on one edge and
// stalls every window's fetch, so the earliest fetch the wait could still be
// covering is one the memory already held back on both harts. What this program
// exercises is the software protocol above and the storage being one storage.

#define PATCH_OLD 0x11
#define PATCH_NEW 0x2a

// 1 and 2 belong to test/dual/dual_test.h.
#define TN_WAIT_PATCH 3
#define TN_WAIT_READY 4
#define TN_JOIN       5

DUALTEST_CODE_BEGIN

  # The stub as the image holds it. Ordered before the patch by `bready`.
  li    a0, 0
  jal   ra, patch_stub
  TEST_CASE( 6, a0, PATCH_OLD, nop; )

  li    t1, 1
  la    t0, bready
  sw    t1, 0(t0)

  DUALTEST_WAIT( TN_WAIT_PATCH, patched, 1 )

  # The other half of the protocol. Hart 1's store is committed and its data
  # fence has retired; this is what makes the new word visible to THIS hart's
  # instruction fetch.
  fence.i

  li    a0, 0
  jal   ra, patch_stub
  TEST_CASE( 7, a0, PATCH_NEW, nop; )

  DUALTEST_JOIN( TN_JOIN )
  TEST_PASSFAIL

hart1:
  DUALTEST_WAIT( TN_WAIT_READY, bready, 1 )

  # A load from `.text`, issued by the hart that does not own the window a load
  # is answered from.
  la    t0, tmpl_new
  lw    t1, 0(t0)
  la    t0, patch_stub
  sw    t1, 0(t0)

  # The specification asks the writing hart for a data fence before it tells the
  # others to fence their instruction fetch. It orders nothing extra on this
  # machine -- one bus, no store buffer, one outstanding access per hart -- and
  # is executed because the protocol is the specification's, not this machine's.
  fence

  li    t1, 1
  la    t0, patched
  sw    t1, 0(t0)
  DUALTEST_HART1_DONE

  # Control cannot fall in here: both harts above end in an infinite loop. The
  # stub is only ever entered by the two `jal`s, and the template is never
  # executed at all -- it is loaded as data.
  .align 2
  .option push
  .option norvc
patch_stub:
  addi  a0, x0, PATCH_OLD
  ret

tmpl_new:
  addi  a0, x0, PATCH_NEW
  .option pop

DUALTEST_CODE_END

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

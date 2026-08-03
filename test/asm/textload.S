#include "riscv_test.h"
#include "test_macros.h"

// The handler reads the faulting instruction out of .text, works out whether
// it was two bytes or four, and resumes past it.
//
// That is why none of the trapping instructions below is wrapped in
// `.option norvc`. riscv_test.h's shared handler resumes at mepc+4 and has no
// way to tell the two lengths apart, so every other trapping test in this
// suite has to keep the assembler from compressing the instruction that
// faults. Text and data share one address space here, so a load can reach the
// encoding and the wrapper is not needed.
//
// mepc can sit two bytes past a word boundary, so the handler word-aligns it
// before the load and takes the upper halfword when bit 1 is set. A word load
// at an odd halfword would fault inside the handler.
//
// The handler clobbers t0, t1 and t2, so nothing here holds a live value in
// any of them across a fault.

RVTEST_CODE_BEGIN

  # mtvec resets to 0, which is _start -- install before faulting.
  RVTEST_INSTALL_TRAP_HANDLER

  #-------------------------------------------------------------
  # A compressed breakpoint at pc % 4 == 0. The `c.li` behind it is two bytes,
  # so a handler that resumed at mepc+4 would step over it and leave a0 at 0.
  #-------------------------------------------------------------

  li    a0, 0

  .option push
  .option rvc
  .align 2
cebrk_aligned:
  c.ebreak
  c.li  a0, 5
  .option pop

  TEST_CASE( 2, a4, 3, la a3, trap_cause; lw a4, 0(a3); )
  TEST_CASE( 3, a0, 5, nop; )

test_4:
  la    a3, trap_epc
  lw    a4, 0(a3)
  la    x29, cebrk_aligned
  li    TESTNUM, 4
  bne   a4, x29, fail

  #-------------------------------------------------------------
  # The same at pc % 4 == 2, which is the halfword the handler has to shift
  # down before it looks at the low two bits.
  #-------------------------------------------------------------

  li    a1, 0

  .option push
  .option rvc
  .align 2
  c.nop
cebrk_odd:
  c.ebreak
  c.li  a1, 6
  .option pop

  TEST_CASE( 5, a4, 3, la a3, trap_cause; lw a4, 0(a3); )
  TEST_CASE( 6, a1, 6, nop; )
  TEST_CASE( 7, a4, 2, la a3, cebrk_odd; andi a4, a3, 3; )

test_8:
  la    a3, trap_epc
  lw    a4, 0(a3)
  la    x29, cebrk_odd
  li    TESTNUM, 8
  bne   a4, x29, fail

  #-------------------------------------------------------------
  # A four-byte fault, so the other arm of the length decision runs. Resuming
  # two bytes early here would land inside `ecall`'s upper halfword, which is
  # zero and decodes to nothing, and the cause and the count would both move.
  #-------------------------------------------------------------

  li    a2, 0
  ecall
  li    a2, 7

  TEST_CASE(  9, a4, 11, la a3, trap_cause; lw a4, 0(a3); )
  TEST_CASE( 10, a2, 7, nop; )

  #-------------------------------------------------------------
  # A compressed instruction whose cause comes from its operand address rather
  # than from its encoding, so the length decision is exercised apart from the
  # breakpoint path.
  #-------------------------------------------------------------

  la    a0, tdat
  addi  a0, a0, 1
  li    a1, 0x5a5

  .option push
  .option rvc
  .align 2
mis_clw:
  c.lw  a1, 0(a0)
  c.li  a2, 9
  .option pop

  TEST_CASE( 11, a4, 4, la a3, trap_cause; lw a4, 0(a3); )
  TEST_CASE( 12, a1, 0x5a5, nop; )
  TEST_CASE( 13, a2, 9, nop; )

test_14:
  la    a3, trap_epc
  lw    a4, 0(a3)
  la    x29, mis_clw
  li    TESTNUM, 14
  bne   a4, x29, fail

  TEST_CASE( 15, a4, 4, la a3, trap_count; lw a4, 0(a3); )

  TEST_PASSFAIL

  # Reached only through mtvec. RVTEST_INSTALL_TRAP_HANDLER installs this
  # label, so the name has to be trap_handler.
  .align 2
trap_handler:
  .option push
  .option norvc
  la    t0, trap_count
  lw    t1, 0(t0)
  addi  t1, t1, 1
  sw    t1, 0(t0)
  csrr  t1, mcause
  la    t0, trap_cause
  sw    t1, 0(t0)
  csrr  t0, mepc
  la    t1, trap_epc
  sw    t0, 0(t1)

  # The load that this test exists for: the faulting instruction, read back out
  # of .text.
  andi  t1, t0, -4
  lw    t1, 0(t1)
  andi  t2, t0, 2
  beqz  t2, 1f
  srli  t1, t1, 16
1:
  # Both low bits set means four bytes; anything else is two.
  andi  t1, t1, 3
  li    t2, 3
  addi  t0, t0, 2
  bne   t1, t2, 2f
  addi  t0, t0, 2
2:
  csrw  mepc, t0
  mret
  .option pop

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  RVTEST_TRAP_DATA

  # Only its address is used. The `c.lw` one byte past it traps before reading.
  .align 2
tdat:
  .word 0x0ff0

RVTEST_DATA_END

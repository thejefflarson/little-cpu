#include "riscv_test.h"
#include "test_macros.h"

// fence, fence.i, wfi and c.nop: the RV32IMC instructions with no riscv-formal
// spec model at the pin AND no other test.
//
// formal/EXPECTED_CHECKS is set-equal to riscv-formal's own
// insns/isa_rv32imc.txt, so every other instruction in the ISA has an `insn_*`
// ladder check. The pin models none of `c_nop`, `c_ebreak`, `fence`, `ecall`,
// `ebreak` -- and that five-instruction blind spot is where the c.ebreak trap
// cause defect lived. `ecall` and `ebreak` are covered by trap.S,
// `c.ebreak` by cebreak.S; these four had nothing.
//
// Three of them are NOPs here: `c.nop` always, `fence` because one hart and no
// caches leave it nothing to order, and `wfi` because the spec lets an
// implementation resume for any reason. `fence.i` is not -- it waits until the
// pipeline is empty, because text is writable and the fetch address goes out a
// cycle early, so an older store's write edge has to land first. Delete that
// wait and no program in this suite goes red, this one included; the only
// grader that currently catches it is test/decoder_tb.v. What is checked here
// is that all four retire without trapping and without disturbing
// architectural state -- the two ways any of them can be wrong.

RVTEST_CODE_BEGIN

  li    a0, 0x0BADF00D
  li    a1, 0x12345678
  li    a2, 0x0000FFFF

  fence
  fence.i
  wfi
  .option push
  .option rvc
  c.nop
  .option pop

  TEST_CASE( 1, a0, 0x0BADF00D, nop )
  TEST_CASE( 2, a1, 0x12345678, nop )
  TEST_CASE( 3, a2, 0x0000FFFF, nop )

  # They must also not disturb the PC: falling through to here at all is the
  # check, and the counter proves we executed rather than skipped.
  li    a3, 0
  fence
  addi  a3, a3, 1
  wfi
  addi  a3, a3, 1
  TEST_CASE( 4, a3, 2, nop )

  TEST_PASSFAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN
  TEST_DATA
RVTEST_DATA_END

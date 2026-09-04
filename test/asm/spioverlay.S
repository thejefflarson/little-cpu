# See LICENSE for license details.

#*****************************************************************************
# spioverlay.S
#-----------------------------------------------------------------------------
#
# A boot loader's other half: not just reading the flash, but using what it
# reads. `spi_read_bytes` below copies a run of bytes out of the flash and
# into wherever it is told, and this program calls it twice -- once into the
# writable text window, to run the result, and once into RAM, to check it.
#
# WHY THE TEXT WINDOW AND NOT A BIGGER ROM. The part's block RAM stops at
# 8 KB and there is no size between that and one that does not fit; the flash
# is 4 MB the design can already read a byte of. So growing what a program can
# do after boot is not a bigger image, it is fetching a segment at run time
# and overwriting some of the 8 KB with it -- the same self-modifying-code
# mechanism test/asm/selfmod.S exercises, sourced from the flash controller
# instead of a local template.
#
# WHAT PROVES THE OVERLAY RAN, RATHER THAN A STUB THAT WAS ALREADY THERE. The
# slot patched below is loaded three times: once from a template in this
# file's own text (test 2), once from the flash (test 3), and once from a
# second, different template (test 4). All three write the same instruction
# shape -- `addi s8, s10, imm` -- so the only thing that can make one test's
# answer differ from the others is which bytes actually landed in the slot.
# A loader that silently failed to copy, or a fence.i that did not order the
# write ahead of the fetch, would leave test 2's 111 in place for test 3 to
# read back, not 224. Deleting either mechanism reads exactly the way it does
# in test/asm/selfmod.S: dropping `instr_fencei` from decode's serialize term
# or the store from imemory's own text-write range leaves either alone able to
# cover the wait, and both would have to go for this file to stop catching it.
#
# THE INSTRUCTION ITSELF IS NOT CHOSEN, IT IS WHAT THE FLASH MODEL'S FORMULA
# GIVES AT THIS ADDRESS. test/spiflash_model.v's data array is
# `a[7:0] ^ a[15:8] ^ 0x5a`, a fixed function of the low 16 bits of the
# address and nothing this program gets to pick beyond which address to read.
# `0x00064f` is the one address in that whole 16-bit space this file searched
# for and found: the four bytes it returns decode as `addi s8, s10, 224` on
# their own, with no hex literal chosen to match. `rd`/`rs1` came out fixed by
# that same search, which is why the two local templates (test 2, test 4) are
# written to match rather than the other way round.
#
# WHY A SECOND, LARGER READ INTO RAM RATHER THAN INTO TEXT TOO. `spi_read_bytes`
# does not know or care whether its destination is code, and test 5 shows
# that: sixteen bytes read straight into a RAM buffer and checked one at a
# time against the model's own formula, recomputed here the way
# test/asm/spiflash.S recomputes the JEDEC id rather than quoting bytes. That
# is the persistent-storage and firmware-update shape the same controller
# supports, next to test 3's execute-in-place one.
#
# THIS PROGRAM CANNOT AGREE WITH THE REFERENCE MODEL, for the reason
# test/asm/spiflash.S does not: Sail has plain memory where the flash
# controller sits, so `busy` is never set and every polled load reads back
# whatever was last stored rather than a real exchange's answer -- and every
# byte this program's own read loop stores as its outgoing dummy is 0x00, so
# every byte it reads back on Sail is 0x00 too. The four bytes patched into
# `patch_stub` come back all zero, which is an illegal instruction, and that
# is why this file installs a trap handler where test/asm/selfmod.S needs
# none: without one, the untrapped fault resumes at `mtvec` == 0 and restarts
# the whole program, forever, on Sail alone -- an actual infinite loop, not a
# budget a bigger `--inst-limit` would clear. With the handler, Sail resumes
# past the fault with `s8` still holding test 2's 111, test 3's own check
# takes the `fail` branch, and the two traces diverge at a value instead.
#

#include "riscv_test.h"
#include "test_macros.h"

# The address this file's own search picked; see the header above.
#define OVERLAY_FLASH_ADDR 0x00064f

# Any address whose low sixteen bits plus BULK_LEN stay inside one 256-byte
# block, so the whole read shares one high byte and the check loop's formula
# does not have to track it changing mid-run.
#define BULK_FLASH_ADDR    0x00002000
#define BULK_LEN           16

RVTEST_CODE_BEGIN

  RVTEST_INSTALL_TRAP_HANDLER

  # The one operand every patch_stub variant reads, set once so a fresh 111,
  # 224 or 77 in the slot is the only thing that can move s8.
  li    s10, 0

  #-------------------------------------------------------------
  # Baseline: patch the stub from this file's own text, not the flash, and
  # confirm the call site reads back what the store just wrote.
  #-------------------------------------------------------------

  la    t0, tmpl_addi_111
  lw    t1, 0(t0)
  la    t0, patch_stub
  sw    t1, 0(t0)
  fence.i

  jal   ra, patch_stub
  TEST_CASE( 2, s8, 111, nop; )

  #-------------------------------------------------------------
  # The overlay: one instruction's worth of the flash, copied into the same
  # slot and run.
  #-------------------------------------------------------------

  li    a0, OVERLAY_FLASH_ADDR
  la    a1, patch_stub
  li    a2, 4
  jal   ra, spi_read_bytes
  fence.i

  jal   ra, patch_stub
  TEST_CASE( 3, s8, 224, nop; )

  #-------------------------------------------------------------
  # Revert to a second local template. Without this, a controller that always
  # answered 224 -- or a slot the ROM image already held that value in --
  # would have passed test 3 for the wrong reason.
  #-------------------------------------------------------------

  la    t0, tmpl_addi_77
  lw    t1, 0(t0)
  la    t0, patch_stub
  sw    t1, 0(t0)
  fence.i

  jal   ra, patch_stub
  TEST_CASE( 4, s8, 77, nop; )

  #-------------------------------------------------------------
  # A longer segment, read as data and checked byte by byte against the
  # model's own formula rather than executed.
  #-------------------------------------------------------------

  li    a0, BULK_FLASH_ADDR
  la    a1, bulk_buf
  li    a2, BULK_LEN
  jal   ra, spi_read_bytes

  li    TESTNUM, 5
  la    t3, bulk_buf
  li    t4, BULK_FLASH_ADDR
  li    t5, BULK_LEN
1:
  beqz  t5, 2f
  lbu   a3, 0(t3)
  andi  a4, t4, 0xff
  srli  a5, t4, 8
  andi  a5, a5, 0xff
  xor   a4, a4, a5
  xori  a4, a4, 0x5a
  bne   a3, a4, fail
  addi  t3, t3, 1
  addi  t4, t4, 1
  addi  t5, t5, -1
  j     1b
2:

  TEST_PASSFAIL

  RVTEST_TRAP_HANDLER

  # Control cannot fall in here: every path above ends in TEST_PASSFAIL's
  # infinite loop, or in the trap handler's `mret` back into it.

  .align 2
  .option push
  .option norvc
patch_stub:
  nop
  ret

tmpl_addi_111:
  addi  s8, s10, 111
tmpl_addi_77:
  addi  s8, s10, 77
  .option pop

  #-------------------------------------------------------------
  # a0 = flash byte address (24 bits used, the rest must be zero), a1 = dest,
  # a2 = byte count (> 0). Selects the flash, sends a sequential-read command
  # and the three address bytes, clocks a2 bytes into (a1), releases the
  # chip select. Clobbers t0-t4 and a0-a2.
  #-------------------------------------------------------------
spi_read_bytes:
  li      t0, SPI_BASE
  SPI_SELECT(t0, 1)
  SPI_XFER(t0, 0x03)
  srli    t3, a0, 16
  SPI_XFER_REG(t0, t3, t4)
  srli    t3, a0, 8
  SPI_XFER_REG(t0, t3, t4)
  SPI_XFER_REG(t0, a0, t4)
  # A named label, not a numeric local one: SPI_WAIT_IDLE and SPI_XFER_REG
  # both define their own "1:" on every expansion, and GAS resolves "1b" to
  # the NEAREST prior "1:" -- so a numeric label here would resolve inside
  # this loop's own body once it had run once, skipping the `sw` that starts
  # each new exchange and re-reading the first byte forever.
.Lspi_read_next_byte:
  SPI_XFER_REG(t0, x0, t4)
  sb      t4, 0(a1)
  addi    a1, a1, 1
  addi    a2, a2, -1
  bnez    a2, .Lspi_read_next_byte
  SPI_SELECT(t0, 0)
  ret

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  RVTEST_TRAP_DATA

  .align 2
bulk_buf:
  .space BULK_LEN

RVTEST_DATA_END

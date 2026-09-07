# See LICENSE for license details.

// A boot loader's other half: not just reading the flash, but using what it reads.

#include "riscv_test.h"
#include "test_macros.h"

// The address this file's own search picked; see the header above.
#define OVERLAY_FLASH_ADDR 0x00064f

#define BULK_FLASH_ADDR    0x00002000
#define BULK_LEN           16

RVTEST_CODE_BEGIN

  RVTEST_INSTALL_TRAP_HANDLER

  li    s10, 0

  la    t0, tmpl_addi_111
  lw    t1, 0(t0)
  la    t0, patch_stub
  sw    t1, 0(t0)
  fence.i

  jal   ra, patch_stub
  TEST_CASE( 2, s8, 111, nop; )

  // The overlay: one instruction's worth of the flash, copied into the same slot and run.

  li    a0, OVERLAY_FLASH_ADDR
  la    a1, patch_stub
  li    a2, 4
  jal   ra, spi_read_bytes
  fence.i

  jal   ra, patch_stub
  TEST_CASE( 3, s8, 224, nop; )

  la    t0, tmpl_addi_77
  lw    t1, 0(t0)
  la    t0, patch_stub
  sw    t1, 0(t0)
  fence.i

  jal   ra, patch_stub
  TEST_CASE( 4, s8, 77, nop; )

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

spi_read_bytes:
  li      t0, SPI_BASE
  SPI_SELECT(t0, 1)
  SPI_XFER(t0, 0x03)
  srli    t3, a0, 16
  SPI_XFER_REG(t0, t3, t4)
  srli    t3, a0, 8
  SPI_XFER_REG(t0, t3, t4)
  SPI_XFER_REG(t0, a0, t4)
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

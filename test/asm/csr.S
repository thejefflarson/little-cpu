# See LICENSE for license details.

#*****************************************************************************
# csr.S
#-----------------------------------------------------------------------------
#
# The Zicsr access semantics and this core's CSR set, checked in band.
# riscv-formal has no spec model for csrr*/mret at the pinned SHA, so the
# per-retire monitor reports nothing about any instruction in this file (see
# riscv_test.h) — every claim here is asserted by the test itself.
#
# No trap is expected anywhere in this file, so it installs the *fatal* handler:
# an unexpected fault reports the live test number instead of restarting the
# program from mtvec = 0.
#

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  RVTEST_INSTALL_TRAP_HANDLER

  #-------------------------------------------------------------
  # A read/write CSR round-trips.
  #-------------------------------------------------------------

  TEST_CASE( 2, a1, 0x1234, \
    li a0, 0x1234; \
    csrw mscratch, a0; \
    csrr a1, mscratch; )

  #-------------------------------------------------------------
  # CSRRW returns the old value and installs the new one.
  #-------------------------------------------------------------

  TEST_CASE( 3, a1, 0x1234, \
    li a0, 0x5678; \
    csrrw a1, mscratch, a0; )

  TEST_CASE( 4, a1, 0x5678, csrr a1, mscratch; )

  #-------------------------------------------------------------
  # CSRRS with rs1 == x0 suppresses the write entirely, so it is
  # legal against a read-only CSR — if it raised illegal-instruction instead,
  # the fatal handler below would report this test number.
  #-------------------------------------------------------------

  TEST_CASE( 5, a0, 0x40001105, csrrs a0, misa, x0; )
  TEST_CASE( 6, a1, 0x5678, csrrs a0, mscratch, x0; csrr a1, mscratch; )

  #-------------------------------------------------------------
  # The immediate forms operate on a 5-bit zimm, not on rs1.
  #-------------------------------------------------------------

  TEST_CASE( 7, a1, 0x567f, csrrsi a0, mscratch, 0x1f; csrr a1, mscratch; )
  TEST_CASE( 8, a1, 0x5670, csrrci a0, mscratch, 0x0f; csrr a1, mscratch; )

  # ...and the value returned is the one from before the update: mscratch is
  # 0x5670 on entry and 0x5673 on exit, so only the pre-update read passes.
  TEST_CASE( 9, a0, 0x5670, csrrsi a0, mscratch, 0x03; )

  #-------------------------------------------------------------
  # The read-only identification CSRs.
  #-------------------------------------------------------------

  TEST_CASE( 10, a0, 0x40001105, csrr a0, misa; )
  TEST_CASE( 11, a0, 0, csrr a0, mvendorid; )
  TEST_CASE( 12, a0, 0, csrr a0, marchid; )
  TEST_CASE( 13, a0, 0, csrr a0, mimpid; )
  TEST_CASE( 14, a0, 0, csrr a0, mhartid; )

  # mtval is hardwired zero, so it reads zero out of reset and
  # after any trap. That much is true of the reference model too.
  TEST_CASE( 15, a0, 0, csrr a0, mtval; )

  # misa is read-only by VALUE, not by address: 0x301 is outside the read-only
  # encoding range, so `csrw misa` retires rather than trapping and the WARL
  # fallback swallows it. The spec allows a read-only misa, and CSRRW writes
  # unconditionally even with rs1 = x0 — so this is the one form that would
  # actually clear the A bit if anything could. The reference model agrees
  # because its config says `writable_misa: false`.
  TEST_CASE( 16, a0, 0x40001105, \
    csrw misa, x0; \
    csrr a0, misa; )

  # THREE ASSERTIONS USED TO LIVE HERE AND CANNOT — not in this file. They are
  # that `mie` reads zero, that `mip` reads zero, and that a write to either
  # (or to `mtval`) does not stick. All three are true of this core and all
  # three are implementation choices rather than ISA rules: with no interrupt
  # sources, mie and mip are read-only zero and mtval may be hardwired, while a
  # machine with a timer reports mip.MTIP set, mie's MSIE/MTIE/MEIE writable
  # and mtval writable — and is equally spec-legal.
  #
  # The Sail RISC-V model IS that other machine and cannot be configured out of
  # it. Measured against sail-riscv 0.13.1: `platform.clint.supported: false`
  # unmaps the CLINT's MMIO window but leaves mtime advancing, so mip.MTIP
  # comes up on the first tick; and the config schema names no knob for mie's
  # writable bits or for a read-only mtval.
  #
  # So an assertion here would not merely read a different value under
  # co-simulation — it would take a DIFFERENT BRANCH: the reference model runs
  # the program to `fail` where the core runs it to `pass`. No value exemption
  # can paper over that, and none should, because two runs of one program
  # taking different paths is exactly what that leg exists to report.
  #
  # The properties are not dropped. test/csr_tb.v asserts every one of them
  # directly against rtl/csrs.v — "mie reads 0", "mip reads 0", "mie ignores a
  # write", "mip ignores a write", "mtval ignores a write" — where there is no
  # reference model to disagree, which is where a claim about an
  # implementation-defined value belongs. It covers the other half in the same
  # checks: `implemented` is asserted high for all three addresses, so a `csrr`
  # of any of them retires here rather than trapping. `make test-units` runs it.

  #-------------------------------------------------------------
  # mtvec reads back the handler this test installed.
  #-------------------------------------------------------------

test_17:
  csrr  a0, mtvec
  la    x29, trap_handler
  li    TESTNUM, 17
  bne   a0, x29, fail

  #-------------------------------------------------------------
  # WARL fields. MPP is hardwired to M-mode, so it survives a zero write, and
  # mepc's bit 0 is always clear — only bit 0, because C makes 2-byte targets
  # legal.
  #-------------------------------------------------------------

  TEST_CASE( 18, a1, 0x1800, \
    csrw mstatus, x0; \
    csrr a0, mstatus; \
    li   a2, 0x1800; \
    and  a1, a0, a2; )

  TEST_CASE( 19, a1, 0x102, \
    li a0, 0x103; \
    csrw mepc, a0; \
    csrr a1, mepc; )

  #-------------------------------------------------------------
  # fence and wfi are NOPs here: they retire and change nothing.
  #-------------------------------------------------------------

  TEST_CASE( 20, a1, 7, \
    li a1, 7; \
    fence; \
    wfi; )

  TEST_PASSFAIL

  RVTEST_TRAP_HANDLER_FATAL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

RVTEST_DATA_END

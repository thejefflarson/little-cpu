# See LICENSE for license details.

#*****************************************************************************
# zicsr.S
#-----------------------------------------------------------------------------
#
# Zicsr's write- and read-suppression rules are tests on the ENCODING, never on
# a value, and whether an access to a read-only CSR is legal follows from them.
# That is the sharpest thing the unprivileged spec says about these six
# instructions and it is the easiest to implement as a value test by accident:
#
#   * CSRRS/CSRRC suppress the write when the SOURCE REGISTER IS x0. A register
#     that merely HOLDS zero is not x0, and the spec says so explicitly -- the
#     instruction still attempts the write and still causes its side effects,
#     which against a read-only CSR means an illegal-instruction trap.
#   * CSRRSI/CSRRCI suppress the write when the 5-bit uimm is zero.
#   * CSRRW/CSRRWI ALWAYS write, uimm == 0 included, so they trap against a
#     read-only CSR no matter what they carry.
#   * CSRRW/CSRRWI suppress the READ when rd is x0 -- and only the read. The
#     write still lands.
#
# "Read-only" is a property of the ADDRESS (csr[11:10] == 2'b11), not of the
# register's implementation. mvendorid (0xF11) is read-only by address, so a
# write to it faults; mtval/mie/mip (0x343/0x304/0x344) are hardwired zero on
# this core but their addresses are writable, so a write to one of them is a
# legal no-op and must NOT fault. Those two cases look identical from a
# program's point of view -- a CSR that never changes -- and are opposite in
# law.
#
# Nothing else in the suite reaches these. test/asm/csr.S installs the FATAL
# handler and therefore cannot contain a deliberate fault at all; test/asm/
# trap.S's two illegal-CSR cases are an unimplemented address and a plain
# `csrw mvendorid`, i.e. the two easy ones. riscv-formal ships no spec model
# for csrr* at the pinned SHA (`spec_valid` is 0 for every instruction in this
# file), so nothing here is cross-checked by the per-retire monitor -- see
# riscv_test.h constraint 3.
#
# Values, not just legality, are asserted wherever the value is an ISA
# consequence rather than an implementation choice. Where it is a choice --
# what mtval/mie/mip actually read -- only the absence of a trap is asserted,
# because the Sail reference model is a machine with interrupt sources and
# would legally disagree about the values (see the long note in csr.S).
#
# Every trapping instruction is `.option norvc`: the handler resumes at mepc+4
# unconditionally (riscv_test.h constraint 1). CSR instructions have no
# compressed form, so this is belt and braces rather than load-bearing, and it
# is written out so the file does not become the exception that teaches the
# rule wrong.
#
# The handler clobbers t0/t1, so nothing here holds a live value in either
# across a fault.
#

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  # mtvec resets to 0, which is _start — install before faulting.
  RVTEST_INSTALL_TRAP_HANDLER

  #-------------------------------------------------------------
  # CSRRS whose rs1 is a real register that happens to hold zero. The write is
  # NOT suppressed -- suppression keys off the encoding -- so this faults
  # against a read-only CSR, where `csrrs a0, mvendorid, x0` two cases below
  # does not. A core that tested the VALUE instead of the register number
  # would sail through here and be wrong in the one direction that never
  # produces a visible symptom until a program writes a read-only CSR on
  # purpose.
  #-------------------------------------------------------------

  li    a5, 0
  .option push
  .option norvc
  csrrs a0, mvendorid, a5
  .option pop

  TEST_CASE( 2, a4, 2, la a3, trap_cause; lw a4, 0(a3); )
  TEST_CASE( 3, a4, 1, la a3, trap_count; lw a4, 0(a3); )

  #-------------------------------------------------------------
  # The same instruction with x0 as the source: write suppressed, legal, and
  # it still reads. CSRRC likewise -- the suppression rule names both.
  #-------------------------------------------------------------

  TEST_CASE( 4, a0, 0, csrrs a0, mvendorid, x0; )
  TEST_CASE( 5, a0, 0, csrrc a0, mvendorid, x0; )

  # ...and neither of them trapped.
  TEST_CASE( 6, a4, 1, la a3, trap_count; lw a4, 0(a3); )

  #-------------------------------------------------------------
  # The immediate forms, zero uimm: write suppressed, legal against a
  # read-only CSR.
  #-------------------------------------------------------------

  TEST_CASE( 7, a0, 0, csrrsi a0, mvendorid, 0; )
  TEST_CASE( 8, a0, 0, csrrci a0, mvendorid, 0; )
  TEST_CASE( 9, a4, 1, la a3, trap_count; lw a4, 0(a3); )

  #-------------------------------------------------------------
  # ...and a NON-zero uimm is a write, so it faults.
  #-------------------------------------------------------------

  .option push
  .option norvc
  csrrsi a0, mvendorid, 1
  .option pop

  TEST_CASE( 10, a4, 2, la a3, trap_cause; lw a4, 0(a3); )
  TEST_CASE( 11, a4, 2, la a3, trap_count; lw a4, 0(a3); )

  .option push
  .option norvc
  csrrci a0, mvendorid, 1
  .option pop

  TEST_CASE( 12, a4, 3, la a3, trap_count; lw a4, 0(a3); )

  #-------------------------------------------------------------
  # CSRRWI always writes, even carrying a zero immediate -- the suppression
  # rule covers CSRRSI/CSRRCI only. So this faults where test 7 did not, on
  # the same CSR with the same immediate.
  #-------------------------------------------------------------

  .option push
  .option norvc
  csrrwi a0, mvendorid, 0
  .option pop

  TEST_CASE( 13, a4, 2, la a3, trap_cause; lw a4, 0(a3); )
  TEST_CASE( 14, a4, 4, la a3, trap_count; lw a4, 0(a3); )

  #-------------------------------------------------------------
  # The same rule seen from the other side: against a WRITABLE CSR, CSRRWI
  # with a zero immediate really does store zero.
  #-------------------------------------------------------------

  TEST_CASE( 15, a1, 0, \
    li a0, 0x7fff; \
    csrw mscratch, a0; \
    csrwi mscratch, 0; \
    csrr a1, mscratch; )

  #-------------------------------------------------------------
  # CSRRW with rd == x0 suppresses the READ ONLY. The write still lands.
  #-------------------------------------------------------------

  TEST_CASE( 16, a1, 0x1234, \
    li a0, 0x1234; \
    csrrw x0, mscratch, a0; \
    csrr a1, mscratch; )

  #-------------------------------------------------------------
  # The immediate forms take a 5-bit zimm out of the rs1 field, NOT the
  # contents of the register that field names. a5 is x15, so a core that read
  # the register file here would OR in 0xffff0000 instead of 15.
  #-------------------------------------------------------------

  TEST_CASE( 17, a1, 0x123f, \
    li a5, 0xffff0000; \
    csrrsi a0, mscratch, 15; \
    csrr a1, mscratch; )

  # ...and the value CSRRSI returned is the one from before its own update.
  TEST_CASE( 18, a0, 0x1234, nop; )

  #-------------------------------------------------------------
  # A write to a CSR that is hardwired but whose ADDRESS is writable is a
  # legal no-op, not a fault. This is the case that is opposite in law to
  # test 2 and identical in appearance. Only the absence of a trap is
  # asserted: what these three read is an implementation choice this core and
  # the Sail reference model legally differ on.
  #-------------------------------------------------------------

  li    a5, 0xffffffff
  csrw  mtval, a5
  csrw  mie, a5
  csrw  mip, a5

  TEST_CASE( 19, a4, 4, la a3, trap_count; lw a4, 0(a3); )

  #-------------------------------------------------------------
  # rd == rs1. Every CSR instruction is an atomic read-modify-write, so the
  # value handed to rd is the one from BEFORE the write even when the two name
  # the same register -- `csrrw a0, mscratch, a0` is a swap. On this core the
  # read result rides the `is_add` pass-through to writeback while the write
  # commits in decode off `reg_rs1`, so the two really do come from
  # different places and the ordering is a property rather than a tautology.
  #-------------------------------------------------------------

  TEST_CASE( 20, a0, 0x0abc, \
    li a0, 0x0abc; \
    csrw mscratch, a0; \
    li a0, 0x0def; \
    csrrw a0, mscratch, a0; )

  TEST_CASE( 21, a1, 0x0def, csrr a1, mscratch; )

  # The same for CSRRS, where the new value is a function of the old one.
  TEST_CASE( 22, a0, 0x0def, \
    li a0, 0x0f00; \
    csrrs a0, mscratch, a0; )

  TEST_CASE( 23, a1, 0x0fef, csrr a1, mscratch; )

  #-------------------------------------------------------------
  # Four traps taken, four handler entries, and the last cause recorded is
  # still 2 -- so nothing above trapped for a reason this file did not name.
  #-------------------------------------------------------------

  TEST_CASE( 24, a4, 2, la a3, trap_cause; lw a4, 0(a3); )
  TEST_CASE( 25, a4, 4, la a3, trap_count; lw a4, 0(a3); )

  TEST_PASSFAIL

  RVTEST_TRAP_HANDLER

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  RVTEST_TRAP_DATA

RVTEST_DATA_END

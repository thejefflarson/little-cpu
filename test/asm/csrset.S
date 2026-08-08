# See LICENSE for license details.

#*****************************************************************************
# csrset.S
#-----------------------------------------------------------------------------
#
# Two CSRs the RV32 privileged specification lists unconditionally in the
# machine-mode register tables. Both were once missing from rtl/csrs.v's
# `implemented` set, so an access to either raised illegal instruction where
# the spec says it should read zero. This file is what says they stay:
#
#   mstatush   (0x310)  priv 20211203 §3.1.6.4, "For RV32 only, mstatush is a
#                       32-bit read/write register". SBE and MBE are its only
#                       fields and both must read zero on a little-endian-only
#                       implementation, so implementing it is a constant-zero
#                       arm in the read mux -- the same shape mtval, mie and
#                       mip already have. Its address is writable
#                       (addr[11:10] == 2'b00), so a write to it must be an
#                       ignored no-op rather than a fault.
#   mconfigptr (0xF15)  priv 20211203 §3.1.4. Read-only, and "if the value is
#                       zero, no configuration data structure exists". Its
#                       address is read-only (addr[11:10] == 2'b11), so a
#                       write to it correctly faults already.
#
# THE CSR SET IS A FLOOR, NOT A CLOSED LIST, and that is why this file exists.
# Trapping on a register the spec mandates is non-conformance, not minimality:
# the spec lets almost every mandatory register read zero, so the two barely
# trade off. A scope statement calling the set "exact" is what once made this
# gap read as a design choice; do not remove either arm to shrink rtl/csrs.v.
#
# TWO THINGS THAT ARE *NOT* FINDINGS, checked the same way and recorded here so
# nobody re-opens them:
#
#   * mhpmcounter3 (0xB03), mhpmcounter3h (0xB83) and mhpmevent3 (0x323) --
#     sail-riscv 0.13.1 raises illegal instruction on all three, exactly as
#     this core does. The reference model agrees; there is nothing to close.
#   * mcountinhibit (0x320) -- the spec makes it explicitly optional ("if not
#     implemented, the implementation behaves as though the register were set
#     to zero"), so trapping is a legal way to not implement it. Sail happens
#     to implement it; that is Sail's choice, not a requirement.
#

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  # mtvec resets to 0, which is _start — install before faulting.
  # The RECORDING handler, not the fatal one: the failure this file reports has
  # to be an assertion about a trap count, so the trap must be survivable.
  RVTEST_INSTALL_TRAP_HANDLER

  #-------------------------------------------------------------
  # mstatush reads zero and does not trap.
  #-------------------------------------------------------------

  .option push
  .option norvc
  csrr  a0, 0x310
  .option pop

  TEST_CASE( 2, a4, 0, la a3, trap_count; lw a4, 0(a3); )
  TEST_CASE( 3, a0, 0, nop; )

  #-------------------------------------------------------------
  # A write to mstatush is a legal no-op: its address is writable, and both of
  # its fields are WARL with zero as the only legal value here.
  #-------------------------------------------------------------

  li    a5, 0xffffffff
  .option push
  .option norvc
  csrw  0x310, a5
  .option pop

  TEST_CASE( 4, a4, 0, la a3, trap_count; lw a4, 0(a3); )
  TEST_CASE( 5, a0, 0, csrr a0, 0x310; )

  #-------------------------------------------------------------
  # mconfigptr reads zero and does not trap.
  #-------------------------------------------------------------

  .option push
  .option norvc
  csrr  a0, 0xF15
  .option pop

  TEST_CASE( 6, a4, 0, la a3, trap_count; lw a4, 0(a3); )
  TEST_CASE( 7, a0, 0, nop; )

  #-------------------------------------------------------------
  # ...and a WRITE to it faults, because its address is read-only. This is the
  # half that is already correct, and it is asserted so that "implement
  # mconfigptr" cannot be discharged by making it writable.
  #-------------------------------------------------------------

  .option push
  .option norvc
  csrw  0xF15, a5
  .option pop

  TEST_CASE( 8, a4, 1, la a3, trap_count; lw a4, 0(a3); )
  TEST_CASE( 9, a4, 2, la a3, trap_cause; lw a4, 0(a3); )

  TEST_PASSFAIL

  RVTEST_TRAP_HANDLER

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA
  RVTEST_TRAP_DATA

RVTEST_DATA_END

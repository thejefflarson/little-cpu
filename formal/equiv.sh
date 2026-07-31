#!/bin/bash
# ADR-0006 / ADR-0020: proves that RISC-V_FORMAL instrumentation does not
# change core behaviour -- gold is the plain build, gate is the
# `RISCV_FORMAL` build with the rvfi_* ports deleted (memory_map/opt then
# sweep the now-dead shadow-payload logic that only fed them). Run from
# formal/ (`./equiv.sh`); paths below are relative to that.
#
# Per ADR-0020: as of `b2dafcc` this was argued (structurally: every
# `ifdef RISCV_FORMAL` block is write-only, read only by rtl/writeback.v's
# RVFI outputs) but not proven -- a hand-built equiv_make/equiv_simple/
# equiv_induct attempt did not converge in a practical budget. This is that
# attempt, ported to littlecpu. If it still does not converge, that is
# itself the finding ADR-0020 asks this script to produce, not something to
# work around by weakening the proof.
set -e
yosys -p '
  read_verilog -sv ../rtl/structs.v ../rtl/fetcher.v ../rtl/regfile.v ../rtl/csrs.v ../rtl/decoder.v ../rtl/executor.v ../rtl/accessor.v ../rtl/writeback.v ../rtl/littlecpu.v
  prep -flatten -top littlecpu
  design -stash gold
  read_verilog -D RISCV_FORMAL -sv ../rtl/structs.v ../rtl/fetcher.v ../rtl/regfile.v ../rtl/csrs.v ../rtl/decoder.v ../rtl/executor.v ../rtl/accessor.v ../rtl/writeback.v ../rtl/littlecpu.v
  prep -flatten -top littlecpu
  delete -port littlecpu/rvfi_*
  design -stash gate
  design -copy-from gold -as gold littlecpu
  design -copy-from gate -as gate littlecpu
  memory_map; opt -fast
  equiv_make gold gate equiv
  hierarchy -top equiv
  opt -fast
  equiv_simple
  equiv_induct
  equiv_status -assert
'

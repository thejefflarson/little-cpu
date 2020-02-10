#!/bin/bash
yosys -p '
  read_verilog -sv ../riscv.v
  prep -flatten -top riscv
  design -stash gold
  read_verilog -D RISCV_FORMAL -sv ../riscv.v
  prep -flatten -top riscv
  delete -port riscv/rvfi_*
  design -stash gate
  design -copy-from gold -as gold riscv
  design -copy-from gate -as gate riscv
  memory_map; opt -fast
  equiv_make gold gate equiv
  hierarchy -top equiv
  opt -fast
  equiv_simple
  equiv_induct
  equiv_status -assert
'

#include <iostream>
#include <fstream>
#include <backends/cxxrtl/cxxrtl_vcd.h>
#include "rtl.cc"

int main() {
  cxxrtl_design::p_testbench top;
  cxxrtl::debug_items all_debug_items;
  top.debug_info(all_debug_items);
  top.p_reset.set(true);
  top.step();
  cxxrtl::vcd_writer vcd;
  vcd.timescale(1, "us");
  vcd.add_without_memories(all_debug_items);
  std::ofstream waves("waves.vcd");
  for(int steps = 0; steps < 100; ++steps) {
    if (steps > 5) top.p_reset.set(false);
    top.p_clk.set<bool>(false);
    top.step();
    vcd.sample(steps*2 + 0);
    top.p_clk.set<bool>(true);
    top.step();
    vcd.sample(steps*2 + 1);
    waves << vcd.buffer;
    vcd.buffer.clear();
  }
}

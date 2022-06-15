#include "Vtestbench.h"
#include "verilated.h"
#include <stdint.h>

uint64_t main_time = 0;
double sc_time_stamp() {
    return main_time;
}

int main(int argc, char** argv, char** env) {
    VerilatedContext* contextp = new VerilatedContext;
    contextp->traceEverOn(true);
    contextp->commandArgs(argc, argv);
    Vtestbench* top = new Vtestbench{contextp};
    top->clk = 0;
    int t = 0;
    while (!Verilated::gotFinish()) {
      contextp->timeInc(1);
      if (!top->clk) {
        if (contextp->time() > 1 && contextp->time() < 10) {
          top->reset = 1;  // Assert reset
        } else {
          top->reset = 0;  // Deassert reset
        }
      }
      if (t > 500) {
        top->final();
        break;
      }
      top->clk = !top->clk;
      top->eval();
      t += 5;
	}

  delete top;
  delete contextp;
  return 0;
}

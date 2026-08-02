// The cxxrtl test runner (ADR-0007, ADR-0008): loads a `.text` and a
// `.data`/`.rodata`/`.bss` image straight into test/testbench.v's `rom` and
// `memory` arrays via debug_items, runs the design for a bounded number of
// cycles, and watches `tohost` (test/asm/riscv_test.h) for the riscv-tests
// pass/fail encoding.
//
// Exit codes: 0 = pass, 1 = fail (test number printed), 2 = cycle-limit
// timeout, 3 = usage/setup error (bad args, missing file, image too big for
// the simulated memories), 4 = RVFI monitor error (a per-retire mismatch,
// ADR-0006 — distinct from 1 because tohost never got a say: the failure is
// in the pipeline's own bookkeeping, not the test program's assertions),
// 5 = trap taken with mtvec == 0 (ADR-0029 — the program has silently
// restarted at `_start`; distinct from 2 because the timeout that would
// otherwise result says nothing about the fault that caused it),
// 6 = the per-retire monitor observed nothing: zero retires, or zero retires
// whose values its spec model checked. See the counter block in
// test/testbench.v for why that is a failure and not a curiosity — a monitor
// that never fires leaves every program passing off `tohost` alone, which
// ADR-0032 measured to be blind to real architectural corruption.
//
// Every run that actually simulates prints one machine-readable line before
// it exits:
//
//     RETIRES <n> SPEC-CHECKED <m>
//
// test/run_tests.sh parses it into the pass/fail table and grades it against
// test/OBSERVED_FLOOR. Making observation a printed, graded quantity rather
// than something inferred from a green run is the whole point of the two
// counters; a number nobody reads is the defect this closes, not a fix for it.
#include <cxxrtl/cxxrtl_vcd.h>
#include "rtl.cc"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

// test/asm/sections.lds' `ram` region starts here; the cxxrtl runner
// subtracts it back out of the `--ram` image's word addresses so they land
// at the right index in test/testbench.v's `memory` array. Keep in sync
// with sections.lds and test/testbench.v's RAM_BASE.
constexpr uint32_t kRamBase = 0x00010000;

// A parsed `objcopy -O verilog --verilog-data-width=4` image: word address
// (byte address / 4, per that format) -> 32-bit word.
using HexImage = std::map<uint32_t, uint32_t>;

bool parse_verilog_hex(const std::string &path, HexImage &image) {
  std::ifstream in(path);
  if (!in) {
    std::fprintf(stderr, "error: cannot open %s\n", path.c_str());
    return false;
  }
  uint32_t addr = 0;
  std::string line;
  while (std::getline(in, line)) {
    size_t start = line.find_first_not_of(" \t\r\n");
    if (start == std::string::npos)
      continue;
    if (line[start] == '@') {
      addr = std::strtoul(line.c_str() + start + 1, nullptr, 16);
      continue;
    }
    std::istringstream words(line);
    std::string word;
    while (words >> word)
      image[addr++] = std::strtoul(word.c_str(), nullptr, 16);
  }
  return true;
}

// Pokes `image` into the MEMORY debug item named `name`, offset by
// `base_words`. Returns false (with a diagnostic on stderr) if any address
// falls outside the simulated array — that means the test image no longer
// fits test/testbench.v's memories, not that the test failed.
bool load_image(cxxrtl::debug_items &items, const std::string &name,
                 const HexImage &image, uint32_t base_words) {
  const cxxrtl::debug_item &item = items.at(name).at(0);
  if (item.type != cxxrtl::debug_item::MEMORY) {
    std::fprintf(stderr, "error: debug item %s is not a memory\n", name.c_str());
    return false;
  }
  uint32_t *data = item.curr;
  for (const auto &[addr, word] : image) {
    if (addr < base_words || addr - base_words >= item.depth) {
      std::fprintf(stderr,
                    "error: %s image address 0x%08x is outside the simulated "
                    "%s (%zu words at base 0x%08x)\n",
                    name.c_str(), addr * 4, name.c_str(), item.depth, base_words * 4);
      return false;
    }
    data[addr - base_words] = word;
  }
  return true;
}

struct Args {
  std::string rom_path;
  std::string ram_path;
  std::string vcd_path;
  long cycles = 0;
};

bool parse_args(int argc, char **argv, Args &args) {
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    auto next = [&](const char *flag) -> const char * {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "error: %s requires an argument\n", flag);
        return nullptr;
      }
      return argv[++i];
    };
    if (arg == "--rom") {
      const char *v = next("--rom");
      if (!v) return false;
      args.rom_path = v;
    } else if (arg == "--ram") {
      const char *v = next("--ram");
      if (!v) return false;
      args.ram_path = v;
    } else if (arg == "--cycles") {
      const char *v = next("--cycles");
      if (!v) return false;
      args.cycles = std::strtol(v, nullptr, 10);
    } else if (arg == "--vcd") {
      const char *v = next("--vcd");
      if (!v) return false;
      args.vcd_path = v;
    } else {
      std::fprintf(stderr, "error: unrecognized argument '%s'\n", arg.c_str());
      return false;
    }
  }
  if (args.rom_path.empty() || args.ram_path.empty() || args.cycles <= 0) {
    std::fprintf(stderr,
                  "usage: sim --rom <hex> --ram <hex> --cycles N [--vcd out.vcd]\n");
    return false;
  }
  return true;
}

} // namespace

int main(int argc, char **argv) {
  Args args;
  if (!parse_args(argc, argv, args))
    return 3;

  HexImage rom_image, ram_image;
  if (!parse_verilog_hex(args.rom_path, rom_image))
    return 3;
  if (!parse_verilog_hex(args.ram_path, ram_image))
    return 3;

  cxxrtl_design::p_testbench top;
  cxxrtl::debug_items all_debug_items;
  top.debug_info(&all_debug_items, nullptr, "");

  if (!load_image(all_debug_items, "rom", rom_image, 0))
    return 3;
  if (!load_image(all_debug_items, "memory", ram_image, kRamBase / 4))
    return 3;

  // tohost is always the first word riscv_test.h places in the ram region
  // (RVTEST_DATA_BEGIN's `.tohost` input section is placed first in
  // sections.lds' `.data` output section), so it is always at kRamBase.
  const cxxrtl::debug_item &memory_item = all_debug_items.at("memory").at(0);
  uint32_t *ram_data = memory_item.curr;
  const uint32_t tohost_index = 0;

  // ADR-0006: test/testbench.v instantiates the RVFI monitor
  // (test/monitor.v, riding along under -D RISCV_FORMAL) alongside the DUT,
  // so this leg is self-checking per-retire, not merely end-state-checking
  // via tohost above. "monitor errcode" is the top-level error code the
  // generated monitor module registers for exactly the one cycle a
  // per-retire check fails (0 otherwise) -- its own $display diagnostics
  // (rvfi_* vs spec_* dump) already reach stdout via cxxrtl's native
  // $display support, so this only needs to turn a nonzero code into a
  // distinct, non-ignorable exit status.
  const cxxrtl::debug_item *monitor_errcode = nullptr;
  try {
    monitor_errcode = &all_debug_items.at("monitor errcode").at(0);
  } catch (const std::out_of_range &) {
    std::fprintf(stderr,
                  "error: RVFI monitor ('monitor errcode') not found in the "
                  "simulated design -- was test/rtl.cc built without "
                  "-D RISCV_FORMAL?\n");
    return 3;
  }

  // ADR-0029: `mtvec` resets to 0 and sections.lds links `.text` there, so a
  // trap taken before a handler is installed restarts the program at `_start`
  // — a livelock that reads as a timeout with no hint of the fault behind it.
  // test/testbench.v raises this the cycle after any trap that redirects to
  // address 0; see the comment there for why that needs no hierarchical
  // reference into the CSR file. Looked up the same way as the monitor's
  // errcode above, and missing is a setup error rather than something to
  // skip: a silently absent check is exactly what this exists to prevent.
  const cxxrtl::debug_item *trap_to_zero = nullptr;
  try {
    trap_to_zero = &all_debug_items.at("trap_to_zero").at(0);
  } catch (const std::out_of_range &) {
    std::fprintf(stderr,
                  "error: ADR-0029's trap-to-zero check ('trap_to_zero') not "
                  "found in the simulated design -- did test/testbench.v lose "
                  "the (* keep *) on it?\n");
    return 3;
  }

  // The observation counters (test/testbench.v). `rvfi_retires` counts the
  // cycles the monitor examined; `rvfi_spec_retires` counts the subset whose
  // values it actually compared against its spec model. Looked up the same way
  // and for the same reason as the two items above: a silently absent counter
  // would report the exact false green this exists to prevent, so missing is a
  // setup error rather than something to skip.
  const cxxrtl::debug_item *retires = nullptr;
  const cxxrtl::debug_item *spec_retires = nullptr;
  try {
    retires = &all_debug_items.at("rvfi_retires").at(0);
    spec_retires = &all_debug_items.at("rvfi_spec_retires").at(0);
  } catch (const std::out_of_range &) {
    std::fprintf(stderr,
                  "error: the monitor observation counters ('rvfi_retires', "
                  "'rvfi_spec_retires') were not found in the simulated design "
                  "-- did test/testbench.v lose the (* keep *) on them?\n");
    return 3;
  }

  // Printed on every path that reaches the simulation loop. Setup failures
  // above return before this point on purpose: nothing ran, so there is no
  // observation to report, and a "RETIRES 0" line for a run that never started
  // would be a claim rather than a measurement.
  auto report_counts = [&]() {
    std::printf("RETIRES %u SPEC-CHECKED %u\n", retires->curr[0],
                 spec_retires->curr[0]);
  };

  // Silence outranks the run's own verdict. A run whose oracle never fired has
  // no verdict worth reporting: `tohost` saying PASS is exactly what a blind
  // monitor looks like, and a FAIL from such a run cannot be attributed either.
  //
  // Exits 4 and 5 are the two exceptions and do NOT come through here. Neither
  // is a verdict the PROGRAM reached: an errcode is direct evidence that the
  // monitor fired, and trap-to-zero is a direct observation of the machine that
  // owes nothing to the monitor. Routing either through the silence gate would
  // replace a specific diagnosis with "the oracle was blind".
  //
  // For exit 5 that is not hypothetical, it is the ADR-0029 case itself. Traps
  // are detected and committed in DECODE (invariant 2) while a retire happens
  // in writeback, so a fault in the first few instructions of `_start` raises
  // trap_to_zero several cycles before anything has retired. Measured: `ecall`
  // as the second instruction gives `RETIRES 0` and used to return 6, which
  // test/run_tests.sh labels MONITOR-SILENT — pointing at the monitor for a
  // program that faulted before installing `mtvec`, which is precisely the
  // misattribution ADR-0029 added the TRAP-TO-ZERO label to prevent. The same
  // `ecall` after six retiring instructions returned 5. A named failure path
  // that is unreachable in its own scenario is the defect this repo keeps
  // finding in its grading layer, one level down.
  auto finish = [&](int code) {
    report_counts();
    uint32_t r = retires->curr[0];
    uint32_t s = spec_retires->curr[0];
    if (r == 0 || s == 0) {
      std::fprintf(stderr,
                    "the RVFI monitor observed nothing this run: %u retires, %u "
                    "of them spec-checked. The per-retire oracle was blind, so "
                    "this run's verdict (exit %d) means nothing -- see the "
                    "counter block in test/testbench.v.\n",
                    r, s, code);
      return 6;
    }
    return code;
  };

  std::unique_ptr<cxxrtl::vcd_writer> vcd;
  std::ofstream vcd_out;
  if (!args.vcd_path.empty()) {
    vcd = std::make_unique<cxxrtl::vcd_writer>();
    vcd->timescale(1, "us");
    vcd->add_without_memories(all_debug_items);
    vcd_out.open(args.vcd_path);
  }

  auto sample = [&](int64_t time) {
    if (!vcd) return;
    vcd->sample(time);
    vcd_out << vcd->buffer;
    vcd->buffer.clear();
  };

  top.p_reset.set(true);
  top.step();

  for (long cycle = 0; cycle < args.cycles; ++cycle) {
    if (cycle == 0)
      top.p_reset.set(false);
    top.p_clk.set<bool>(false);
    top.step();
    sample(cycle * 2 + 0);
    top.p_clk.set<bool>(true);
    top.step();
    sample(cycle * 2 + 1);

    uint32_t errcode = monitor_errcode->curr[0] & 0xffff;
    if (errcode != 0) {
      std::fprintf(stderr, "RVFI monitor error %u at cycle %ld\n", errcode, cycle);
      report_counts();
      return 4;
    }

    if ((trap_to_zero->curr[0] & 1) != 0) {
      std::fprintf(stderr,
                    "trap taken with mtvec == 0 at cycle %ld -- the handler was "
                    "never installed and the program has restarted at _start "
                    "(ADR-0029)\n",
                    cycle);
      report_counts();
      return 5;
    }

    uint32_t tohost = ram_data[tohost_index];
    if (tohost != 0) {
      if (tohost == 1) {
        std::printf("PASS\n");
        return finish(0);
      }
      uint32_t testnum = tohost >> 1;
      std::printf("FAIL %u\n", testnum);
      return finish(1);
    }
  }

  std::printf("TIMEOUT\n");
  return finish(2);
}

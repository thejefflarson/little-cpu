// The cxxrtl test runner (ADR-0007, ADR-0008): loads a `.text` and a
// `.data`/`.rodata`/`.bss` image straight into test/testbench.v's `rom` and
// `memory` arrays via debug_items, runs the design for a bounded number of
// cycles, and watches `tohost` (test/asm/riscv_test.h) for the riscv-tests
// pass/fail encoding.
//
// Exit codes: 0 = pass, 1 = fail (test number printed), 2 = cycle-limit
// timeout, 3 = usage/setup error (bad args, missing file, image too big for
// the simulated memories), 4 = RVFI monitor error (a per-retire mismatch,
// JEF-628 — distinct from 1 because tohost never got a say: the failure is
// in the pipeline's own bookkeeping, not the test program's assertions).
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

  // ADR-0006/JEF-628: test/testbench.v instantiates the RVFI monitor
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
      return 4;
    }

    uint32_t tohost = ram_data[tohost_index];
    if (tohost != 0) {
      if (tohost == 1) {
        std::printf("PASS\n");
        return 0;
      }
      uint32_t testnum = tohost >> 1;
      std::printf("FAIL %u\n", testnum);
      return 1;
    }
  }

  std::printf("TIMEOUT\n");
  return 2;
}

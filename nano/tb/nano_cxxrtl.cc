// The cxxrtl runner for nano: loads a flat word image into nano_testbench.v's one memory
// (there is no ROM/RAM split on nano's Von Neumann bus), runs the design for a bounded
// number of cycles, and watches either the riscv-tests tohost word (test/asm/riscv_test.h,
// the .S suite) or soc/compare/dhry_monitor.v's verdict word (Dhrystone/CoreMark).
#include <cxxrtl/cxxrtl_vcd.h>
#include "nano_rtl.cc"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

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

bool load_image(cxxrtl::debug_items &items, const std::string &name, const HexImage &image) {
  const cxxrtl::debug_item &item = items.at(name).at(0);
  if (item.type != cxxrtl::debug_item::MEMORY) {
    std::fprintf(stderr, "error: debug item %s is not a memory\n", name.c_str());
    return false;
  }
  uint32_t *data = item.curr;
  for (const auto &[addr, word] : image) {
    if (addr >= item.depth) {
      std::fprintf(stderr,
                    "error: %s image address 0x%08x is outside the simulated "
                    "%s (%zu words)\n",
                    name.c_str(), addr * 4, name.c_str(), item.depth);
      return false;
    }
    data[addr] = word;
  }
  return true;
}

struct Args {
  std::string rom_path;
  std::string ram_path;
  long cycles = 0;
  bool bench = false;
  std::string vcd_path;
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
    } else if (arg == "--bench") {
      args.bench = true;
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
                  "usage: nano_sim --rom <hex> --ram <hex> --cycles N [--bench] "
                  "[--vcd out.vcd]\n");
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

  cxxrtl_design::p_nano__testbench top;
  cxxrtl::debug_items items;
  top.debug_info(&items, nullptr, "");

  // Every word of the simulated memory is zeroed before either image is poked in: an
  // undefined word turns the whole pipeline X under iverilog and stays green under
  // cxxrtl, which is the divergence test/testbench.v's own zeroing loop exists to avoid.
  const cxxrtl::debug_item &mem_item = items.at("mem mem").at(0);
  std::memset(mem_item.curr, 0, mem_item.depth * sizeof(uint32_t));

  if (!load_image(items, "mem mem", rom_image))
    return 3;
  if (!load_image(items, "mem mem", ram_image))
    return 3;

  const cxxrtl::debug_item *monitor_errcode = nullptr;
  try {
    monitor_errcode = &items.at("monitor errcode").at(0);
  } catch (const std::out_of_range &) {
    std::fprintf(stderr,
                  "error: RVFI monitor ('monitor errcode') not found -- was "
                  "nano_rtl.cc built without -D RISCV_FORMAL?\n");
    return 3;
  }
  const cxxrtl::debug_item *retires = nullptr;
  try {
    retires = &items.at("rvfi_retires").at(0);
  } catch (const std::out_of_range &) {
    std::fprintf(stderr,
                  "error: the monitor retire counter ('rvfi_retires') was not "
                  "found -- did nano_testbench.v lose the (* keep *) on it?\n");
    return 3;
  }
  const cxxrtl::debug_item &trap_latched = items.at("trap_latched").at(0);
  const cxxrtl::debug_item &bench_marks = items.at("bench_marks").at(0);
  const cxxrtl::debug_item &bench_begin = items.at("bench_begin_cycle").at(0);
  const cxxrtl::debug_item &bench_end = items.at("bench_end_cycle").at(0);
  const cxxrtl::debug_item &bench_writes = items.at("bench_writes").at(0);
  const cxxrtl::debug_item &bench_verdict = items.at("bench_verdict").at(0);

  auto print_bench = [&]() {
    std::printf("BENCH marks=%u cycles=%u verdict=%u writes=%u\n",
                bench_marks.curr[0], bench_end.curr[0] - bench_begin.curr[0],
                bench_verdict.curr[0], bench_writes.curr[0]);
  };

  auto finish = [&](int code) {
    std::printf("RETIRES %u\n", retires->curr[0]);
    if (args.bench)
      print_bench();
    if (retires->curr[0] == 0) {
      std::fprintf(stderr,
                    "the RVFI monitor observed nothing this run: 0 retires. The "
                    "per-retire oracle was blind, so this run's verdict (exit "
                    "%d) means nothing.\n",
                    code);
      return 6;
    }
    return code;
  };

  std::unique_ptr<cxxrtl::vcd_writer> vcd;
  std::ofstream vcd_out;
  if (!args.vcd_path.empty()) {
    vcd = std::make_unique<cxxrtl::vcd_writer>();
    vcd->timescale(1, "us");
    vcd->add_without_memories(items);
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

  const uint32_t tohost_index = 0x00010000u / 4u;

  for (long cycle = 0; cycle < args.cycles; ++cycle) {
    top.p_clk.set<bool>(false);
    top.step();
    sample(cycle * 2 + 0);
    top.p_clk.set<bool>(true);
    top.step();
    sample(cycle * 2 + 1);

    if (cycle == 0)
      top.p_reset.set(false);

    uint32_t errcode = monitor_errcode->curr[0] & 0xffff;
    if (errcode != 0) {
      std::fprintf(stderr, "RVFI monitor error %u at cycle %ld\n", errcode, cycle);
      return finish(4);
    }

    if ((trap_latched.curr[0] & 1) != 0) {
      std::fprintf(stderr, "trap taken at cycle %ld -- nano halts permanently on a trap\n",
                   cycle);
      return finish(5);
    }

    if (args.bench) {
      if (bench_verdict.curr[0] != 0) {
        std::printf(bench_verdict.curr[0] == 1 ? "PASS\n" : "FAIL\n");
        return finish(bench_verdict.curr[0] == 1 ? 0 : 1);
      }
    } else {
      uint32_t tohost = mem_item.curr[tohost_index];
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
  }

  std::printf("TIMEOUT\n");
  return finish(2);
}

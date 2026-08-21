`timescale 1 ns / 1 ps
// The dual harness. It is a SEPARATE harness from test/testbench.v and
// test/dual_cxxrtl.cc is a separate runner, deliberately: the single-hart runner
// is a merge gate and must not grow a configuration axis, and it looks its
// signals up by flat debug-item name, which two of everything does not have.
//
// Two monitor instances read test/monitor.sim.v UNMODIFIED. That file is a
// plain module with no hierarchical reference in it, and its per-retire spec
// model is a function of one hart's `rvfi_*` stream -- it does not predict load
// data, so a load that observes the other hart's store is not something it has
// an opinion about. Two instances therefore need no per-hart configuration and
// test/sanitize_monitor.py is untouched.
//
// This harness has ONE leg. Two monitors roughly double a 7000-line generated
// module, so it stays off `make test`'s path and out of the iverilog flow;
// `make dual-elaborate` is the second frontend's look at it and elaborates
// only. Everything below that would be a `$display` in test/testbench.v is a
// register the runner reads instead.
module dual_testbench(
  input clk,
  input reset,
  // Hold hart 1 in reset for the whole run. THE HARNESS'S OWN RED DIRECTION: a
  // dual harness that is really measuring one hart reports the same thing a
  // working one does unless something makes the difference visible, so
  // test/dual_smoke.sh runs the same program both ways and requires the shared
  // counter to come out at N here and 2N without it.
  input hold_hart1
);
  // test/testbench.v's ROM, for test/asm/sections.lds' sake: both harnesses
  // link programs with the same script, so both simulate the same 16 KB text
  // window. The part cannot hold two copies of it, which is why nothing up5k
  // builds this.
  localparam int ROM_WORDS = 4096;

  logic [1:0] hart_reset;
  assign hart_reset = {reset | hold_hart1, reset};

  logic [1:0]   trap;
  logic [1:0]   rvfi_valid;
  logic [127:0] rvfi_order;
  logic [63:0]  rvfi_insn;
  logic [1:0]   rvfi_trap;
  logic [1:0]   rvfi_halt;
  logic [1:0]   rvfi_intr;
  logic [9:0]   rvfi_rs1_addr;
  logic [9:0]   rvfi_rs2_addr;
  logic [63:0]  rvfi_rs1_rdata;
  logic [63:0]  rvfi_rs2_rdata;
  logic [9:0]   rvfi_rd_addr;
  logic [63:0]  rvfi_rd_wdata;
  logic [63:0]  rvfi_pc_rdata;
  logic [63:0]  rvfi_pc_wdata;
  logic [63:0]  rvfi_mem_addr;
  logic [7:0]   rvfi_mem_rmask;
  logic [7:0]   rvfi_mem_wmask;
  logic [63:0]  rvfi_mem_rdata;
  logic [63:0]  rvfi_mem_wdata;
  logic [1:0]   probe_bus_active;
  logic [63:0]  probe_imem_addr;

  // No init files: the runner fills both banks through `debug_items`, after
  // zeroing every word of them. rtl/imemory.v holds ONE storage however many
  // windows read it, so there are two arrays here and not four -- the copies
  // two windows need are made by the mapper and no simulation of this RTL can
  // see them.
  littledual #(.ROM_WORDS(ROM_WORDS)) dut (
    .clk(clk),
    .reset(hart_reset),
    .trap(trap),
    .rvfi_valid(rvfi_valid),
    .rvfi_order(rvfi_order),
    .rvfi_insn(rvfi_insn),
    .rvfi_trap(rvfi_trap),
    .rvfi_halt(rvfi_halt),
    .rvfi_intr(rvfi_intr),
    .rvfi_rs1_addr(rvfi_rs1_addr),
    .rvfi_rs2_addr(rvfi_rs2_addr),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_rd_addr(rvfi_rd_addr),
    .rvfi_rd_wdata(rvfi_rd_wdata),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_pc_wdata(rvfi_pc_wdata),
    .rvfi_mem_addr(rvfi_mem_addr),
    .rvfi_mem_rmask(rvfi_mem_rmask),
    .rvfi_mem_wmask(rvfi_mem_wmask),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .rvfi_mem_wdata(rvfi_mem_wdata),
    .probe_bus_active(probe_bus_active),
    .probe_imem_addr(probe_imem_addr)
  );

  // AT MOST ONE HART ON THE BUS PER CYCLE. rtl/littledual.v joins the two harts'
  // address, write-data and strobe ports with an OR, which is only sound while
  // that holds -- and an OR of two live masters produces a plausible wrong
  // address rather than an error. So it is checked here every cycle, and it is
  // sticky: the runner turns it into an exit status of its own so a collision
  // cannot be read as a program bug.
  (* keep *) logic bus_conflict;
  initial bus_conflict = 1'b0;
  always @(posedge clk) begin
    if (&probe_bus_active) bus_conflict <= 1'b1;
  end

  // Everything below is per hart, written out twice rather than looped. A
  // generate loop would name every debug item `l_hart[0] ...`, and the runner
  // reads these by name.
  logic [15:0] errcode0, errcode1;

  monitor monitor0 (
    .clock(clk),
    .reset(hart_reset[0]),
    .rvfi_valid(rvfi_valid[0]),
    .rvfi_order(rvfi_order[63:0]),
    .rvfi_insn(rvfi_insn[31:0]),
    .rvfi_trap(rvfi_trap[0]),
    .rvfi_halt(rvfi_halt[0]),
    .rvfi_intr(rvfi_intr[0]),
    .rvfi_rs1_addr(rvfi_rs1_addr[4:0]),
    .rvfi_rs2_addr(rvfi_rs2_addr[4:0]),
    .rvfi_rs1_rdata(rvfi_rs1_rdata[31:0]),
    .rvfi_rs2_rdata(rvfi_rs2_rdata[31:0]),
    .rvfi_rd_addr(rvfi_rd_addr[4:0]),
    .rvfi_rd_wdata(rvfi_rd_wdata[31:0]),
    .rvfi_pc_rdata(rvfi_pc_rdata[31:0]),
    .rvfi_pc_wdata(rvfi_pc_wdata[31:0]),
    .rvfi_mem_addr(rvfi_mem_addr[31:0]),
    .rvfi_mem_rmask(rvfi_mem_rmask[3:0]),
    .rvfi_mem_wmask(rvfi_mem_wmask[3:0]),
    .rvfi_mem_rdata(rvfi_mem_rdata[31:0]),
    .rvfi_mem_wdata(rvfi_mem_wdata[31:0]),
    .errcode(errcode0)
  );

  monitor monitor1 (
    .clock(clk),
    .reset(hart_reset[1]),
    .rvfi_valid(rvfi_valid[1]),
    .rvfi_order(rvfi_order[127:64]),
    .rvfi_insn(rvfi_insn[63:32]),
    .rvfi_trap(rvfi_trap[1]),
    .rvfi_halt(rvfi_halt[1]),
    .rvfi_intr(rvfi_intr[1]),
    .rvfi_rs1_addr(rvfi_rs1_addr[9:5]),
    .rvfi_rs2_addr(rvfi_rs2_addr[9:5]),
    .rvfi_rs1_rdata(rvfi_rs1_rdata[63:32]),
    .rvfi_rs2_rdata(rvfi_rs2_rdata[63:32]),
    .rvfi_rd_addr(rvfi_rd_addr[9:5]),
    .rvfi_rd_wdata(rvfi_rd_wdata[63:32]),
    .rvfi_pc_rdata(rvfi_pc_rdata[63:32]),
    .rvfi_pc_wdata(rvfi_pc_wdata[63:32]),
    .rvfi_mem_addr(rvfi_mem_addr[63:32]),
    .rvfi_mem_rmask(rvfi_mem_rmask[7:4]),
    .rvfi_mem_wmask(rvfi_mem_wmask[7:4]),
    .rvfi_mem_rdata(rvfi_mem_rdata[63:32]),
    .rvfi_mem_wdata(rvfi_mem_wdata[63:32]),
    .errcode(errcode1)
  );

  // A second `monitor_isa_spec` per hart rather than reaching into the monitor,
  // for the two reasons test/testbench.v gives: the monitor is regenerated, and
  // yosys answers a hierarchical reference with a new undriven wire. Keep each
  // one wired exactly like the monitor beside it.
  logic       spec_valid0, spec_valid1;
  logic       spec_trap0, spec_trap1;
  logic [4:0] spec_rs1_addr0, spec_rs2_addr0, spec_rd_addr0;
  logic [4:0] spec_rs1_addr1, spec_rs2_addr1, spec_rd_addr1;
  logic [31:0] spec_rd_wdata0, spec_pc_wdata0, spec_mem_addr0, spec_mem_wdata0;
  logic [31:0] spec_rd_wdata1, spec_pc_wdata1, spec_mem_addr1, spec_mem_wdata1;
  logic [3:0]  spec_mem_rmask0, spec_mem_wmask0, spec_mem_rmask1, spec_mem_wmask1;

  monitor_isa_spec spec_probe0 (
    .rvfi_valid(rvfi_valid[0]),
    .rvfi_insn(rvfi_insn[31:0]),
    .rvfi_pc_rdata(rvfi_pc_rdata[31:0]),
    .rvfi_rs1_rdata(rvfi_rs1_rdata[31:0]),
    .rvfi_rs2_rdata(rvfi_rs2_rdata[31:0]),
    .rvfi_mem_rdata(rvfi_mem_rdata[31:0]),
    .spec_valid(spec_valid0),
    .spec_trap(spec_trap0),
    .spec_rs1_addr(spec_rs1_addr0),
    .spec_rs2_addr(spec_rs2_addr0),
    .spec_rd_addr(spec_rd_addr0),
    .spec_rd_wdata(spec_rd_wdata0),
    .spec_pc_wdata(spec_pc_wdata0),
    .spec_mem_addr(spec_mem_addr0),
    .spec_mem_rmask(spec_mem_rmask0),
    .spec_mem_wmask(spec_mem_wmask0),
    .spec_mem_wdata(spec_mem_wdata0)
  );

  monitor_isa_spec spec_probe1 (
    .rvfi_valid(rvfi_valid[1]),
    .rvfi_insn(rvfi_insn[63:32]),
    .rvfi_pc_rdata(rvfi_pc_rdata[63:32]),
    .rvfi_rs1_rdata(rvfi_rs1_rdata[63:32]),
    .rvfi_rs2_rdata(rvfi_rs2_rdata[63:32]),
    .rvfi_mem_rdata(rvfi_mem_rdata[63:32]),
    .spec_valid(spec_valid1),
    .spec_trap(spec_trap1),
    .spec_rs1_addr(spec_rs1_addr1),
    .spec_rs2_addr(spec_rs2_addr1),
    .spec_rd_addr(spec_rd_addr1),
    .spec_rd_wdata(spec_rd_wdata1),
    .spec_pc_wdata(spec_pc_wdata1),
    .spec_mem_addr(spec_mem_addr1),
    .spec_mem_rmask(spec_mem_rmask1),
    .spec_mem_wmask(spec_mem_wmask1),
    .spec_mem_wdata(spec_mem_wdata1)
  );

  // PER-HART SILENCE GATES. Zero retires on EITHER hart is a failed run and not
  // a curiosity: the whole claim of this harness is that it watches two harts,
  // and a run in which one of them never retired is a run that measured one.
  (* keep *) logic [31:0] rvfi_retires0, rvfi_retires1;
  (* keep *) logic [31:0] rvfi_spec_retires0, rvfi_spec_retires1;
  initial begin
    rvfi_retires0 = 32'b0;
    rvfi_retires1 = 32'b0;
    rvfi_spec_retires0 = 32'b0;
    rvfi_spec_retires1 = 32'b0;
  end
  always @(posedge clk) begin
    if (!hart_reset[0] && rvfi_valid[0]) begin
      rvfi_retires0 <= rvfi_retires0 + 32'd1;
      if (spec_valid0) rvfi_spec_retires0 <= rvfi_spec_retires0 + 32'd1;
    end
    if (!hart_reset[1] && rvfi_valid[1]) begin
      rvfi_retires1 <= rvfi_retires1 + 32'd1;
      if (spec_valid1) rvfi_spec_retires1 <= rvfi_spec_retires1 + 32'd1;
    end
  end

  // `mtvec` resets to 0 and the linker scripts put `.text` there, so a trap
  // before a handler is installed restarts that hart at `_start`. One per hart,
  // because which hart restarted is the diagnostic.
  logic trap_taken_d0, trap_taken_d1;
  (* keep *) logic trap_to_zero0, trap_to_zero1;
  initial begin
    trap_taken_d0 = 1'b0;
    trap_taken_d1 = 1'b0;
    trap_to_zero0 = 1'b0;
    trap_to_zero1 = 1'b0;
  end
  always @(posedge clk) begin
    trap_taken_d0 <= !hart_reset[0] && trap[0];
    trap_taken_d1 <= !hart_reset[1] && trap[1];
    if (trap_taken_d0 && probe_imem_addr[31:0] == 32'b0)  trap_to_zero0 <= 1'b1;
    if (trap_taken_d1 && probe_imem_addr[63:32] == 32'b0) trap_to_zero1 <= 1'b1;
  end
endmodule

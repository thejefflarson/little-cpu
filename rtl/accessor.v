`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
module accessor(
    input  logic clk,
    input  logic reset,
    // The instruction the executor is working on this cycle, one stage ahead of
    // `in`. Its bus transaction goes out now, so a synchronous memory has the
    // answer ready on the cycle the same instruction arrives here -- which is
    // what makes a load cost exactly what an add costs.
    input  decoder_output launch,
    // The executor consumes `launch` this cycle. While a divide runs decode
    // holds an instruction nothing has taken, and presenting a held one again
    // would be a second bus transaction for one store. Two writes are one write
    // for RAM and are not one for a device.
    input  logic launch_taken,
    // The same instruction a cycle later, carrying the result the executor
    // computed for it. A load's result is not in it: that comes off the bus.
    input  executor_output in,
    output logic [31:0] mem_addr,
    output logic [3:0]  mem_wstrb,
    output logic [31:0] mem_wdata,
    input  logic [31:0] mem_rdata,
    // There is no fault output here on purpose. A misaligned load or store is
    // caught in decode, which clears its `is_l*`/`is_s*` flags, so an access
    // that gets this far cannot fail. Add one and an instruction could fail
    // after decode had already let the ones behind it through, and there is no
    // way to take those back.
    //
    // The instruction memory shares one read port between fetch and data, and
    // uses this to tell a real load from an idle bus. An idle bus shows address
    // 0, which is a text address, so a memory looking only at the address would
    // give up a fetch every idle cycle and the core would never get anywhere.
    output logic mem_ren,
    output accessor_output out
);
  // Pulled out here rather than read inside the always_* blocks below: iverilog
  // cannot work out a precise sensitivity list for a field read taken inside
  // one, and warns. Use a continuous assign for any field read added later.
  logic launch_is_lb;
  logic launch_is_lbu;
  logic launch_is_lh;
  logic launch_is_lhu;
  logic launch_is_lw;
  logic launch_is_sb;
  logic launch_is_sh;
  logic launch_is_sw;
  logic [31:0] launch_mem_addr;
  logic [31:0] launch_mem_data;
  logic launch_valid;
  logic [4:0] in_rd;
  logic [31:0] in_rd_data;
  logic in_valid;

  assign launch_is_lb = launch.is_lb;
  assign launch_is_lbu = launch.is_lbu;
  assign launch_is_lh = launch.is_lh;
  assign launch_is_lhu = launch.is_lhu;
  assign launch_is_lw = launch.is_lw;
  assign launch_is_sb = launch.is_sb;
  assign launch_is_sh = launch.is_sh;
  assign launch_is_sw = launch.is_sw;
  assign launch_mem_addr = launch.mem_addr;
  assign launch_mem_data = launch.rs2;
  assign launch_valid = launch.valid;
  assign in_rd = in.rd;
  assign in_rd_data = in.rd_data;
  assign in_valid = in.valid;

  // The one gate on the whole request block. Everything below is zero when it
  // is low, which is what makes the transaction fire exactly once per memory
  // instruction however many cycles decode holds that instruction for.
  logic requesting;
  assign requesting = !reset && launch_taken && launch_valid;

  logic addr16;
  assign addr16 = launch_mem_addr[1];
  logic [1:0] addr24;
  assign addr24 = launch_mem_addr[1:0];
  logic is_load;
  assign is_load = launch_is_lw || launch_is_lh || launch_is_lhu || launch_is_lb || launch_is_lbu;
  assign mem_ren = requesting && is_load;

  // What the request needs back on the cycle its answer arrives. `in` carries
  // the same instruction then, but the address and the width were spent with
  // the request a cycle earlier and are not in it.
  logic       take_is_lb, take_is_lbu, take_is_lh, take_is_lhu, take_is_lw;
  logic [1:0] take_lane;
  logic       take_load;
  assign take_load = take_is_lb || take_is_lbu || take_is_lh || take_is_lhu || take_is_lw;

  // One shift down to the addressed byte, then one extension. Written out per
  // width and per offset the same thing is twelve 32-bit selects over four
  // distinct lanes. A halfword has `addr[0]` clear because a misaligned access
  // traps in decode, so this one shift serves both widths, and a word is the
  // lane at offset zero.
  //
  // Two continuous assigns rather than a four-way case: a constant part-select
  // read inside an `always_*` draws iverilog's `sorry:` note, and the two stages
  // are the byte shifter a LUT fabric builds from the case anyway.
  logic [31:0] load_lane1, load_lane, load_value;
  logic        load_byte, load_half, load_sign;
  assign load_lane1 = take_lane[0] ? {8'b0,  mem_rdata[31:8]}   : mem_rdata;
  assign load_lane  = take_lane[1] ? {16'b0, load_lane1[31:16]} : load_lane1;
  assign load_byte = take_is_lb || take_is_lbu;
  assign load_half = take_is_lh || take_is_lhu;
  assign load_sign = (take_is_lb && load_lane[7]) || (take_is_lh && load_lane[15]);
  assign load_value[7:0]   = load_lane[7:0];
  assign load_value[15:8]  = load_byte ? {8{load_sign}} : load_lane[15:8];
  assign load_value[31:16] = (load_byte || load_half) ? {16{load_sign}} : load_lane[31:16];

  logic [31:0] word_aligned_addr;
  logic [15:0] store_halfword;
  logic [7:0]  store_byte;
  assign word_aligned_addr = {launch_mem_addr[31:2], 2'b00};
  assign store_halfword    = launch_mem_data[15:0];
  assign store_byte        = launch_mem_data[7:0];
  // All three outputs are driven from this one block and none of them is
  // registered: the write data has to change on the same cycle the address and
  // the strobes do, or the memory writes the last cycle's data to this cycle's
  // address.
  always_comb begin
    mem_addr = 0;
    mem_wstrb = 0;
    mem_wdata = 0;
    if (requesting) begin
      mem_wdata = launch_mem_data;
      (* parallel_case *)
      case (1'b1)
        launch_is_lw || launch_is_lh || launch_is_lhu || launch_is_lb || launch_is_lbu: begin
          mem_addr = word_aligned_addr;
        end

        launch_is_sw || launch_is_sh || launch_is_sb: begin
          (* parallel_case, full_case *)
          case (1'b1)
            launch_is_sw: begin
              mem_addr = launch_mem_addr;
              mem_wstrb = 4'b1111;
              mem_wdata = launch_mem_data;
            end

            launch_is_sh: begin
              mem_wstrb = addr16 ? 4'b1100 : 4'b0011;
              mem_wdata = {2{store_halfword}};
            end

            launch_is_sb: begin
              mem_wstrb = 4'b0001 << addr24;
              mem_wdata = {4{store_byte}};
            end
          endcase // case (1'b1)
          mem_addr = word_aligned_addr;
        end // case: launch_is_sw || launch_is_sh || launch_is_sb

      endcase // case (1'b1)
    end // if (requesting)
  end // always_comb

 `ifdef RISCV_FORMAL
  // The request as it really went out, held for the one cycle until the
  // instruction it belongs to reaches `out`. This is the only stage that ever
  // knows the real address, masks and write data, and now it knows them a cycle
  // before it reports them.
  logic [31:0] take_rvfi_mem_addr, take_rvfi_mem_wdata;
  logic [3:0]  take_rvfi_mem_wstrb;
 `endif

  always_ff @(posedge clk) begin
    if (reset) begin
      take_is_lb <= 0; take_is_lbu <= 0; take_is_lh <= 0; take_is_lhu <= 0; take_is_lw <= 0;
      take_lane <= 0;
     `ifdef RISCV_FORMAL
      take_rvfi_mem_addr <= 0;
      take_rvfi_mem_wdata <= 0;
      take_rvfi_mem_wstrb <= 0;
     `endif
    end else begin
      take_is_lb <= requesting && launch_is_lb;
      take_is_lbu <= requesting && launch_is_lbu;
      take_is_lh <= requesting && launch_is_lh;
      take_is_lhu <= requesting && launch_is_lhu;
      take_is_lw <= requesting && launch_is_lw;
      take_lane <= addr24;
     `ifdef RISCV_FORMAL
      take_rvfi_mem_addr <= mem_addr;
      take_rvfi_mem_wdata <= mem_wdata;
      take_rvfi_mem_wstrb <= mem_wstrb;
     `endif
    end
  end

  always_ff @(posedge clk) begin
    if (reset || !in_valid) begin
      out <= 0;
    end else begin
      // A load and everything else settle in the same cycle: the only
      // difference is where the result comes from.
      out.valid <= 1'b1;
      out.rd <= in_rd;
      out.rd_data <= take_load ? load_value : in_rd_data;
     `ifdef RISCV_FORMAL
      out.rvfi <= in.rvfi;
      // Zero for every op that touched no memory, so a plain ALU op cannot
      // inherit a stale access from whatever last used this register. A load's
      // read mask is full whatever its width -- the monitor complains about a
      // bit that should be set and is not, never the other way round.
      out.rvfi_mem_addr <= take_rvfi_mem_addr;
      out.rvfi_mem_rmask <= take_load ? 4'b1111 : 4'b0;
      out.rvfi_mem_rdata <= take_load ? mem_rdata : 32'b0;
      out.rvfi_mem_wmask <= take_rvfi_mem_wstrb;
      out.rvfi_mem_wdata <= |take_rvfi_mem_wstrb ? take_rvfi_mem_wdata : 32'b0;
     `endif
    end
  end

 `ifdef FORMAL
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked <= 1;
  // Assumed: reset is asserted before the first clock edge. True of every
  // harness in the tree and asserted by none. Without it the `take_*` registers
  // hold no defined value at step 0.
  initial assume(reset);
  always_comb if (!clocked) assume(reset);

  logic is_store, transacting;
  assign is_store = launch_is_sw || launch_is_sh || launch_is_sb;
  assign transacting = mem_ren || |mem_wstrb;

  // The issued-once guard, in the two halves that make it one. Decode holds
  // `launch` unchanged for every cycle of a divide and the executor takes it on
  // exactly one of them, so a bus driven only on a taken cycle is a bus driven
  // once per memory instruction. Two writes are one write for RAM and are not
  // one for a device, which is why this is a correctness statement and not a
  // tidiness one. Delete `launch_taken` from `requesting` above and both go red.
  always_comb assert(transacting == (requesting && (is_load || is_store)));
  always_comb if (!launch_taken) assert(!transacting && mem_addr == 32'b0);

  // An instruction that touches no memory leaves the bus alone, so an idle
  // cycle cannot be mistaken for a load by a memory that arbitrates on address
  // alone -- address 0 is a text address.
  always_comb if (!is_load && !is_store) assert(!transacting && mem_addr == 32'b0);
 `endif
endmodule

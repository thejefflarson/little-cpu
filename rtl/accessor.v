`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
module accessor(
    input  logic clk,
    input  logic reset,
    input  executor_output in,
    output logic [31:0] mem_addr,
    output logic [3:0]  mem_wstrb,
    output logic [31:0] mem_wdata,
    input  logic [31:0] mem_rdata,
    // This stage has no fault signal to give: every trap is detected and
    // committed in decode (invariant 2), and a trapping load or store arrives
    // with every `is_l*`/`is_s*` flag clear.
    //
    // The memory registers `mem_rdata` a cycle after the address is presented, a
    // real round trip, so a load cannot be answered in the one cycle every other
    // instruction takes through this stage (ADR-0015). `stalled` is high for
    // exactly the request cycle; decode freezes and rtl/executor.v bubbles for
    // it, so the completing load and a fresh instruction cannot collide over
    // this stage's single output register.
    output logic stalled,
    // rtl/imemory.v needs this to tell a real read from the idle bus, which
    // presents address 0 -- inside the text range, so an unqualified address
    // would steal a fetch every idle cycle and the core would never run.
    output logic mem_ren,
    // A load's result is a live producer from its request until write-through,
    // one cycle later than decoder_out/executor_out alone can see (ADR-0004).
    output logic       pending_valid,
    output logic [4:0] pending_rd,
    output accessor_output out
);
  // Selected out here because iverilog cannot build a precise sensitivity entry
  // for a constant part-select taken inside an always_* block; prefer a
  // continuous assign for any field read added later (ADR-0034).
  logic in_is_lb;
  logic in_is_lbu;
  logic in_is_lh;
  logic in_is_lhu;
  logic in_is_lw;
  logic in_is_sb;
  logic in_is_sh;
  logic in_is_sw;
  logic [31:0] in_mem_addr;
  logic [31:0] in_mem_data;
  logic [4:0] in_rd;
  logic [31:0] in_rd_data;
  logic in_valid;

  assign in_is_lb = in.is_lb;
  assign in_is_lbu = in.is_lbu;
  assign in_is_lh = in.is_lh;
  assign in_is_lhu = in.is_lhu;
  assign in_is_lw = in.is_lw;
  assign in_is_sb = in.is_sb;
  assign in_is_sh = in.is_sh;
  assign in_is_sw = in.is_sw;
  assign in_mem_addr = in.mem_addr;
  assign in_mem_data = in.mem_data;
  assign in_rd = in.rd;
  assign in_rd_data = in.rd_data;
  assign in_valid = in.valid;

  logic addr16;
  assign addr16 = in_mem_addr[1];
  logic [1:0] addr24;
  assign addr24 = in_mem_addr[1:0];
  logic is_load;
  assign is_load = in_is_lw || in_is_lh || in_is_lhu || in_is_lb || in_is_lbu;
  assign stalled = in_valid && is_load;
  assign mem_ren = !reset && in_valid && is_load;
  logic is_store;
  assign is_store = in_is_sw || in_is_sh || in_is_sb;

  logic       pending_is_lb, pending_is_lbu, pending_is_lh, pending_is_lhu, pending_is_lw;
  logic       pending_addr16;
  logic [1:0] pending_addr24;
 `ifdef RISCV_FORMAL
  // These survive the request/response gap for the same reason `pending_rd`
  // does: `in` is a guaranteed bubble on the response cycle, so nothing of this
  // instruction is left to read off it then.
  rvfi_shadow  pending_rvfi;
  logic [31:0] pending_rvfi_mem_addr;
 `endif
  logic [31:0] write_request;

  logic [31:0] word_aligned_addr;
  logic [15:0] store_halfword;
  logic [7:0]  store_byte;
  assign word_aligned_addr = {in_mem_addr[31:2], 2'b00};
  assign store_halfword    = in_mem_data[15:0];
  assign store_byte        = in_mem_data[7:0];
  // Must stay combinational, in lockstep with mem_addr/mem_wstrb: a registered
  // `mem_wdata` lags them by a cycle, so the memory latches the previous cycle's
  // data at the current cycle's address.
  assign mem_wdata = write_request;
  // The zeroed defaults below are what stop the accessor re-issuing whatever
  // request was last in `in` for every cycle the executor sits busy in `divide`.
  always_comb begin
    mem_addr = 0;
    mem_wstrb = 0;
    write_request = 0;
    if (!reset && in_valid) begin
      write_request = in_mem_data;
      (* parallel_case *)
      case (1'b1)
        in_is_lw || in_is_lh || in_is_lhu || in_is_lb || in_is_lbu: begin
          mem_addr = word_aligned_addr;
        end

        in_is_sw || in_is_sh || in_is_sb: begin
          (* parallel_case, full_case *)
          case (1'b1)
            in_is_sw: begin
              mem_addr = in_mem_addr;
              mem_wstrb = 4'b1111;
              write_request = in_mem_data;
            end

            in_is_sh: begin
              mem_wstrb = addr16 ? 4'b1100 : 4'b0011;
              write_request = {2{store_halfword}};
            end

            in_is_sb: begin
              mem_wstrb = 4'b0001 << addr24;
              write_request = {4{store_byte}};
            end
          endcase // case (1'b1)
          mem_addr = word_aligned_addr;
        end // case: in_is_sw || in_is_sh || in_is_sb

      endcase // case (1'b1)
    end // if (!reset && in_valid)
  end // always_comb

  always_ff @(posedge clk) begin
    if (reset) begin
      out <= 0;
      pending_valid <= 1'b0;
      pending_rd <= 0;
      pending_is_lb <= 0; pending_is_lbu <= 0; pending_is_lh <= 0; pending_is_lhu <= 0; pending_is_lw <= 0;
      pending_addr16 <= 0;
      pending_addr24 <= 0;
     `ifdef RISCV_FORMAL
      pending_rvfi <= '0;
      pending_rvfi_mem_addr <= 0;
     `endif
    end else if (pending_valid) begin
      // The request fired last cycle and mem_rdata now reflects it. `in` is
      // guaranteed to be a bubble, so nothing competes for `out`.
      out.valid <= 1'b1;
      out.rd <= pending_rd;
     `ifdef RISCV_FORMAL
      // A full rmask regardless of load width is a safe over-approximation: the
      // monitor flags a missing bit against the spec, never an extra one.
      out.rvfi <= pending_rvfi;
      out.rvfi_mem_addr <= pending_rvfi_mem_addr;
      out.rvfi_mem_rmask <= 4'b1111;
      out.rvfi_mem_wmask <= 4'b0;
      out.rvfi_mem_rdata <= mem_rdata;
      out.rvfi_mem_wdata <= 32'b0;
     `endif
      (* parallel_case, full_case *)
      case (1'b1)
        pending_is_lb: begin
          case (pending_addr24)
            2'b00: out.rd_data <= {{24{mem_rdata[7]}}, mem_rdata[7:0]};
            2'b01: out.rd_data <= {{24{mem_rdata[15]}}, mem_rdata[15:8]};
            2'b10: out.rd_data <= {{24{mem_rdata[23]}}, mem_rdata[23:16]};
            2'b11: out.rd_data <= {{24{mem_rdata[31]}}, mem_rdata[31:24]};
          endcase
        end

        pending_is_lbu: begin
          case (pending_addr24)
            2'b00: out.rd_data <= {24'b0, mem_rdata[7:0]};
            2'b01: out.rd_data <= {24'b0, mem_rdata[15:8]};
            2'b10: out.rd_data <= {24'b0, mem_rdata[23:16]};
            2'b11: out.rd_data <= {24'b0, mem_rdata[31:24]};
          endcase
        end

        pending_is_lh: begin
          case (pending_addr16)
            1'b0: out.rd_data <= {{16{mem_rdata[15]}}, mem_rdata[15:0]};
            1'b1: out.rd_data <= {{16{mem_rdata[31]}}, mem_rdata[31:16]};
          endcase
        end

        pending_is_lhu: begin
          case (pending_addr16)
            1'b0: out.rd_data <= {16'b0, mem_rdata[15:0]};
            1'b1: out.rd_data <= {16'b0, mem_rdata[31:16]};
          endcase
        end

        pending_is_lw: out.rd_data <= mem_rdata;
      endcase
      pending_valid <= 1'b0;
    end else if (!in_valid) begin
      out <= 0;
    end else if (is_load) begin
      // Bubble now, and remember what is needed to decode mem_rdata next cycle.
      out <= 0;
      pending_valid <= 1'b1;
      pending_rd <= in_rd;
      pending_is_lb <= in_is_lb;
      pending_is_lbu <= in_is_lbu;
      pending_is_lh <= in_is_lh;
      pending_is_lhu <= in_is_lhu;
      pending_is_lw <= in_is_lw;
      pending_addr16 <= addr16;
      pending_addr24 <= addr24;
     `ifdef RISCV_FORMAL
      // `mem_addr` is this cycle's real bus address, already word-aligned.
      pending_rvfi <= in.rvfi;
      pending_rvfi_mem_addr <= mem_addr;
     `endif
    end else begin
      // Stores and every other op settle in one cycle.
      out.valid <= 1'b1;
      out.rd_data <= in_rd_data;
      out.rd <= in_rd;
     `ifdef RISCV_FORMAL
      out.rvfi <= in.rvfi;
      // Zeroed for every non-store op, so a plain ALU op cannot inherit a stale
      // memory access from whatever last used this register.
      out.rvfi_mem_addr <= is_store ? mem_addr : 32'b0;
      out.rvfi_mem_wmask <= is_store ? mem_wstrb : 4'b0;
      out.rvfi_mem_wdata <= is_store ? write_request : 32'b0;
      out.rvfi_mem_rmask <= 4'b0;
      out.rvfi_mem_rdata <= 32'b0;
     `endif
    end
  end

endmodule

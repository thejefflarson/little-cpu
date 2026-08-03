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
    // There is no fault output here on purpose. A misaligned load or store is
    // caught in decode, which clears its `is_l*`/`is_s*` flags, so an access
    // that gets this far cannot fail. Add one and an instruction could fail
    // after decode had already let the ones behind it through, and there is no
    // way to take those back.
    //
    // The memory registers `mem_rdata` a cycle after it is given an address, so
    // a load takes two cycles here where everything else takes one. `stalled` is
    // high for the first of them. Decode holds and the executor sends nothing,
    // so the load finishing next cycle and a new instruction cannot both want
    // `out` at once.
    output logic stalled,
    // The instruction memory shares one read port between fetch and data, and
    // uses this to tell a real load from an idle bus. An idle bus shows address
    // 0, which is a text address, so a memory looking only at the address would
    // give up a fetch every idle cycle and the core would never get anywhere.
    output logic mem_ren,
    // A load has left `executor_out` and has not yet written the register file
    // while it waits for memory. Decode watches these so it can wait on the load
    // during that cycle, which none of its other checks cover.
    output logic       pending_valid,
    output logic [4:0] pending_rd,
    output accessor_output out
);
  // Pulled out here rather than read inside the always_* blocks below: iverilog
  // cannot work out a precise sensitivity list for a field read taken inside
  // one, and warns. Use a continuous assign for any field read added later.
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
  // Saved for the same reason `pending_rd` is. On the cycle the data comes back,
  // `in` is empty, so nothing about this instruction can still be read from it.
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
  // Keep this off a register. It has to change on the same cycle mem_addr and
  // mem_wstrb do. Registered, it arrives a cycle late, and the memory writes the
  // last cycle's data to this cycle's address.
  assign mem_wdata = write_request;
  // The zeros below are what stop this block sending the same request again on
  // every cycle the executor spends dividing.
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
      // The read went out last cycle and mem_rdata now holds its answer. `in` is
      // empty this cycle, so nothing else wants `out`.
      out.valid <= 1'b1;
      out.rd <= pending_rd;
     `ifdef RISCV_FORMAL
      // Always a full mask, whatever the load width. The monitor complains about
      // a bit that should be set and is not, never the other way round.
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
      // Send nothing on, and keep what is needed to unpack mem_rdata next cycle.
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

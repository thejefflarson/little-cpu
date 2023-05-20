`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
module accessor(
    input  logic clk,
    input  logic reset,
    // inputs
    input  executor_output in,
    // memory access
    output logic [31:0] mem_addr,
    output logic [3:0]  mem_wstrb,
    output logic [31:0] mem_wdata,
    input  logic [31:0] mem_rdata,
    // outputs
    output accessor_output out
);
  logic addr16;
  assign addr16 = in.mem_addr[1];
  logic [1:0] addr24;
  assign addr24 = in.mem_addr[1:0];
  logic [31:0] write_request;
  // make the request
  always_comb begin
    if(reset) begin
      mem_addr = 0;
      mem_wstrb = 0;
      write_request = 0;
    end else begin
      write_request = in.mem_data;
      // request is synchronous
      (* parallel_case, full_case *)
      case (1'b1)
        in.is_lw || in.is_lh || in.is_lhu || in.is_lb || in.is_lbu: begin
          mem_wstrb = 4'b0000;
          mem_addr = {in.mem_addr[31:2], 2'b00};
        end

        in.is_sw || in.is_sh || in.is_sb: begin
          (* parallel_case, full_case *)
          case (1'b1)
            in.is_sw: begin
              mem_addr = in.mem_addr;
              mem_wstrb = 4'b1111;
              write_request = in.mem_data;
            end

            in.is_sh: begin
              // Offset to the right position
              mem_wstrb = in.mem_addr[1] ? 4'b1100 : 4'b0011;
              write_request = {2{in.mem_data[15:0]}};
            end

            in.is_sb: begin
              mem_wstrb = 4'b0001 << in.mem_addr[1:0];
              write_request = {4{in.mem_data[7:0]}};
            end
          endcase // case (1'b1)
          mem_addr = {in.mem_addr[31:2], 2'b00};
        end // case: in.is_sw || in.is_sh || in.is_sb
      endcase // case (1'b1)
    end // else: !if(reset)
  end // always_comb

  always_ff @(posedge clk) begin
    // response is registered
    if (reset) begin
      out <= 0;
      mem_wdata <= 0;
    end else begin
      mem_wdata <= write_request;
      out.rd_data <= in.rd_data;
      out.rd <= in.rd;
      (* parallel_case, full_case *)
      case (1'b1)
        // unpack the alignment from above
        in.is_lb: begin
          case (addr24)
            2'b00: out.rd_data <= {{24{mem_rdata[7]}}, mem_rdata[7:0]};
            2'b01: out.rd_data <= {{24{mem_rdata[15]}}, mem_rdata[15:8]};
            2'b10: out.rd_data <= {{24{mem_rdata[23]}}, mem_rdata[23:16]};
            2'b11: out.rd_data <= {{24{mem_rdata[31]}}, mem_rdata[31:24]};
          endcase
        end

        in.is_lbu: begin
          case (addr24)
            2'b00: out.rd_data <= {24'b0, mem_rdata[7:0]};
            2'b01: out.rd_data <= {24'b0, mem_rdata[15:8]};
            2'b10: out.rd_data <= {24'b0, mem_rdata[23:16]};
            2'b11: out.rd_data <= {24'b0, mem_rdata[31:24]};
          endcase
        end

        in.is_lh: begin
          case (addr16)
            1'b0: out.rd_data <= {{16{mem_rdata[15]}}, mem_rdata[15:0]};
            1'b1: out.rd_data <= {{16{mem_rdata[31]}}, mem_rdata[31:16]};
          endcase
        end

        in.is_lhu: begin
          case (addr16)
            1'b0: out.rd_data <= {16'b0, mem_rdata[15:0]};
            1'b1: out.rd_data <= {16'b0, mem_rdata[31:16]};
          endcase
        end

        in.is_lw: out.rd_data <= mem_rdata;
      endcase
    end // else: !if(reset)
  end

 `ifdef FORMAL
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked <= 1;
  // assume we've reset at clk 0
  initial assume(reset);
  always_comb if(!clocked) assume(reset);
  // if we've been valid but stalled, we're not valid anymore
  always_ff @(posedge clk) if(clocked && $past(accessor_valid) && $past(!writeback_ready)) assert(!accessor_valid);

  // if we're stalled we aren't requesting anytthing, and we're not publishing anything
    always_ff @(posedge clk) if(clocked && $past(stalled)) assert(!accessor_valid);
  always_ff @(posedge clk) if(clocked && !$past(reset) && $past(stalled)) assert(!accessor_ready);
 `endif
endmodule

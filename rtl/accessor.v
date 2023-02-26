`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
module accessor(
    input  logic clk,
    input  logic reset,
    // handshake
    input  logic executor_valid,
    output logic accessor_ready,
    output logic accessor_valid,
    input  logic writeback_ready,
    // inputs
    input  executor_output in,
    // memory access
    output logic mem_ready,
    input  logic mem_valid,
    output logic [31:0] mem_addr,
    output logic [3:0]  mem_wstrb,
    output logic [31:0] mem_wdata,
    input  logic [31:0] mem_rdata,
    // outputs
    output accessor_output out
);
  logic stalled;
  always_comb
    stalled = state != init;

  // handshake
  handshake handshake(
    .clk(clk),
    .reset(reset),
    .unit_ready(accessor_ready),
    .input_valid(executor_valid),
    .output_ready(writeback_ready),
    .unit_valid(accessor_valid),
    .busy(stalled)
  );
  logic addr16;
  logic [1:0] state, addr24;
  localparam init = 2'b00;
  localparam load = 2'b01;
  localparam store = 2'b10;
  // make the request
  always_ff @(posedge clk) begin
    if(reset) begin
      mem_ready <= 0;
      out <= 0;
      mem_addr <= 0;
      mem_wstrb <= 0;
      mem_wdata <= 0;
      state <= init;
    end else if(executor_valid || stalled) begin
      out.rd_data <= in.rd_data;
      out.rd <= in.rd;
      (* parallel_case, full_case *)
      case(state)
        init: begin
          (* parallel_case, full_case *)
          case (1'b1)
            in.is_lw || in.is_lh || in.is_lhu || in.is_lb || in.is_lbu: begin
              addr24 <= in.mem_addr[1:0];
              addr16 <= in.mem_addr[1];
              out.rd <= in.rd;
              mem_wstrb <= 4'b0000;
              mem_addr <= {in.mem_addr[31:2], 2'b00};
              mem_ready <= 1; // kick off a memory request
              state <= load;
            end

            in.is_sw || in.is_sh || in.is_sb: begin
              (* parallel_case, full_case *)
              case (1'b1)
                in.is_sw: begin
                  mem_addr <= in.mem_addr;
                  mem_wstrb <= 4'b1111;
                  mem_wdata <= in.mem_data;
                end

                in.is_sh: begin
                  // Offset to the right position
                  mem_wstrb <= in.mem_addr[1] ? 4'b1100 : 4'b0011;
                  mem_wdata <= {2{in.mem_data[15:0]}};
                end

                in.is_sb: begin
                  mem_wstrb <= 4'b0001 << in.mem_addr[1:0];
                  mem_wdata <= {4{in.mem_data[7:0]}};
                end
              endcase // case (1'b1)
              mem_addr <= {in.mem_addr[31:2], 2'b00};
              mem_ready <= 1; // kick off a memory request
              state <= store;
            end // case: in.is_sw || in.is_sh || in.is_sb
          endcase // case (1'b1)
        end // case: init
        load: begin
           if (mem_valid) begin
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
            state <= init;
            mem_ready <= 0;
          end
        end
        store: begin
          if (mem_valid) begin
            state <= init;
            mem_ready <= 0;
          end
        end
        default: ;
      endcase
    end else begin
      mem_ready <= 0;
    end
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

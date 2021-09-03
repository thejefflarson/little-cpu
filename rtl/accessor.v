`default_nettype none
module accessor(
    input  var logic clk,
    input  var logic reset,
    // handshake
    input  var logic executor_valid,
    input  var logic fetcher_ready,
    output var logic accessor_ready,
    output var logic accessor_valid,
    // forwards
    input executor_forward forward,
    // inputs
    input executor_output in,
    // memory access
    input  var logic mem_instr,
    output var logic mem_ready,
    input  var logic mem_valid,
    output var logic [3:0] mem_wstrb,
    output var logic [31:0] mem_wdata,
    // outputs
    output accessor_output out
);
  logic stalled;
  assign stalled = (mem_ready && !mem_valid) || !fetcher_ready;
    // handshake
  always_ff @(posedge clk) begin
    if (reset) begin
      accessor_valid <= 0;
    end else if (executor_valid && !accessor_valid && !stalled) begin
      accessor_valid <= executor_valid;
    end else if (!accessor_ready) begin
      accessor_valid <= 0;
    end
  end

  // request something from the executor
  always_ff @(posedge clk) begin
    if (reset) begin
      accessor_ready <= 0;
    end else if(!executor_valid && !accessor_valid && !stalled) begin
      accessor_ready <= 1;
    end else begin
      accessor_ready <= 0;
    end
  end

  // make the request
  always_ff @(posedge clk) begin
    if(reset) begin
      mem_ready <= 0;
    end else if(!mem_valid && !accessor_valid && !stalled) begin
      out.rd_data <= in.rd_data;
      out.rd <= in.rd;
      mem_ready <= 1;
      (* parallel_case, full_case *)
      case (1'b1)
        forward.is_lw || forward.is_lh || forward.is_lhu || forward.is_lb || forward.is_lbu: begin
          addr24 <= load_store_address[1:0];
          addr16 <= load_store_address[1];
          addr8 <= load_store_address[0];
          if ((is_lw && |load_store_address[1:0]) ||
              ((is_lh || is_lhu) && load_store_address[0])) begin
            cpu_state <= cpu_trap;
          end else begin
            rd_addr <= rd;
            mem_wstrb <= 4'b0000;
            mem_addr <= {load_store_address[31:2], 2'b00};
            mem_valid <= 1; // kick off a memory request
            cpu_state <= finish_load;
          end
        end

        forward.is_sw || forward.is_sh || forward.is_sb: begin
          (* parallel_case, full_case *)
          case (1'b1)
            is_sw: begin
              mem_addr <= load_store_address;
              mem_wstrb <= 4'b1111;
              mem_wdata <= regs[rs2];
            end

            is_sh: begin
              // Offset to the right position
              mem_wstrb <= load_store_address[1] ? 4'b1100 : 4'b0011;
              mem_wdata <= {2{regs[rs2][15:0]}};
            end

            is_sb: begin
              mem_wstrb <= 4'b0001 << load_store_address[1:0];
              mem_wdata <= {4{regs[rs2][7:0]}};
            end
          endcase // case (1'b1)
          mem_addr <= {load_store_address[31:2], 2'b00};
          mem_instr <= 0;
          mem_valid <= 1; // kick off a memory request
          cpu_state <= finish_store;
        end // case: is_sw || is_sh || is_sb
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
  always_ff @(posedge clk) if(clocked && $past(accessor_valid) && stalled) assert(!accessor_valid);

  // if we're stalled we aren't requesting anytthing, and we're not publishing anything
  always_ff @(posedge clk) if(clocked && $past(stalled)) assert(!accessor_valid && !accessor_ready);
 `endif
endmodule

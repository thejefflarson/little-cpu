`timescale 1 ns / 1 ps
`default_nettype none
module fetcher(
  input  logic clk,
  input  logic reset,
  // handshake
  input  logic decoder_ready,
  output logic fetcher_valid,
  // inputs
  input  logic [31:0] pc,
  output logic [31:0] imem_addr,
  input  logic [31:0] imem_data,
  // outputs
  output fetcher_output out
);

  assign imem_addr = pc;
  always_ff @(posedge clk) begin
    if(reset) begin
      fetcher_valid <= 0;
    end else if(!fetcher_valid && decoder_ready) begin
      out.instr <= imem_data;
      out.pc <= imem_addr;
      fetcher_valid <= 1;
    end else begin
      fetcher_valid <= 0;
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
  always_ff @(posedge clk) if(clocked && $past(fetcher_valid) && $past(fetcher_valid && !fetcher_ready)) assert(!fetcher_valid);

  // if we've been valid but the next stage is busy, we're not valid anymore
  always_ff @(posedge clk) if(clocked && $past(fetcher_valid) && $past(!fetcher_ready)) assert(!fetcher_valid);
 `endif
endmodule

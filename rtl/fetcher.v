`default_nettype none
module fetcher(
  input  var logic clk,
  input  var logic reset,
  // handshake
  input  var logic fetcher_ready,
  output var logic fetcher_valid,
  // inputs
  input  var logic [31:0] pc,
  input  var logic [31:0] mem_rdata,
  output var logic [31:0] imem_addr,
  input  var logic [31:0] imem_data,
  // outputs
  output fetcher_output out,
);
  // handshake
  always_ff @(posedge clk) begin
    if (reset) begin
      fetcher_valid <= 0;
    // we've received a request and we can pass it along
    end else if (!fetcher_valid && fetcher_ready) begin
      fetcher_valid <= 1;
    end else if (!fetcher_ready) begin
      fetcher_valid <= 0;
    end
  end // always_ff @ (posedge clk)

  // make the request whenever we're not valid, and the memory isn't busy
  always_ff @(posedge clk) begin
    if (reset) begin
      imem_addr <= 0;
    end else if(!fetcher_valid) begin
      imem_addr <= pc;
      out.pc <= pc;
    end
  end // always_ff @ (posedge clk)

  // we have something from memory
  always_ff @(posedge clk) begin
    if (reset) begin
      out.instr <= 0;
    end else if (fetcher_ready) begin
      out.instr <= imem_data;
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

`default_nettype none
module fetcher(
  input  var logic        clk,
  input  var logic        reset,
  // handshake
  input  var logic        decoder_ready,
  output var logic        fetcher_valid,
  // inputs
  input  var logic [31:0] pc,
  input  var logic        mem_valid,
  input  var logic [31:0] mem_rdata,
  // outputs
  output var logic [31:0] instr,
  output var logic [31:0] fetcher_pc,
  output var logic        mem_ready,
  output var logic        mem_instr,
  output var logic [31:0] mem_addr,
  output var logic [3:0]  mem_wstrb
);
  // handshake
  always_ff @(posedge clk) begin
    if (reset) begin
      fetcher_valid <= 0;
    // we've received a request and we can pass it along
    end else if (mem_valid && !fetcher_valid && decoder_ready) begin
      fetcher_valid <= mem_valid;
    end else if (!decoder_ready) begin
      fetcher_valid <= 0;
    end
  end // always_ff @ (posedge clk)

  // make the request whenever we're not valid, and the memory isn't busy
  always_ff @(posedge clk) begin
    if (reset) begin
      mem_ready <= 0;
      mem_instr <= 0;
      mem_addr <= 0;
      mem_wstrb <= 3'b000;
    end else if(!mem_valid && !fetcher_valid) begin
      mem_ready <= 1;
      mem_addr <= pc;
      mem_wstrb <= 3'b000;
      mem_instr <= 1;
      fetcher_pc <= pc;
    end else begin
      mem_ready <= 0;
    end
  end // always_ff @ (posedge clk)

  // we have something from memory
  always_ff @(posedge clk) begin
    if (reset) begin
      instr <= 0;
    end else if (mem_valid) begin
      instr <= mem_rdata;
    end
  end

 `ifdef FORMAL
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked = 1;
  // assume we've reset at clk 0
  initial assume(reset);
  always @(*) if(!clocked) assume(reset);

  // mem_ready and mem_valid happen in tandem
  logic past_valid = 0;
  always_ff @(posedge clk) begin
    past_valid <= mem_valid;
    if (past_valid && mem_ready) assume(mem_valid);
  end

  // no writing! always get an instruction!
  always_ff @(posedge clk) begin
    if(clocked && mem_ready) begin
      assert(mem_wstrb == 4'b0000);
      assert(mem_instr == 1);
    end
  end

  // if we've been valid but stalled, we're not valid anymore
  always_ff @(posedge clk) if(clocked && $past(fetcher_valid) && $past(fetcher_valid && !decoder_ready)) assert(!fetcher_valid);

  // if we've been valid but the next stage is busy, we're not valid anymore
  always_ff @(posedge clk) if(clocked && $past(fetcher_valid) && $past(!decoder_ready)) assert(!fetcher_valid);
 `endif
endmodule

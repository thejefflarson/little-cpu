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
  logic stalled, waiting, en;
  assign stalled = fetcher_valid && !decoder_ready;
  assign waiting = !fetcher_valid && decoder_ready;

  always_ff @(posedge clk) begin
    if (reset) begin
      fetcher_valid <= 0;
    // we've recieved a request and we can pass it along
    end else if (mem_valid && !stalled) begin
      fetcher_valid <= mem_valid;
    end else if (!decoder_ready) begin
      fetcher_valid <= 0;
    end
  end // always_ff @ (posedge clk)

  // make the request when we the decoder is ready
  always_ff @(posedge clk) begin
    if (reset) begin
      mem_ready <= 0;
      mem_instr <= 0;
      mem_addr <= 0;
      mem_wstrb <= 4'b0000;
    end else if(!mem_valid && waiting) begin
      mem_ready <= 1;
      mem_addr <= pc;
      mem_wstrb <= 4'b0000;
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
    end else if (mem_valid && !fetcher_valid) begin
      instr <= mem_rdata;
    end
  end

 `ifdef FORMAL
  // can't be not ready and valid at the same time
  always_ff @(posedge clk) assert(!(!decoder_ready && fetcher_valid));
  // if noone has asked for anything, we shouldn't have anything
  always_ff @(posedge clk) if(!decoder_ready) assume(!fetcher_valid);
  // set up a clock
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked = 1;
  // assume we've reset at clk 0
  initial assume(reset);
  always @(*) if(!clocked) assume(reset);
  // nothing changes as long as we're valid
  always_ff @(posedge clk) begin
    if(clocked && $past(fetcher_valid) && fetcher_valid) begin
      assert($stable(fetcher_pc));
      assert($stable(instr));
    end
  end
 `endif
endmodule

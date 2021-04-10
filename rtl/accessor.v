`default_nettype none
module accessor(
    input  var logic        clk,
    input  var logic        reset,
    // handshake
    input  var logic        executor_valid,
    output var logic        accessor_ready,
    output var logic        accessor_valid,
    input  var logic        writeback_ready,
    // forwards
    input  var logic [4:0]  executor_rd,
    input  var logic [31:0] executor_rd_data,
    // inputs
    input  var logic [31:0] executor_mem_addr,
    input  var logic [31:0] executor_mem_data,
    input  var logic        executor_is_lui,
    input  var logic        executor_is_lb,
    input  var logic        executor_is_lbu,
    input  var logic        executor_is_lh,
    input  var logic        executor_is_lhu,
    input  var logic        executor_is_lw,
    input  var logic        executor_is_sb,
    input  var logic        executor_is_sh,
    input  var logic        executor_is_sw,
    // memory access
    output var logic        mem_ready,
    output var logic        mem_valid,
    output var logic [3:0]  mem_wstrb,
    output var logic [31:0] mem_wdata,
    // outputs
    output var logic [4:0]  accessor_rd,
    output var logic [31:0] accessor_rd_data
);
  logic stalled;
  assign stalled = mem_ready && !mem_valid;
    // handshake
  always_ff @(posedge clk) begin
    if (reset) begin
      accessor_valid <= 0;
    end else if (executor_valid && !accessor_valid && writeback_ready && !stalled) begin
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

    // state machine
 `ifdef FORMAL
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked = 1;
  // assume we've reset at clk 0
  initial assume(reset);
  always @(*) if(!clocked) assume(reset);
  // if we've been valid but stalled, we're not valid anymore
  always_ff @(posedge clk) if(clocked && $past(accessor_valid) && $past(accessor_valid && !writeback_ready)) assert(!accessor_valid);

  // if we've been valid but the next stage is busy, we're not valid anymore
  always_ff @(posedge clk) if(clocked && $past(accessor_valid) && $past(!writeback_ready)) assert(!accessor_valid);

  // if we're stalled we aren't requesting anytthing, and we're not publishing anything
  always_ff @(posedge clk) if(clocked && $past(stalled)) assert(!accessor_valid && !accessor_ready);
 `endif
endmodule

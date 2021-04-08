`default_nettype none
module accessor(
    input var logic         clk,
    input var logic         reset,
    // handshake
    input var logic         executor_valid,
    input var logic         accessor_ready,
    input var logic         accessor_valid,
    input var logic         writeback_ready,
    // forwards
    input var logic [4:0]   executor_rd,
    input var logic [31:0]  executor_rd_data,
    // inputs
    input var logic [31:0]  executor_mem_addr,
    input var logic [31:0]  executor_mem_data,
    input var logic         executor_is_lui,
    input var logic         executor_is_lb,
    input var logic         executor_is_lbu,
    input var logic         executor_is_lh,
    input var logic         executor_is_lhu,
    input var logic         executor_is_lw,
    input var logic         executor_is_sb,
    input var logic         executor_is_sh,
    input var logic         executor_is_sw,
    // memory access
    output var logic        mem_ready,
    output var logic        mem_valid,
    output var logic [3:0]  mem_wstrb,
    output var logic [31:0] mem_wdata,
    // outputs
    output var logic [4:0]  accessor_rd,
    output var logic [31:0] accessor_rd_data
);

endmodule

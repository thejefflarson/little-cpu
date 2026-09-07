// There is no bus handshake to model.
module rvfi_wrapper (
  input var clock, reset,
  `RVFI_OUTPUTS
);
  `RVFI_WIRES

  (* keep *) `rvformal_rand_reg [31:0] imem_data;
  // The fetch window's second word, resampled every cycle exactly like `imem_data`.
  (* keep *) `rvformal_rand_reg [31:0] imem_data2;
  (* keep *) `rvformal_rand_reg [31:0] mem_rdata;

  (* keep *) logic [31:0] imem_addr;
  (* keep *) logic [31:0] imem_addr2;
  // The fetch address one cycle early, for a synchronous memory.
  (* keep *) logic [31:0] imem_addr_next;
  (* keep *) logic [31:0] mem_addr;
  (* keep *) logic [31:0] mem_wdata;
  (* keep *) logic [3:0]  mem_wstrb;
  (* keep *) logic        mem_ren;
  (* keep *) logic        trap;

  (* keep *) logic fetch_stall;
  logic text_write;

  // This environment models no address map, so `imem_fault` is free. What they need is the assumption below, which is what a memory that answers nothing always does.
  (* keep *) `rvformal_rand_reg imem_fault;

  (* keep *) `rvformal_rand_reg mem_reservable;

  (* keep *) `rvformal_rand_reg atomic_supported;
  wire [31:0] atomic_addr;

  wire mem_lock;
  wire bus_request;

  always @* begin
    if (imem_fault) begin
      assume (imem_data  == 32'b0);
      assume (imem_data2 == 32'b0);
    end
  end

  imem_arbiter arbiter (
    .clock(clock),
    .reset(reset),
    .mem_addr(mem_addr),
    .mem_wstrb(mem_wstrb),
    .mem_ren(mem_ren),
    .fetch_stall(fetch_stall),
    .text_write(text_write)
  );

  logic [1:0] text_write_age = 0;
  always @(posedge clock)
    text_write_age <= {text_write_age[0], text_write};

  logic [31:0] past_imem_addr, past_imem_data;
  logic [31:0] past_imem_addr2, past_imem_data2;
  logic        past_imem_valid = 0;

  always @(posedge clock) begin
    past_imem_addr  <= imem_addr;
    past_imem_data  <= imem_data;
    past_imem_addr2 <= imem_addr2;
    past_imem_data2 <= imem_data2;
    past_imem_valid <= !reset;
  end

  always @* begin
    if (past_imem_valid && !reset && !fetch_stall && !text_write_age[1]) begin
      if (imem_addr  == past_imem_addr)  assume (imem_data  == past_imem_data);
      if (imem_addr2 == past_imem_addr2) assume (imem_data2 == past_imem_data2);
    end
  end

  littlecpu dut (
    .clk(clock),
    .reset(reset),
    .imem_addr(imem_addr),
    .imem_data(imem_data),
    .imem_addr2(imem_addr2),
    .imem_data2(imem_data2),
    .imem_addr_next(imem_addr_next),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_ren(mem_ren),
    .mem_rdata(mem_rdata),
    .fetch_stall(fetch_stall),
    .imem_fault(imem_fault),
    .mem_reservable(mem_reservable),
    .atomic_addr(atomic_addr),
    .atomic_supported(atomic_supported),
    .bus_wait(1'b0),
    .snoop_write(1'b0),
    .snoop_addr(32'b0),
    .mem_lock(mem_lock),
    .bus_request(bus_request),
    .irq_timer(1'b0),
    .trap(trap),
    `RVFI_CONN
  );

 `ifdef RISCV_FAIRNESS
 `endif
endmodule

`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
// Every access reads and commits in decode, on the edge its instruction issues. The
// decoder serializes CSR instructions and decides illegal-CSR traps, so nothing here
// stalls or faults.
module csrs #(
  parameter logic [31:0] HART_ID = 32'd0
) (
  input  logic clk,
  input  logic reset,

  input  logic [11:0] addr,
  input  logic        ren,
  input  logic        wen,
  input  logic [31:0] wdata,
  output logic [31:0] rdata,
  output logic        implemented,

  input  logic        instret,

  input  logic        trap_entry,
  input  logic [31:0] trap_cause,
  input  logic [31:0] trap_epc,
  input  logic [31:0] trap_tval,
  input  logic        mret_entry,
  // MUST ARRIVE REGISTERED. `interrupt_pending` is one AND away from `next_pc`, so an
  // unregistered 64-bit compare here would land in the fetch loop.
  input  logic        irq_timer,
  output logic [31:0] mtvec_value,
  output logic [31:0] mepc_value,
  output logic        interrupt_pending
 `ifdef RISCV_FORMAL
  ,
  output rvfi_csr64 rvfi_mcycle,
  output rvfi_csr64 rvfi_minstret,
  output rvfi_csr32 rvfi_mscratch
  `ifdef RISCV_FORMAL_CSR_MCAUSE
  , output rvfi_csr32 rvfi_mcause
  `endif
 `endif
);
  localparam logic [11:0] MSTATUS   = 12'h300;
  localparam logic [11:0] MISA      = 12'h301;
  localparam logic [11:0] MSTATUSH  = 12'h310;
  localparam logic [11:0] MCONFIGPTR = 12'hF15;
  localparam logic [11:0] MIE       = 12'h304;
  localparam logic [11:0] MTVEC     = 12'h305;
  localparam logic [11:0] MSCRATCH  = 12'h340;
  localparam logic [11:0] MEPC      = 12'h341;
  localparam logic [11:0] MCAUSE    = 12'h342;
  localparam logic [11:0] MTVAL     = 12'h343;
  localparam logic [11:0] MIP       = 12'h344;
  localparam logic [11:0] MCYCLE    = 12'hB00;
  localparam logic [11:0] MINSTRET  = 12'hB02;
  localparam logic [11:0] MCYCLEH   = 12'hB80;
  localparam logic [11:0] MINSTRETH = 12'hB82;
  localparam logic [11:0] MVENDORID = 12'hF11;
  localparam logic [11:0] MARCHID   = 12'hF12;
  localparam logic [11:0] MIMPID    = 12'hF13;
  localparam logic [11:0] MHARTID   = 12'hF14;

  // MXL = 1, and extensions I, M, A and C.
  localparam logic [31:0] MISA_VALUE = 32'h4000_1105;

  logic [63:0] mcycle, minstret;
  logic [31:0] mscratch, mtvec, mepc, mcause, mtval;
  logic        mstatus_mie, mstatus_mpie;
  logic        mie_mtie;

  logic [31:0] mcycle_lo, mcycle_hi, minstret_lo, minstret_hi;
  assign mcycle_lo   = mcycle[31:0];
  assign mcycle_hi   = mcycle[63:32];
  assign minstret_lo = minstret[31:0];
  assign minstret_hi = minstret[63:32];

  logic [31:0] mstatus_value;
  // MPP = 2'b11 at [12:11]; MPIE at [7]; MIE at [3]; everything else 0.
  assign mstatus_value = {19'b0, 2'b11, 3'b0, mstatus_mpie, 3'b0, mstatus_mie, 3'b0};

  // Bit 7 is MTIE in `mie` and MTIP in `mip`.
  logic [31:0] mie_value, mip_value;
  assign mie_value = {24'b0, mie_mtie, 7'b0};
  assign mip_value = {24'b0, irq_timer, 7'b0};

  assign interrupt_pending = irq_timer && mie_mtie && mstatus_mie;

  // The 87 performance-monitor addresses, every one of them read-only zero.
  localparam logic [6:0] MHPMCOUNTER_WINDOW  = 7'h58; // 0xB00-0xB1F
  localparam logic [6:0] MHPMCOUNTERH_WINDOW = 7'h5C; // 0xB80-0xB9F
  localparam logic [6:0] MHPMEVENT_WINDOW    = 7'h19; // 0x320-0x33F
  logic hpm_number, hpm_counter, hpm_event, hpm_zero;
  assign hpm_number  = addr[4:0] > 5'd2;
  assign hpm_counter = addr[11:5] == MHPMCOUNTER_WINDOW ||
                       addr[11:5] == MHPMCOUNTERH_WINDOW;
  assign hpm_event   = addr[11:5] == MHPMEVENT_WINDOW;
  assign hpm_zero    = hpm_number && (hpm_counter || hpm_event);

  assign mtvec_value = mtvec;
  assign mepc_value  = mepc;

  always_comb begin
    implemented = 1'b1;
    (* parallel_case *)
    case (addr)
      MSTATUS:   rdata = mstatus_value;
      MSTATUSH:  rdata = 32'b0;
      MISA:      rdata = MISA_VALUE;
      MIE:       rdata = mie_value;
      MTVEC:     rdata = mtvec;
      MSCRATCH:  rdata = mscratch;
      MEPC:      rdata = mepc;
      MCAUSE:    rdata = mcause;
      MTVAL:     rdata = mtval;
      MIP:       rdata = mip_value;
      MCYCLE:    rdata = mcycle_lo;
      MCYCLEH:   rdata = mcycle_hi;
      MINSTRET:  rdata = minstret_lo;
      MINSTRETH: rdata = minstret_hi;
      MHARTID:   rdata = HART_ID;
      MVENDORID, MARCHID, MIMPID, MCONFIGPTR: rdata = 32'b0;
      default: begin
        rdata = 32'b0;
        implemented = hpm_zero;
      end
    endcase
  end

  logic [31:0] wdata_mtvec, wdata_mepc, wdata_mstatus, wdata_mie;
  // Direct mode only, so a write forces the two mode bits at `mtvec[1:0]` to zero.
  assign wdata_mtvec   = {wdata[31:2], 2'b00};
  assign wdata_mepc    = {wdata[31:1], 1'b0};
  assign wdata_mstatus = {19'b0, 2'b11, 3'b0, wdata[7], 3'b0, wdata[3], 3'b0};
  assign wdata_mie     = {24'b0, wdata[7], 7'b0};

  logic [31:0] warl;
  always_comb begin
    (* parallel_case *)
    case (addr)
      MSTATUS:   warl = wdata_mstatus;
      MIE:       warl = wdata_mie;
      MTVEC:     warl = wdata_mtvec;
      MSCRATCH:  warl = wdata;
      MEPC:      warl = wdata_mepc;
      MCAUSE:    warl = wdata;
      MTVAL:     warl = wdata;
      MCYCLE, MCYCLEH, MINSTRET, MINSTRETH: warl = wdata;
      default:   warl = rdata;
    endcase
  end
  logic warl_mie, warl_mpie, warl_mtie;
  assign warl_mie  = warl[3];
  assign warl_mpie = warl[7];
  assign warl_mtie = warl[7];

  // BIT 1 MUST SURVIVE. C makes 2-byte targets legal, so masking it would resume a fault
  // on a compressed instruction two bytes early.
  logic [31:0] trap_epc_warl;
  assign trap_epc_warl = {trap_epc[31:1], 1'b0};

  logic wr_mcycle, wr_mcycleh, wr_minstret, wr_minstreth, wr_mscratch;
  assign wr_mcycle    = wen && addr == MCYCLE;
  assign wr_mcycleh   = wen && addr == MCYCLEH;
  assign wr_minstret  = wen && addr == MINSTRET;
  assign wr_minstreth = wen && addr == MINSTRETH;
  assign wr_mscratch  = wen && addr == MSCRATCH;

  logic mcycle_tick, minstret_tick;
  assign mcycle_tick   = !wr_mcycle   && !wr_mcycleh;
  assign minstret_tick = !wr_minstret && !wr_minstreth && instret;

  logic [63:0] mcycle_plus, minstret_plus;
  assign mcycle_plus   = mcycle_tick   ? mcycle   + 64'd1 : mcycle;
  assign minstret_plus = minstret_tick ? minstret + 64'd1 : minstret;

  logic [31:0] mcycle_plus_lo, mcycle_plus_hi, minstret_plus_lo, minstret_plus_hi;
  assign mcycle_plus_lo   = mcycle_plus[31:0];
  assign mcycle_plus_hi   = mcycle_plus[63:32];
  assign minstret_plus_lo = minstret_plus[31:0];
  assign minstret_plus_hi = minstret_plus[63:32];

  logic [31:0] mcycle_next_lo, mcycle_next_hi, minstret_next_lo, minstret_next_hi;
  assign mcycle_next_lo   = wr_mcycle    ? warl : mcycle_plus_lo;
  assign mcycle_next_hi   = wr_mcycleh   ? warl : mcycle_plus_hi;
  assign minstret_next_lo = wr_minstret  ? warl : minstret_plus_lo;
  assign minstret_next_hi = wr_minstreth ? warl : minstret_plus_hi;

  always_ff @(posedge clk) begin
    if (reset) begin
      mcycle       <= 64'b0;
      minstret     <= 64'b0;
      mscratch     <= 32'b0;
      mtvec        <= 32'b0;
      mepc         <= 32'b0;
      mcause       <= 32'b0;
      mtval        <= 32'b0;
      mstatus_mie  <= 1'b0;
      mstatus_mpie <= 1'b0;
      mie_mtie     <= 1'b0;
    end else begin
      mcycle   <= {mcycle_next_hi, mcycle_next_lo};
      minstret <= {minstret_next_hi, minstret_next_lo};
      if (wen) begin
        (* parallel_case *)
        case (addr)
          MSTATUS: begin
            mstatus_mie  <= warl_mie;
            mstatus_mpie <= warl_mpie;
          end
          MIE:      mie_mtie <= warl_mtie;
          MTVEC:    mtvec    <= warl;
          MSCRATCH: mscratch <= warl;
          MEPC:     mepc     <= warl;
          MCAUSE:   mcause   <= warl;
          MTVAL:    mtval    <= warl;
          default: ;
        endcase
      end
      if (trap_entry) begin
        mepc         <= trap_epc_warl;
        mcause       <= trap_cause;
        mtval        <= trap_tval;
        mstatus_mpie <= mstatus_mie;
        mstatus_mie  <= 1'b0;
      end else if (mret_entry) begin
        mstatus_mie  <= mstatus_mpie;
        mstatus_mpie <= 1'b1;
      end
    end
  end

 `ifdef RISCV_FORMAL
  logic rd_mcycle, rd_mcycleh, rd_minstret, rd_minstreth, rd_mscratch;
  assign rd_mcycle    = ren && addr == MCYCLE;
  assign rd_mcycleh   = ren && addr == MCYCLEH;
  assign rd_minstret  = ren && addr == MINSTRET;
  assign rd_minstreth = ren && addr == MINSTRETH;
  assign rd_mscratch  = ren && addr == MSCRATCH;

  logic [31:0] mcycle_next_lo_reported, mcycle_next_hi_reported;
  logic [31:0] minstret_next_lo_reported, minstret_next_hi_reported;
  assign mcycle_next_lo_reported   = wr_mcycle    ? warl : mcycle_lo;
  assign mcycle_next_hi_reported   = wr_mcycleh   ? warl : mcycle_hi;
  assign minstret_next_lo_reported = wr_minstret  ? warl : minstret_lo;
  assign minstret_next_hi_reported = wr_minstreth ? warl : minstret_hi;

  always_comb begin
    rvfi_mcycle.rmask = {{32{rd_mcycleh}}, {32{rd_mcycle}}};
    rvfi_mcycle.wmask = {{32{wr_mcycleh}}, {32{wr_mcycle}}};
    rvfi_mcycle.rdata = mcycle;
    rvfi_mcycle.wdata = {mcycle_next_hi_reported, mcycle_next_lo_reported};

    rvfi_minstret.rmask = {{32{rd_minstreth}}, {32{rd_minstret}}};
    rvfi_minstret.wmask = {{32{wr_minstreth}}, {32{wr_minstret}}};
    rvfi_minstret.rdata = minstret;
    rvfi_minstret.wdata = {minstret_next_hi_reported, minstret_next_lo_reported};

    rvfi_mscratch.rmask = {32{rd_mscratch}};
    rvfi_mscratch.wmask = {32{wr_mscratch}};
    rvfi_mscratch.rdata = mscratch;
    rvfi_mscratch.wdata = wr_mscratch ? warl : mscratch;
  end

  `ifdef RISCV_FORMAL_CSR_MCAUSE
  logic rd_mcause, wr_mcause;
  assign rd_mcause = ren && addr == MCAUSE;
  assign wr_mcause = wen && addr == MCAUSE;

  always_comb begin
    rvfi_mcause.rmask = {32{rd_mcause}};
    rvfi_mcause.wmask = {32{wr_mcause || trap_entry}};
    rvfi_mcause.rdata = mcause;
    rvfi_mcause.wdata = trap_entry ? trap_cause : (wr_mcause ? warl : mcause);
  end
  `endif
 `endif
endmodule

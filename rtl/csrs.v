`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
// The machine-mode CSR file (ADR-0005).
//
// This is a *sibling of the decoder*, not a pipeline stage. Every access is
// read and committed in decode, on the same edge the accessing instruction
// issues, so no CSR state exists anywhere downstream: there is nothing to
// forward, nothing to replay, and no "what does mcycle read with three
// instructions in flight" corner. CLAUDE.md invariant 5 (CSR instructions
// serialize) is enforced in rtl/decoder.v, not here -- this module has no
// idea a stall exists and does not need one.
//
// The CSR set is ADR-0005's, exactly:
//
//   read/write   mstatus (MIE/MPIE; MPP is WARL -> 2'b11), mtvec (direct
//                mode, base 4-byte aligned), mepc (bit 0 always clear --
//                only bit 0, because C makes 2-byte targets legal), mcause,
//                mscratch, mcycle/mcycleh, minstret/minstreth
//   read-only    mtval = 0, mie = 0, mip = 0 (no interrupt sources),
//                misa = 0x4000_1104 (RV32IMC), mvendorid/marchid/mimpid/
//                mhartid = 0
//
// Trap entry and `mret` are the second write port, and the only architectural
// CSR update no CSR *instruction* performs. They arrive from the decoder on
// the edge the trapping instruction (or the `mret`) issues, because that is
// where every trap is detected and committed (CLAUDE.md invariant 2,
// ADR-0005). `mtvec_value`/`mepc_value` go back out to the decoder, which owns
// the PC (invariant 1) and therefore has to be the thing that redirects it.
//
// A trap does NOT raise a trap here: this module never decides that an access
// is illegal. `implemented` feeds the decoder's `instr_valid` term and the
// read-only test is on the ADDRESS (addr[11:10] == 2'b11), so both illegal-CSR
// rules are decided in rtl/decoder.v alongside every other trap cause -- one
// priority encoder, one commit point, one ADR (ADR-0030) covering all five.
module csrs(
  input  logic clk,
  input  logic reset,

  // ---- the decode-stage access port -------------------------------------
  // `addr` is instr[31:20], presented combinationally every cycle; `rdata`
  // and `implemented` answer it the same cycle, because the decoder needs
  // the read value in time to register it into decoder_out at this edge.
  //
  // `ren`/`wen` describe the instruction the decoder is *issuing* this edge.
  // Zicsr's suppression rules (CSRRS/CSRRC with a zero source suppress the
  // write; CSRRW with rd == x0 suppresses the read) are resolved in the
  // decoder, where the operand fields live, and arrive here already applied
  // -- so `wen` means "commit this", full stop.
  input  logic [11:0] addr,
  input  logic        ren,
  input  logic        wen,
  input  logic [31:0] wdata,
  output logic [31:0] rdata,
  output logic        implemented,

  // ADR-0027: minstret counts non-trapping *issues*. High for exactly the
  // cycles the decoder issues one.
  input  logic        instret,

  // ---- trap entry and mret (ADR-0005 / ADR-0028) -------------------------
  // `trap_entry` is high for exactly the cycle the decoder commits a trap;
  // `mret_entry` for exactly the cycle it commits an `mret`. They cannot
  // coincide with each other (a trapping instruction does not also execute)
  // nor with `wen` (the decoder gates `csr_wen` on a NON-trapping commit, and
  // `mret` is not a CSR access), so the three write paths below need no
  // priority between them -- but the code says so explicitly rather than
  // relying on last-assignment-wins.
  input  logic        trap_entry,
  input  logic [31:0] trap_cause,
  input  logic [31:0] trap_epc,
  input  logic        mret_entry,
  // Read back by rtl/decoder.v, combinationally, to redirect the PC.
  output logic [31:0] mtvec_value,
  output logic [31:0] mepc_value
 `ifdef RISCV_FORMAL
  ,
  // ADR-0006 shadow payloads, for exactly the CSRs formal/checks.cfg's
  // `[csrs]` list names. Combinational off this cycle's access; the decoder
  // latches them into the issuing instruction's shadow so they ride to
  // writeback on the ordinary valid-bit protocol.
  output rvfi_csr64 rvfi_mcycle,
  output rvfi_csr64 rvfi_minstret,
  output rvfi_csr32 rvfi_mscratch
 `endif
);
  // CSR addresses (RISC-V privileged spec, Ch. 2). Named rather than spelled
  // as hex at each use site for the same reason the decoder names its
  // register-index fields.
  localparam logic [11:0] MSTATUS   = 12'h300;
  localparam logic [11:0] MISA      = 12'h301;
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

  // RV32 I M C, MXL = 1. CLAUDE.md's ISA target; the C bit stopped being an
  // untested claim when ADR-0003's fetch window landed.
  localparam logic [31:0] MISA_VALUE = 32'h4000_1104;

  // ---- state ------------------------------------------------------------
  logic [63:0] mcycle, minstret;
  logic [31:0] mscratch, mtvec, mepc, mcause;
  // mstatus is three fields, not a register: MPP is hardwired to M-mode
  // (WARL, and this core has no other mode), and every other bit is 0.
  logic        mstatus_mie, mstatus_mpie;

  // Halves of the two counters, selected out here rather than inside the
  // always_* blocks below. A constant part-select taken inside an always_*
  // block defeats iverilog's sensitivity analysis and draws a `sorry:` note;
  // CLAUDE.md says warnings are errors and names rtl/executor.v as the one
  // documented exception, so this file uses named continuous assigns
  // throughout.
  logic [31:0] mcycle_lo, mcycle_hi, minstret_lo, minstret_hi;
  assign mcycle_lo   = mcycle[31:0];
  assign mcycle_hi   = mcycle[63:32];
  assign minstret_lo = minstret[31:0];
  assign minstret_hi = minstret[63:32];

  logic [31:0] mstatus_value;
  // MPP = 2'b11 at [12:11]; MPIE at [7]; MIE at [3]; everything else 0.
  assign mstatus_value = {19'b0, 2'b11, 3'b0, mstatus_mpie, 3'b0, mstatus_mie, 3'b0};

  // The two registers the decoder redirects the PC through. Straight
  // continuous assigns of the register outputs, i.e. the values as of the
  // START of the issuing cycle. That is the right phase and not an accident:
  // decode issues at most one instruction per cycle, so a trapping
  // instruction and a `csrw mtvec` are never the same edge, and the trap must
  // vector through the mtvec that was architecturally in place when it
  // issued.
  assign mtvec_value = mtvec;
  assign mepc_value  = mepc;

  // ---- read mux ---------------------------------------------------------
  always_comb begin
    implemented = 1'b1;
    (* parallel_case *)
    case (addr)
      MSTATUS:   rdata = mstatus_value;
      MISA:      rdata = MISA_VALUE;
      MIE:       rdata = 32'b0;
      MTVEC:     rdata = mtvec;
      MSCRATCH:  rdata = mscratch;
      MEPC:      rdata = mepc;
      MCAUSE:    rdata = mcause;
      MTVAL:     rdata = 32'b0;
      MIP:       rdata = 32'b0;
      MCYCLE:    rdata = mcycle_lo;
      MCYCLEH:   rdata = mcycle_hi;
      MINSTRET:  rdata = minstret_lo;
      MINSTRETH: rdata = minstret_hi;
      MVENDORID, MARCHID, MIMPID, MHARTID: rdata = 32'b0;
      default: begin
        rdata = 32'b0;
        implemented = 1'b0;
      end
    endcase
  end

  // ---- WARL ------------------------------------------------------------
  // `warl` is the value the addressed CSR holds *after* this write, with the
  // legal-value mask already applied. Defined for every address, including
  // the read-only ones (where "after this write" is the unchanged value, so
  // it falls back to `rdata`) -- that uniformity is what lets the RVFI report
  // below tell the truth about what landed rather than echoing back an
  // operand the mask threw away.
  logic [31:0] wdata_mtvec, wdata_mepc, wdata_mstatus;
  // Direct mode only: mtvec[1:0] is the mode field, and a 4-byte-aligned
  // base leaves it zero either way.
  assign wdata_mtvec   = {wdata[31:2], 2'b00};
  // Only bit 0 is forced -- C makes 2-byte branch targets legal, so bit 1
  // is a legal value here and must survive (ADR-0005).
  assign wdata_mepc    = {wdata[31:1], 1'b0};
  assign wdata_mstatus = {19'b0, 2'b11, 3'b0, wdata[7], 3'b0, wdata[3], 3'b0};

  logic [31:0] warl;
  always_comb begin
    (* parallel_case *)
    case (addr)
      MSTATUS:   warl = wdata_mstatus;
      MTVEC:     warl = wdata_mtvec;
      MSCRATCH:  warl = wdata;
      MEPC:      warl = wdata_mepc;
      MCAUSE:    warl = wdata;
      MCYCLE, MCYCLEH, MINSTRET, MINSTRETH: warl = wdata;
      // Read-only, or not implemented at all: nothing lands, so the value
      // after the write is the value before it.
      default:   warl = rdata;
    endcase
  end
  logic warl_mie, warl_mpie;
  assign warl_mie  = warl[3];
  assign warl_mpie = warl[7];

  // Trap entry writes mepc through the SAME WARL mask an explicit `csrw mepc`
  // goes through. `trap_epc` is an instruction address, so bit 0 is already
  // clear and the mask changes nothing today -- it is here so that the mask is
  // stated in one place rather than two, and so a reader can see that bit 1 is
  // deliberately preserved: C makes 2-byte targets legal (ADR-0005), and a
  // mask that cleared bit 1 too would resume two bytes early after any fault
  // at `pc % 4 == 2`. test/asm/trap.S faults a compressed instruction at
  // exactly that alignment for this reason.
  logic [31:0] trap_epc_warl;
  assign trap_epc_warl = {trap_epc[31:1], 1'b0};

  // ---- the counters -----------------------------------------------------
  logic wr_mcycle, wr_mcycleh, wr_minstret, wr_minstreth, wr_mscratch;
  assign wr_mcycle    = wen && addr == MCYCLE;
  assign wr_mcycleh   = wen && addr == MCYCLEH;
  assign wr_minstret  = wen && addr == MINSTRET;
  assign wr_minstreth = wen && addr == MINSTRETH;
  assign wr_mscratch  = wen && addr == MSCRATCH;

  // ADR-0005's "an explicit write to a counter takes precedence over that
  // cycle's increment" -- and the privileged spec's own "any CSR write takes
  // precedence over the automatic increment" (20211203 §3.1.11) -- are
  // statements about the 64-BIT COUNTER, not about the half whose address the
  // instruction happens to name. mcycle and mcycleh are two views of one
  // register, so a write to either half suppresses that cycle's increment of
  // the whole thing. Hence a `_tick` term per counter rather than the two
  // per-half overrides below doing the job on their own.
  //
  // Suppressing per half instead is wrong only at the carry boundary, and
  // wrong there in a way that nothing else in this repo can see. With
  // mcycle == 32'hffff_ffff the increment's carry lands in the HIGH half while
  // the explicit write replaces the low one, so `csrw mcycle` advances mcycleh
  // by one -- contradicting the invariant the RVFI report at the bottom of this
  // file is built to satisfy, and doing it once every 2**32 cycles and only if
  // a write falls on that exact cycle. checks/rvfi_csrw_check.sv reads the
  // SELF-REPORTED masks and never observes the register; every ladder check is
  // `mode bmc` from reset; and a `.S` program can land a write on that cycle
  // only by calibrating instruction spacing first, which is a test that stops
  // testing the moment the spacing changes. test/csr_tb.v drives this port
  // directly and is the one place it is deterministic (ADR-0048).
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
      // ADR-0029: mtvec resets to 0. The spec leaves it implementation-
      // defined and 0 is the readable choice; test/asm/sections.lds puts
      // .text there, so both sim legs are expected to make a trap taken
      // before a handler is installed loud rather than let it look like a
      // silent program restart.
      mtvec        <= 32'b0;
      mepc         <= 32'b0;
      mcause       <= 32'b0;
      mstatus_mie  <= 1'b0;
      mstatus_mpie <= 1'b0;
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
          MTVEC:    mtvec    <= warl;
          MSCRATCH: mscratch <= warl;
          MEPC:     mepc     <= warl;
          MCAUSE:   mcause   <= warl;
          // The counters are driven unconditionally above (an explicit write
          // is folded into *_next so it beats the increment); every other
          // address is read-only or unimplemented, and a write to one is a
          // no-op here by construction rather than by omission.
          default: ;
        endcase
      end
      // ---- trap entry / mret (ADR-0005) ----------------------------------
      // The privileged-spec sequence, written out: a trap saves the faulting
      // PC and the cause, pushes MIE into MPIE and disables interrupts; `mret`
      // pops it back and leaves MPIE set. There are no interrupts on this core
      // (mie/mip are read-only zero), so MIE/MPIE are architectural state a
      // program can observe through mstatus rather than something that gates
      // anything -- which is exactly why they have to be right: nothing else
      // would notice if they were not.
      //
      // `else if` rather than two independent `if`s: mutually exclusive by
      // construction (see the port comments), and stating the exclusion here
      // means a future cause that broke it fails loudly at the priority
      // encoder in rtl/decoder.v rather than silently double-writing mstatus.
      if (trap_entry) begin
        mepc         <= trap_epc_warl;
        mcause       <= trap_cause;
        mstatus_mpie <= mstatus_mie;
        mstatus_mie  <= 1'b0;
      end else if (mret_entry) begin
        mstatus_mie  <= mstatus_mpie;
        mstatus_mpie <= 1'b1;
      end
    end
  end

 `ifdef RISCV_FORMAL
  // ---- RVFI CSR reporting ------------------------------------------------
  // Write-only with respect to the core: nothing below drives a signal any
  // non-`ifdef` logic reads, which is the structural argument ADR-0020's
  // non-perturbation guarantee rests on. Do not break it.
  //
  // The two counters are ONE 64-bit RVFI CSR each, with mcycleh/minstreth
  // addressing the upper half of the same report -- which is why the masks
  // are built per half. riscv-formal's rvfi_csrw_check asserts that a write
  // through the low address leaves the high half unchanged (and vice versa),
  // computed from these reported values, so a mask that claimed both halves
  // would fail on a correct core.
  logic rd_mcycle, rd_mcycleh, rd_minstret, rd_minstreth, rd_mscratch;
  assign rd_mcycle    = ren && addr == MCYCLE;
  assign rd_mcycleh   = ren && addr == MCYCLEH;
  assign rd_minstret  = ren && addr == MINSTRET;
  assign rd_minstreth = ren && addr == MINSTRETH;
  assign rd_mscratch  = ren && addr == MSCRATCH;

  // What the *instruction* wrote, which is not what the register will hold:
  // the free-running increment is not this instruction's doing, and RVFI's
  // wdata is the instruction's write. Reporting the incremented value with a
  // zero wmask would be harmless (rvfi_csrw_check masks it away) but would
  // read as though the CSR instruction had bumped the counter.
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
 `endif
endmodule

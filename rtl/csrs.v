`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
// The machine-mode CSR file (ADR-0005).
//
// This is a sibling of the decoder, not a pipeline stage. Every access is read
// and committed in decode on the same edge the accessing instruction issues, so
// no CSR state exists downstream: nothing to forward, nothing to replay, and no
// "what does mcycle read with three instructions in flight" corner. CLAUDE.md
// invariant 5 (CSR instructions serialize) is enforced in rtl/decoder.v; this
// module has no idea a stall exists.
//
// The set below is a floor, not a closed list: every register the privileged
// spec lists unconditionally for RV32 machine mode is here, because trapping on
// a mandatory register is non-conformant however minimal the set is.
//
// Trap entry and `mret` are the second write port, and the only architectural
// CSR update no CSR instruction performs. They arrive from the decoder on the
// edge the trapping instruction issues, because that is where every trap is
// detected and committed (invariant 2). `mtvec_value`/`mepc_value` go back out
// to the decoder, which owns the PC and therefore has to redirect it.
//
// This module never decides that an access is illegal. `implemented` feeds the
// decoder's `instr_valid` term and the read-only test is on the address
// (addr[11:10] == 2'b11), so both illegal-CSR rules are decided in
// rtl/decoder.v alongside every other trap cause (ADR-0030).
module csrs(
  input  logic clk,
  input  logic reset,

  // The decode-stage access port. `addr` is instr[31:20], presented
  // combinationally every cycle; `rdata` and `implemented` answer it the same
  // cycle, because the decoder needs the read value in time to register it into
  // decoder_out at this edge.
  //
  // `ren`/`wen` describe the instruction the decoder is issuing this edge.
  // Zicsr's suppression rules (CSRRS/CSRRC with a zero source suppress the
  // write; CSRRW with rd == x0 suppresses the read) are resolved in the decoder,
  // where the operand fields live, so `wen` here means "commit this".
  input  logic [11:0] addr,
  input  logic        ren,
  input  logic        wen,
  input  logic [31:0] wdata,
  output logic [31:0] rdata,
  output logic        implemented,

  // minstret counts non-trapping issues, so this is high for exactly the cycles
  // the decoder issues one (ADR-0027).
  input  logic        instret,

  // `trap_entry` is high for exactly the cycle the decoder commits a trap;
  // `mret_entry` for exactly the cycle it commits an `mret`. They cannot
  // coincide with each other (a trapping instruction does not also execute) nor
  // with `wen` (the decoder gates `csr_wen` on a non-trapping commit, and `mret`
  // is not a CSR access), so the three write paths below need no priority
  // between them -- the code states the exclusion rather than relying on
  // last-assignment-wins (ADR-0028).
  input  logic        trap_entry,
  input  logic [31:0] trap_cause,
  input  logic [31:0] trap_epc,
  input  logic        mret_entry,
  // Read back by rtl/decoder.v, combinationally, to redirect the PC.
  output logic [31:0] mtvec_value,
  output logic [31:0] mepc_value
 `ifdef RISCV_FORMAL
  ,
  // Shadow payloads for exactly the CSRs formal/checks.cfg's `[csrs]` list
  // names. Combinational off this cycle's access; the decoder latches them into
  // the issuing instruction's shadow so they ride to writeback on the ordinary
  // valid-bit protocol.
  output rvfi_csr64 rvfi_mcycle,
  output rvfi_csr64 rvfi_minstret,
  output rvfi_csr32 rvfi_mscratch
 `endif
);
  localparam logic [11:0] MSTATUS   = 12'h300;
  localparam logic [11:0] MISA      = 12'h301;
  // Both read-only zero, and zero is a legal value for each rather than a stub:
  // mstatush's only fields are SBE and MBE, where 0 means the little-endian
  // M-mode data accesses this core does, and mconfigptr = 0 is the spec's own
  // encoding for "no configuration data structure exists".
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

  // RV32 I M C, MXL = 1 -- CLAUDE.md's ISA target.
  localparam logic [31:0] MISA_VALUE = 32'h4000_1104;

  logic [63:0] mcycle, minstret;
  logic [31:0] mscratch, mtvec, mepc, mcause;
  // mstatus is three fields rather than a register: MPP is hardwired to M-mode
  // (WARL, and this core has no other mode) and every other bit is 0.
  logic        mstatus_mie, mstatus_mpie;

  // Halves of the two counters, selected out here rather than inside the
  // always_* blocks below. A constant part-select taken inside an always_* block
  // defeats iverilog's sensitivity analysis and draws a `sorry:` note, so this
  // file uses named continuous assigns throughout (ADR-0034).
  logic [31:0] mcycle_lo, mcycle_hi, minstret_lo, minstret_hi;
  assign mcycle_lo   = mcycle[31:0];
  assign mcycle_hi   = mcycle[63:32];
  assign minstret_lo = minstret[31:0];
  assign minstret_hi = minstret[63:32];

  logic [31:0] mstatus_value;
  // MPP = 2'b11 at [12:11]; MPIE at [7]; MIE at [3]; everything else 0.
  assign mstatus_value = {19'b0, 2'b11, 3'b0, mstatus_mpie, 3'b0, mstatus_mie, 3'b0};

  // The two registers the decoder redirects the PC through, as of the start of
  // the issuing cycle. That phase is the right one: decode issues at most one
  // instruction per cycle, so a trapping instruction and a `csrw mtvec` are
  // never the same edge, and the trap must vector through the mtvec that was
  // architecturally in place when it issued.
  assign mtvec_value = mtvec;
  assign mepc_value  = mepc;

  always_comb begin
    implemented = 1'b1;
    (* parallel_case *)
    case (addr)
      MSTATUS:   rdata = mstatus_value;
      MSTATUSH:  rdata = 32'b0;
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
      MVENDORID, MARCHID, MIMPID, MHARTID, MCONFIGPTR: rdata = 32'b0;
      default: begin
        rdata = 32'b0;
        implemented = 1'b0;
      end
    endcase
  end

  // `warl` is the value the addressed CSR holds after this write, with the
  // legal-value mask already applied. It is defined for every address, including
  // the read-only ones (where it falls back to `rdata`), and that uniformity is
  // what lets the RVFI report below tell the truth about what landed rather than
  // echo back an operand the mask threw away.
  logic [31:0] wdata_mtvec, wdata_mepc, wdata_mstatus;
  // Direct mode only: mtvec[1:0] is the mode field, and a 4-byte-aligned
  // base leaves it zero either way.
  assign wdata_mtvec   = {wdata[31:2], 2'b00};
  // Only bit 0 is forced: C makes 2-byte branch targets legal, so bit 1 is a
  // legal value here and must survive.
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

  // Trap entry writes mepc through the same WARL mask an explicit `csrw mepc`
  // goes through. `trap_epc` is an instruction address, so bit 0 is already
  // clear and the mask changes nothing today; what it states is that bit 1 is
  // preserved, because a mask that cleared it too would resume two bytes early
  // after a fault at `pc % 4 == 2`. test/asm/trap.S faults a compressed
  // instruction at exactly that alignment.
  logic [31:0] trap_epc_warl;
  assign trap_epc_warl = {trap_epc[31:1], 1'b0};

  logic wr_mcycle, wr_mcycleh, wr_minstret, wr_minstreth, wr_mscratch;
  assign wr_mcycle    = wen && addr == MCYCLE;
  assign wr_mcycleh   = wen && addr == MCYCLEH;
  assign wr_minstret  = wen && addr == MINSTRET;
  assign wr_minstreth = wen && addr == MINSTRETH;
  assign wr_mscratch  = wen && addr == MSCRATCH;

  // "Any CSR write takes precedence over the automatic increment" (priv spec
  // 20211203 §3.1.11) is a statement about the 64-bit COUNTER, not about the
  // half whose address the instruction names: mcycle and mcycleh are two views
  // of one register, so a write to either half suppresses that cycle's increment
  // of the whole thing. Hence a `_tick` term per counter rather than the two
  // per-half overrides below doing the job alone.
  //
  // Suppressing per half instead differs only at the carry boundary, where the
  // increment's carry lands in the high half while the write replaces the low
  // one -- so `csrw mcycle` advances mcycleh. test/csr_tb.v is the only thing
  // that can see it (ADR-0048).
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
      // The spec leaves mtvec's reset value implementation-defined. 0 puts it
      // at test/asm/sections.lds's .text, so both sim legs catch a trap taken
      // before a handler is installed rather than let it look like a silent
      // program restart (ADR-0029).
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
          // The counters are driven unconditionally above, with an explicit
          // write folded into *_next so it beats the increment. Every other
          // address is read-only or unimplemented.
          default: ;
        endcase
      end
      // The privileged-spec sequence: a trap saves the faulting PC and the
      // cause, pushes MIE into MPIE and disables interrupts; `mret` pops it back
      // and leaves MPIE set. This core has no interrupts (mie/mip are read-only
      // zero), so MIE/MPIE gate nothing and are only observable through mstatus
      // -- nothing but test/csr_tb.v would notice them being wrong.
      //
      // `else if` rather than two independent `if`s: the two are mutually
      // exclusive by construction (see the port comments), and stating that here
      // means a future cause that broke it fails at rtl/decoder.v's priority
      // encoder rather than silently double-writing mstatus.
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
  // Write-only with respect to the core: nothing below drives a signal any
  // non-`ifdef` logic reads. That is the structural argument the
  // non-perturbation guarantee rests on (ADR-0047), so do not break it.
  //
  // The two counters are one 64-bit RVFI CSR each, with mcycleh/minstreth
  // addressing the upper half of the same report, which is why the masks are
  // built per half. rvfi_csrw_check asserts that a write through the low address
  // leaves the high half unchanged, computed from these reported values, so a
  // mask claiming both halves fails on a correct core.
  logic rd_mcycle, rd_mcycleh, rd_minstret, rd_minstreth, rd_mscratch;
  assign rd_mcycle    = ren && addr == MCYCLE;
  assign rd_mcycleh   = ren && addr == MCYCLEH;
  assign rd_minstret  = ren && addr == MINSTRET;
  assign rd_minstreth = ren && addr == MINSTRETH;
  assign rd_mscratch  = ren && addr == MSCRATCH;

  // What the instruction wrote, which is not what the register will hold: the
  // free-running increment is not this instruction's doing, and RVFI's wdata is
  // the instruction's write. Reporting the incremented value with a zero wmask
  // would be harmless (rvfi_csrw_check masks it away) but would read as though
  // the CSR instruction had bumped the counter.
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

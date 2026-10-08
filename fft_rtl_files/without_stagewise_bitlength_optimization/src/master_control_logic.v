`timescale 1ns / 1ps
// =============================================================================
// Module : master_control_logic  (continuous-flow revision)
// Purpose: Sequencing controller for all three FFT/IFFT stages (Fig.4).
//          Generates cycle_cnt, set_sel, wr_en, valid_in for each stage and
//          per-frame EOP, supporting CONTINUOUS FLOW: a new 4096-sample
//          frame may start every N/P = 256 CC, with up to
//          ceil(861/256) = 4 frames overlapped in the pipeline.
//
// ALL timing parameters are taken directly from paper Eq.21 (Sec.V):
//
//   Ctotal = 256 + (32 + 16) + (32 + 256) + 13 = 605 CC (first output)
//   Full-frame streaming adds N/P = 256 CC (Sec.V) -> 861 CC per frame.
//
//   Frame-relative schedule (offsets from the frame's first data cycle):
//     Stage-I   writes   0..255      reads 256..511
//     Stage-II  writes 288..543      reads 304..559
//     Stage-III writes 336..591      reads 592..847
//     outputs 605..860, EOP pulse at 605 (first output sample)
//
// ARCHITECTURE - occupancy delay line:
//   A master counter mc free-runs once started; every stage phase is a
//   fixed Eq.21 offset of the frame start, and every phase lasts exactly
//   256 CC, so back-to-back frames tile seamlessly on the modular
//   counters (cycle_cnt_sX / set_sel_sX below). Which 256-CC windows
//   actually carry a frame is tracked by a 1-bit occupancy flag:
//     occ_now             : the current window carries a frame
//     occ_pipe[d]         : occupancy d cycles ago
//   Each stage enable is simply the occupancy delayed by its Eq.21
//   offset:
//     wr_en_s1    = occ_now            valid_in_s1 = occ_pipe[256]
//     wr_en_s2    = occ_pipe[288]      valid_in_s2 = occ_pipe[304]
//     wr_en_s3    = occ_pipe[336]      valid_in_s3 = occ_pipe[592]
//   This one mechanism covers single frames (old behaviour, windows
//   identical to the verified single-frame controller), gapped frames
//   (gaps in multiples of 256 CC), and full back-to-back streaming.
//
// SET ALTERNATION (in-place, paper Sec.III-A):
//   set_sel_sX toggles once per stage window and free-runs with mc, so
//   the pairing "frame written with one set is read with the other,
//   which simultaneously serves as the write set of the next frame"
//   holds automatically for any frame spacing. Stage-I/II closure was
//   established earlier; Stage-III closure requires the Fig.8 Addr_set2
//   ({k2,k1} rows - see aagu_stage3.v), with which all three stages
//   alternate with period 2.
//
// SOP PROTOCOL:
//   - Idle: a SOP pulse starts the machine; frame data cycle 0 must be
//     presented the following cycle (= mc 0).
//   - Running: SOP is sampled ONLY during the LAST cycle of each input
//     window (mc[7:0] == 255); if high, the next window carries a new
//     frame whose data starts the following cycle. This is the same
//     "SOP one cycle before the frame's first data" relative timing as
//     the idle case. SOP at any other cycle while running is ignored.
//   - The controller returns to idle automatically once no frame
//     occupies any pipeline stage (all occupancy taps clear, i.e. 861 CC
//     after the last frame's start).
//
//   NOTE: fft_ifft (top level) must be held constant while frames are
//   in flight; switch modes only when the pipeline is drained.
//
// Compatible: Verilog-2001, Vivado 2022.2
// =============================================================================
module master_control_logic #(
    // -------------------------------------------------------------------------
    // Structural parameters - do not change without paper justification
    // -------------------------------------------------------------------------
    parameter N           = 4096, // FFT/IFFT size  (Eq.3)
    parameter P           = 16,   // Parallel streams (Fig.4: 16x16-bit I/O)
    parameter CPLSTGI     = 32,   // Cpipelined_stgI   (paper Eq.21)
    parameter CPLSTGII    = 32,   // Cpipelined_stgII  (paper Eq.21)
    parameter CPLSTGIII   = 13    // Cpipelined_stgIII (paper Eq.21)
)(
    input  wire  clk,
    input  wire  rst,
    input  wire  sop,       // Start-of-Process (Fig.4, Sec.III) - see protocol

    output reg   eop,       // End-of-Process - 1-cycle pulse per frame,
                            // aligned with the frame's FIRST output sample

    // --- Stage-I control (feeds stage1_datapath / aagu_stage1) ---
    output wire [7:0] cycle_cnt_s1,
    output wire       set_sel_s1,
    output wire       wr_en_s1,
    output wire       valid_in_s1,

    // --- Stage-II control (feeds stage2_datapath / aagu_stage2) ---
    output wire [3:0] cycle_cnt_s2,
    output wire       set_sel_s2,
    output wire       wr_en_s2,
    output wire       valid_in_s2,

    // --- Stage-III control (feeds stage3_datapath / aagu_stage3) ---
    output wire [7:0] cycle_cnt_s3,
    output wire       set_sel_s3,
    output wire       wr_en_s3,
    output wire       valid_in_s3,
    output wire [15:0] debug_mc,
    // ---- NOVELTY #1: exact per-block liveness (occupancy-driven gating) ----
    // Each occ_pipe tap is a 256-cycle-wide pulse delayed by its index, so an
    // OR-reduction over a block's span is EXACT liveness for that block.
    output wire        live_m1,   // Stage-I   memory path (write+read)
    output wire        live_b1,   // Stage-I   BS16 compute
    output wire        live_c1,   // Stage-I   CORDIC bank
    output wire        live_m2,   // Stage-II  memory path
    output wire        live_b2,   // Stage-II  BS16 compute
    output wire        live_c2,   // Stage-II  CORDIC bank
    output wire        live_m3,   // Stage-III memory path
    output wire        live_b3
);

    // =========================================================================
    // Derived constants - ALL from Eq.21
    // =========================================================================
    localparam MC_S1_RD   = N/P;                     // 256
    localparam MC_S2_WR   = MC_S1_RD + CPLSTGI;      // 288
    localparam MC_S2_RD   = MC_S2_WR + N/(16*P);     // 304
    localparam MC_S3_WR   = MC_S2_RD + CPLSTGII;     // 336
    localparam MC_S3_RD   = MC_S3_WR + N/P;          // 592
    localparam MC_EOP     = MC_S3_RD + CPLSTGIII;    // 605 (first output)

    // =========================================================================
    // Master counter + occupancy delay line
    // =========================================================================
    reg        running;
    reg [15:0] mc;                 // free-runs while running; wraps mod 2^16
                                   // (a multiple of 256, so window phase is
                                   // preserved across wrap)
    reg        occ_now;            // current input window carries a frame
    reg [MC_EOP:1] occ_pipe;       // occ_now delayed 1..605 cycles

    wire window_last = (mc[7:0] == 8'hFF);
    wire pipe_empty  = !occ_now && (occ_pipe == {MC_EOP{1'b0}});

    always @(posedge clk) begin
        if (rst) begin
            running  <= 1'b0;
            mc       <= 16'd0;
            occ_now  <= 1'b0;
            occ_pipe <= {MC_EOP{1'b0}};
            eop      <= 1'b0;
        end else begin
            // Per-frame EOP: high during the frame's first output cycle
            // t = frame_start + 605. Registered one cycle early:
            // occ_pipe[604] = occ(t-605) and mc[7:0] == (605-1) mod 256.
            eop <= running && occ_pipe[MC_EOP-1]
                           && (mc[7:0] == ((MC_EOP-1) % 256));

            if (!running) begin
                if (sop) begin
                    running <= 1'b1;
                    mc      <= 16'd0;
                    occ_now <= 1'b1;   // first window carries the frame
                end
            end else begin
                mc       <= mc + 16'd1;
                occ_pipe <= {occ_pipe[MC_EOP-1:1], occ_now};
                if (window_last)
                    occ_now <= sop;    // new frame iff SOP during cycle 255
                if (pipe_empty && !sop)
                    running <= 1'b0;   // auto-idle: nothing left in flight
            end
        end
    end

    // =========================================================================
    // Stage-I (aagu_stage1: 8-bit cycle_cnt, set alternates every 256 CC)
    // =========================================================================
    assign cycle_cnt_s1 = mc[7:0];
    assign set_sel_s1   = mc[8];
    assign wr_en_s1     = running && occ_now;
    assign valid_in_s1  = running && occ_pipe[MC_S1_RD];

    // =========================================================================
    // Stage-II (aagu_stage2: 4-bit cycle_cnt, set alternates every 16 CC)
    // 256 CC per frame = 16 stage-II windows (even count), so alternation
    // parity tiles seamlessly across back-to-back frames.
    // =========================================================================
    wire [15:0] s2_cnt  = mc - MC_S2_WR[15:0]; // don't-care when disabled
    assign cycle_cnt_s2 = s2_cnt[3:0];
    assign set_sel_s2   = s2_cnt[4];
    assign wr_en_s2     = running && occ_pipe[MC_S2_WR];
    assign valid_in_s2  = running && occ_pipe[MC_S2_RD];

    // =========================================================================
    // Stage-III (aagu_stage3: 8-bit cycle_cnt, set alternates every 256 CC)
    // =========================================================================
    wire [15:0] s3_cnt  = mc - MC_S3_WR[15:0]; // don't-care when disabled
    assign cycle_cnt_s3 = s3_cnt[7:0];
    assign set_sel_s3   = s3_cnt[8];
    assign wr_en_s3     = running && occ_pipe[MC_S3_WR];
    assign valid_in_s3  = running && occ_pipe[MC_S3_RD];
    assign debug_mc = mc;


    // =========================================================================
    // NOVELTY #1: occupancy-driven liveness
    //   Spans (absolute pipeline positions a frame occupies in each block):
    //     Stage-I  : input-load window (occ_now) .. Stage-II write   [0 .. MC_S2_WR]
    //     S1 CORDIC: Stage-I read .. Stage-II write        [MC_S1_RD .. MC_S2_WR]
    //     Stage-II : Stage-II write .. Stage-III write     [MC_S2_WR .. MC_S3_WR]
    //     S2 CORDIC: Stage-II read .. Stage-III write      [MC_S2_RD .. MC_S3_WR]
    //     Stage-III: Stage-III write .. output drain       [MC_S3_WR .. MC_EOP]
    //   occ_pipe[MC_EOP] stays high for the full 256-cycle output burst, so the
    //   drain is covered without extra taps.
    // =========================================================================
    // Memory paths: clocked only on their write window OR their read window,
    // with +2 taps of margin for the registered (1 CC) memory read output.
    // BS16 compute: live only while read data flows through the 3-CC butterfly,
    // i.e. idle during the entire memory write-only phase. This split is what
    // removes the Stage-I / Stage-III saturation seen with one domain per stage.
    assign live_m1 = running && ( occ_now || |occ_pipe[MC_S1_RD+2:MC_S1_RD] );
    assign live_b1 = running && ( |occ_pipe[MC_S1_RD+4:MC_S1_RD] );
    assign live_m2 = running && ( |occ_pipe[MC_S2_WR+2:MC_S2_WR] || |occ_pipe[MC_S2_RD+2:MC_S2_RD] );
    assign live_b2 = running && ( |occ_pipe[MC_S2_RD+4:MC_S2_RD] );
    assign live_m3 = running && ( |occ_pipe[MC_S3_WR+2:MC_S3_WR] || |occ_pipe[MC_S3_RD+2:MC_S3_RD] );
    assign live_b3 = running && ( |occ_pipe[MC_EOP:MC_S3_RD] );
    assign live_c1 = running && ( |occ_pipe[MC_S2_WR:MC_S1_RD] );
    assign live_c2 = running && ( |occ_pipe[MC_S3_WR:MC_S2_RD] );

endmodule

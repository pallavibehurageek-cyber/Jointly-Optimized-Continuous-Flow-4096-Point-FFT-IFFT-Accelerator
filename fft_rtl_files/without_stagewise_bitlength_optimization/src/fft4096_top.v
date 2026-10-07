`timescale 1ns / 1ps
// =============================================================================
// Module : fft4096_top
// Purpose: Top-level integration of the 4096-point FFT/IFFT hardware
//          architecture, faithfully replicating Fig.4 of the paper.
//
// Architecture (Fig.4, paper Sec.III) - THREE memory units, one per stage:
//
//   x_in --> [Conj] --> [2x1 MUX] --> Stage-I  --> [CORDIC x16] --> [pad 16]
//                                                                      |
//                                     Stage-II <------------------------
//                                        |
//                                        +--> [CORDIC x16] --> [pad 16]
//                                                                  |
//                                     Stage-III <-------------------
//                                        |
//                                        +--> [pad 9] --> [Conj] --> [2x1 MUX] --> y_out
//
//   Per Sec.III-A the inter-stage reordering is performed IN-PLACE by each
//   stage's own RCS -> 16 memory banks -> LCS structure under conflict-free
//   AAGU addressing ("three memory units are efficiently utilized"). There
//   are NO standalone inter-stage reorder buffers in the architecture; the
//   Eq.21 schedule (16-CC set fill before Stage-II reads, 256-CC fill before
//   Stage-III reads) is realized by the Master Control Logic windows.
//
//   Master Control Logic drives SOP/EOP and all stage control signals.
//
// FFT/IFFT mode (Eq.2, paper Sec.II):
//   IFFT is computed by conjugating the input, running the same FFT
//   hardware, then conjugating the output.  The 2x1 input MUX selects
//   between raw input (FFT) and conjugated input (IFFT); the output MUX
//   selects between raw Stage-III output (FFT) and conjugated (IFFT),
//   controlled by (fft_ifft AND EN), EN activated by EOP (paper Sec.III).
//
// Twiddle factors (Eq.6, Eq.8, paper Sec.III-B):
//   Stage-I inter-stage TF : W^(nu2*k1)_4096  for k1=0..15
//   Stage-II inter-stage TF: W^(n3*k2)_256    for k2=0..15
//   Both implemented with CORDIC (Fig.13) using Q2.18 angle format:
//     theta_s1[k1] = -(nu2 * k1 * 256)  [20-bit, wraps mod 2^20 = 2pi period]
//     theta_s2[k2] = -(n3  * k2 * 4096) [20-bit, wraps mod 2^20]
//
// Timing (Eq.21, paper Sec.V) - paper constants, padded to exactly:
//   Cpipelined_stgI = Cpipelined_stgII = 32, Cpipelined_stgIII = 13.
//   Actual sub-module latencies:
//     mem registered read = 1, bs16_unit = 3, cordic_rotator = 12
//     Stage-I/II raw = 16 -> pad 16;  Stage-III raw = 4 -> pad 9
//   (extra_pipe_regs banks below). Resulting frame timeline (mc from SOP):
//     Stage-I  writes   0..255, reads 256..511
//     Stage-II writes 288..543, reads 304..559
//     Stage-III writes 336..591, reads 592..847
//     First output at mc = 605 (= Ctotal, Eq.21), last at mc = 860
//     (paper Sec.V: full-frame transfer adds N/P = 256 CC -> 861 total)
//
// Compatible: Verilog-2001, Vivado 2022.2
// =============================================================================
module fft4096_top #(
    parameter W  = 16,   // complex word width (signed)
    parameter WZ = 20    // CORDIC phase accumulator width (Q2.18)
)(
    input  wire             clk,
    input  wire             rst,
    input  wire             sop,        // Start-of-Process (paper Fig.4)
    input  wire             fft_ifft,   // 0=FFT, 1=IFFT (paper Sec.III)

    // 16-parallel complex input (one group per clock, 256 groups per frame)
    input  wire [W*16-1:0]  x_re_flat,
    input  wire [W*16-1:0]  x_im_flat,

    output wire             eop,        // End-of-Process pulse (paper Fig.4)
    output wire             valid_out,  // output data valid (256 cycles/frame)

    // 16-parallel complex output
    output wire [W*16-1:0]  y_re_flat,
    output wire [W*16-1:0]  y_im_flat
);

    // =========================================================================
    // Pipeline padding depths (paper Eq.21 constants minus implemented latency)
    //   Stage-I : 32 - (1 mem + 3 BS16 + 12 CORDIC) = 16
    //   Stage-II: 32 - (1 mem + 3 BS16 + 12 CORDIC) = 16
    //   Stage-III: 13 - (1 mem + 3 BS16)            =  9
    // =========================================================================
    localparam PAD_S1  = 16;
    localparam PAD_S2  = 16;
    localparam PAD_S3  = 9;

    // =========================================================================
    // Master Control Logic (paper Fig.4, Sec.V Eq.21)
    // =========================================================================
    wire [7:0] cycle_cnt_s1;
    wire       set_sel_s1, wr_en_s1, valid_in_s1;
    wire [3:0] cycle_cnt_s2;
    wire       set_sel_s2, wr_en_s2, valid_in_s2;
    wire [7:0] cycle_cnt_s3;
    wire       set_sel_s3, wr_en_s3, valid_in_s3;

    // =========================================================================
    // NOVELTY #1: occupancy-driven clock gating
    //   Exact per-block liveness from the controller's occupancy line drives
    //   ICG cells. When a block holds no live data its clock is suppressed,
    //   eliminating dynamic power spent clocking stale registers. Functionally
    //   transparent: gating only removes edges where no state change occurs.
    // =========================================================================
    wire live_m1, live_b1, live_c1, live_m2, live_b2, live_c2, live_m3, live_b3;
    wire gclk_m1, gclk_b1, gclk_c1, gclk_m2, gclk_b2, gclk_c2, gclk_m3, gclk_b3;

    clock_gate u_cg_m1 (.clk(clk), .en(live_m1), .gclk(gclk_m1));  // S1 memory
    clock_gate u_cg_b1 (.clk(clk), .en(live_b1), .gclk(gclk_b1));  // S1 BS16
    clock_gate u_cg_c1 (.clk(clk), .en(live_c1), .gclk(gclk_c1));  // S1 CORDIC
    clock_gate u_cg_m2 (.clk(clk), .en(live_m2), .gclk(gclk_m2));  // S2 memory
    clock_gate u_cg_b2 (.clk(clk), .en(live_b2), .gclk(gclk_b2));  // S2 BS16
    clock_gate u_cg_c2 (.clk(clk), .en(live_c2), .gclk(gclk_c2));  // S2 CORDIC
    clock_gate u_cg_m3 (.clk(clk), .en(live_m3), .gclk(gclk_m3));  // S3 memory
    clock_gate u_cg_b3 (.clk(clk), .en(live_b3), .gclk(gclk_b3));  // S3 BS16

    master_control_logic #(
        .N(4096), .P(16),
        .CPLSTGI(13), .CPLSTGII(13), .CPLSTGIII(4)  // CORDIC now 9 CC -> 558 total  // FORK-B: actual PE latencies (mem1+bs3+cordic12 / +0). occ_pipe -> 564 CC
    ) u_mcl (
        .clk          (clk),          .rst      (rst),
        .sop          (sop),          .eop      (eop),
        .cycle_cnt_s1 (cycle_cnt_s1), .set_sel_s1 (set_sel_s1),
        .wr_en_s1     (wr_en_s1),     .valid_in_s1(valid_in_s1),
        .cycle_cnt_s2 (cycle_cnt_s2), .set_sel_s2 (set_sel_s2),
        .wr_en_s2     (wr_en_s2),     .valid_in_s2(valid_in_s2),
        .cycle_cnt_s3 (cycle_cnt_s3), .set_sel_s3 (set_sel_s3),
        .wr_en_s3     (wr_en_s3),     .valid_in_s3(valid_in_s3),
        .debug_mc     (),
        // NOVELTY #1 liveness taps
        .live_m1      (live_m1),
        .live_b1      (live_b1),
        .live_c1      (live_c1),
        .live_m2      (live_m2),
        .live_b2      (live_b2),
        .live_c2      (live_c2),
        .live_m3      (live_m3),
        .live_b3      (live_b3)
    );

    // =========================================================================
    // EN control: activated by EOP, enables output MUX (paper Sec.III).
    // EOP is aligned with the FIRST output sample (mc=605); include eop in
    // the select so the first output cycle is also correctly conjugated in
    // IFFT mode.
    // =========================================================================
    reg en;
    always @(posedge clk) begin
        if (rst)      en <= 1'b0;
        else if (eop) en <= 1'b1;
    end

    // =========================================================================
    // Input Conjugate and 2x1 MUX (paper Sec.II, Eq.2, Fig.4)
    //   FFT  (fft_ifft=0): feed x as-is
    //   IFFT (fft_ifft=1): feed x* (negate imaginary parts)
    // =========================================================================
    wire [W*16-1:0] x_im_conj;
    genvar ci;
    generate
        for (ci = 0; ci < 16; ci = ci + 1) begin : CONJ_IN
            assign x_im_conj[W*(15-ci) +: W] = -$signed(x_im_flat[W*(15-ci) +: W]);
        end
    endgenerate

    wire [W*16-1:0] in_re_mux = x_re_flat;
    wire [W*16-1:0] in_im_mux = fft_ifft ? x_im_conj : x_im_flat;

    // =========================================================================
    // Stage-I datapath (paper Fig.4, Fig.5, Fig.6, Sec.III-A.1)
    // =========================================================================
    wire [W*16-1:0] s1_y_re, s1_y_im;
    wire            s1_valid_out;

    stage1_datapath #(.W(W)) u_s1 (
        .clk           (gclk_m1),
        .clk_c         (gclk_b1),
        .rst           (rst),
        .cycle_cnt     (cycle_cnt_s1),
        .set_sel       (set_sel_s1),
        .wr_en         (wr_en_s1),
        .valid_in      (valid_in_s1),
        .x_re_flat     (in_re_mux),
        .x_im_flat     (in_im_mux),
        .pe_in_re_flat (),             // not used at top-level
        .pe_in_im_flat (),
        .pe_in_valid   (),
        .y_re_flat     (s1_y_re),
        .y_im_flat     (s1_y_im),
        .valid_out     (s1_valid_out)
    );

    // =========================================================================
    // Stage-I inter-stage twiddle CORDIC x16 (paper Eq.6, Fig.13)
    //   TF = W^(nu2*k1)_4096 = exp(-j*2pi*nu2*k1/4096)
    //   In Q2.18: theta_k1 = -(nu2 * k1 * 256) [mod 2^20, 20-bit signed]
    //
    //   nu2 index: derived from the output-cycle position counter below.
    //   The position counter increments while s1_valid_out is high (a
    //   continuous 256-cycle window per frame) and clears while idle.
    //   (Replaces the previous enable-latch scheme, which held the index at
    //   0 for the first TWO valid cycles and free-ran between frames.)
    // =========================================================================
    reg  [7:0] s1_pos;          // Stage-I output-cycle position r = 0..255
    wire [7:0] s1_ni2;          // nu2 at Stage-I CORDIC input (Eq.6)

    always @(posedge clk) begin
        if (rst)                s1_pos <= 8'd0;
        else if (!s1_valid_out) s1_pos <= 8'd0;          // idle between frames
        else                    s1_pos <= s1_pos + 8'd1; // next output cycle
    end

    // Stage-I reads emit n2-FAST (paper Sec.III-A.2: "after 16 rounds of
    // operation... all concurrent elements for the second stage... are made
    // available"): output cycle r carries (n2, n3) = (r[3:0], r[7:4]).
    // Eq.6 defines nu2 = 16*n2 + n3, so the angle index is the NIBBLE-SWAP
    // of the position counter, not the counter itself:
    assign s1_ni2 = {s1_pos[3:0], s1_pos[7:4]};

    // 16 CORDIC rotators: one per k1 lane
    wire [W*16-1:0]  s1c_re, s1c_im;  // Stage-I CORDIC outputs
    wire [16-1:0]    s1c_valid;        // per-lane valid

    genvar k1v;
    generate
        for (k1v = 0; k1v < 16; k1v = k1v + 1) begin : S1_CORDIC
            // theta = -(nu2 * k1 * 256) mod 2^20, 20-bit signed
            // Product max: 255*15*256 = 979200 < 2^20=1048576, so mod
            // naturally fits after taking lower 20 bits of signed product.
            wire signed [19:0] theta_s1 =
                $signed(20'd0) -
                $signed({{12{1'b0}}, s1_ni2}) *
                $signed({{16{1'b0}}, k1v[3:0]}) *
                $signed(20'd256);

            cordic_rotator #(.W(W), .WZ(WZ)) u_cord (
                .clk       (gclk_c1),
                .rst       (rst),
                .valid_in  (s1_valid_out),
                .x_in      (s1_y_re[W*(15-k1v) +: W]),
                .y_in      (s1_y_im[W*(15-k1v) +: W]),
                .theta_in  (theta_s1),
                .valid_out (s1c_valid[k1v]),
                .x_out     (s1c_re[W*(15-k1v) +: W]),
                .y_out     (s1c_im[W*(15-k1v) +: W])
            );
        end
    endgenerate

    // =========================================================================
    // Stage-I -> Stage-II pipeline padding (Eq.21 alignment, see header).
    // Total Stage-I PE latency = 16 (raw) + 16 (pad) = Cpipelined_stgI = 32,
    // so Stage-I data reaches the Stage-II memory write port exactly at
    // mc = MC_S2_WR = 288, in step with cycle_cnt_s2 = 0.
    //
    // Inter-stage reordering itself is performed by the Stage-II memory
    // (RCS -> banks -> LCS under aagu_stage2), per paper Sec.III-A. No
    // standalone reorder buffer exists in Fig.4.
    // =========================================================================
    wire [W*16-1:0] s2_in_re, s2_in_im;
    wire            s2_in_valid;

    // FORK-B: pad removed; Stage-I CORDIC output wired direct to Stage-II write.
    assign s2_in_re = s1c_re;  assign s2_in_im = s1c_im;  assign s2_in_valid = s1c_valid[0];

    // =========================================================================
    // Stage-II datapath (paper Fig.4, Fig.5, Fig.7, Sec.III-A.2)
    // =========================================================================
    wire [W*16-1:0] s2_y_re, s2_y_im;
    wire            s2_valid_out;

    stage2_datapath #(.W(W)) u_s2 (
        .clk           (gclk_m2),
        .clk_c         (gclk_b2),
        .rst           (rst),
        .cycle_cnt     (cycle_cnt_s2),
        .set_sel       (set_sel_s2),
        .wr_en         (wr_en_s2),
        .valid_in      (valid_in_s2),
        .x_re_flat     (s2_in_re),
        .x_im_flat     (s2_in_im),
        .pe_in_re_flat (),
        .pe_in_im_flat (),
        .pe_in_valid   (),
        .y_re_flat     (s2_y_re),
        .y_im_flat     (s2_y_im),
        .valid_out     (s2_valid_out)
    );

    // =========================================================================
    // Stage-II inter-stage twiddle CORDIC x16 (paper Eq.8, Fig.13)
    //   TF = W^(n3*k2)_256 = exp(-j*2pi*n3*k2/256)
    //   In Q2.18: theta_k2 = -(n3 * k2 * 4096) [mod 2^20, 20-bit signed]
    //
    //   n3 index: set index (0..15) of the data at the Stage-II CORDIC
    //   input, derived from the output-cycle position counter below.
    // =========================================================================
    reg  [7:0] s2_pos;          // Stage-II output-cycle position t = 0..255
    wire [3:0] s2_n3;           // n3 at Stage-II CORDIC input (Eq.8)

    always @(posedge clk) begin
        if (rst)                s2_pos <= 8'd0;
        else if (!s2_valid_out) s2_pos <= 8'd0;          // idle between frames
        else                    s2_pos <= s2_pos + 8'd1; // next output cycle
    end

    // Stage-II emits one k1 read-column per cycle within each n3 set (forced
    // by the 16x16 in-place transpose): output cycle t carries set index
    // n3 = t[7:4] (SLOW digit) and within-set position k1 = t[3:0]. Eq.8's
    // twiddle W^(n3*k2)_256 therefore uses the HIGH nibble:
    assign s2_n3 = s2_pos[7:4];

    wire [W*16-1:0]  s2c_re, s2c_im;
    wire [16-1:0]    s2c_valid;

    genvar k2v;
    generate
        for (k2v = 0; k2v < 16; k2v = k2v + 1) begin : S2_CORDIC
            // theta = -(n3 * k2 * 4096) mod 2^20, 20-bit signed
            // Product max: 15*15*4096 = 921600 < 2^20; wraps correctly.
            wire signed [19:0] theta_s2 =
                $signed(20'd0) -
                $signed({{16{1'b0}}, s2_n3}) *
                $signed({{16{1'b0}}, k2v[3:0]}) *
                $signed(20'd4096);

            cordic_rotator #(.W(W), .WZ(WZ)) u_cord (
                .clk       (gclk_c2),
                .rst       (rst),
                .valid_in  (s2_valid_out),
                .x_in      (s2_y_re[W*(15-k2v) +: W]),
                .y_in      (s2_y_im[W*(15-k2v) +: W]),
                .theta_in  (theta_s2),
                .valid_out (s2c_valid[k2v]),
                .x_out     (s2c_re[W*(15-k2v) +: W]),
                .y_out     (s2c_im[W*(15-k2v) +: W])
            );
        end
    endgenerate

    // =========================================================================
    // Stage-II -> Stage-III pipeline padding (Eq.21 alignment).
    // Total Stage-II PE latency = 16 (raw) + 16 (pad) = Cpipelined_stgII = 32,
    // so Stage-II data reaches the Stage-III memory write port exactly at
    // mc = MC_S3_WR = 336, in step with cycle_cnt_s3 = 0.
    //
    // The full-frame (N/P = 256 CC) reordering between Stage-II and Stage-III
    // is performed by the Stage-III memory itself (DEPTH=256 banks under
    // aagu_stage3) - this fill time is the "N/P" term of Eq.21.
    // =========================================================================
    wire [W*16-1:0] s3_in_re, s3_in_im;
    wire            s3_in_valid;

    // FORK-B: pad removed; Stage-II CORDIC output wired direct to Stage-III write.
    assign s3_in_re = s2c_re;  assign s3_in_im = s2c_im;  assign s3_in_valid = s2c_valid[0];

    //----------------------------------------------------------------------
    // Debug signals from Stage-III
    //----------------------------------------------------------------------
    wire [W*16-1:0] s3_pe_re;
    wire [W*16-1:0] s3_pe_im;
    wire            s3_pe_valid;

    // =========================================================================
    // Stage-III datapath (paper Fig.4, Fig.5, Fig.8, Sec.III-A.3)
    //   No CORDIC after Stage-III BS16 (final stage, per Fig.4).
    // =========================================================================
    wire [W*16-1:0] s3_y_re, s3_y_im;
    wire            s3_valid_out;

    stage3_datapath #(.W(W)) u_s3 (
        .clk           (gclk_m3),
        .clk_c         (gclk_b3),
        .rst           (rst),
        .cycle_cnt     (cycle_cnt_s3),
        .set_sel       (set_sel_s3),
        .wr_en         (wr_en_s3),
        .valid_in      (valid_in_s3),
        .x_re_flat     (s3_in_re),
        .x_im_flat     (s3_in_im),
        .pe_in_re_flat (s3_pe_re),
        .pe_in_im_flat (s3_pe_im),
        .pe_in_valid   (s3_pe_valid),
        .y_re_flat     (s3_y_re),
        .y_im_flat     (s3_y_im),
        .valid_out     (s3_valid_out)
    );

    // =========================================================================
    // Stage-III output padding: 4 (raw) + 9 (pad) = Cpipelined_stgIII = 13,
    // so the FIRST output sample appears exactly at mc = MC_EOP = 605 (Eq.21)
    // and the last at mc = 860 (605 + N/P - 1, paper Sec.V).
    // =========================================================================
    wire [W*16-1:0] out_re, out_im;
    wire            out_valid;

    // FORK-B: pad removed; Stage-III output wired direct to output conjugate MUX.
    assign out_re = s3_y_re;  assign out_im = s3_y_im;  assign out_valid = s3_valid_out;

    // =========================================================================
    // Output Conjugate and 2x1 MUX (paper Sec.II, Eq.2, Fig.4)
    //   FFT  (fft_ifft=0): output Stage-III result as-is
    //   IFFT (fft_ifft=1, EN): conjugate Stage-III result
    //   Select = fft_ifft AND EN (paper Sec.III: "logical AND");
    //   (en | eop) so the select is active from the first output cycle.
    // =========================================================================
    wire [W*16-1:0] out_im_conj;
    genvar co;
    generate
        for (co = 0; co < 16; co = co + 1) begin : CONJ_OUT
            assign out_im_conj[W*(15-co) +: W] = -$signed(out_im[W*(15-co) +: W]);
        end
    endgenerate

    wire out_sel = fft_ifft & (en | eop);
    assign y_re_flat = out_re;
    assign y_im_flat = out_sel ? out_im_conj : out_im;

    assign valid_out = out_valid;

endmodule

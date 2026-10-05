`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 24.06.2026 12:26:44
// Design Name: 
// Module Name: bs16_unit
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

// =============================================================================
// Module : bs16_unit
// Purpose: Complete 16-point butterfly unit BS_16 (Fig.9, Eq.11-13)
//          Two cascaded radix-4 stages + 16 CSD intra-twiddles.
//          3 pipeline registers ? 3 CC latency.
//
// Architecture (Eq.11-13):
//   Stage-I:  4 � radix-4 butterfly (indexed by s1=0..3)
//             s1-th BF: inputs x[s1], x[s1+4], x[s1+8], x[s1+12]
//             Output l2 multiplied by W16^(s1*l2)
//   Stage-II: 4 � radix-4 butterfly (indexed by l2=0..3)
//             l2-th BF: inputs from all s1 at fixed l2
//             Output at bin k1 = 4*l1 + l2
//
// Twiddle table (s1*l2 mod 16):
//   l2:   0  1  2  3
//   s1=0: 0  0  0  0   (all trivial W16^0=1)
//   s1=1: 0  1  2  3
//   s1=2: 0  2  4  6
//   s1=3: 0  3  6  9
//
// KEY FIX: All 16 csd_mult_w16 instances use hardcoded k parameter
//          (not computed from genvar expression) to ensure correct
//          constant propagation in Verilog-2001 / Vivado 2022.2.
//
// Port convention: flat packed bus [W*16-1:0]
//   sample 0 at bits [W*16-1:W*15], sample 15 at bits [W-1:0]
//
// Compatible with: Verilog-2001, Vivado 2022.2, Cadence Genus/Innovus
// =============================================================================
module bs16_unit #(
    parameter W = 16
)(
    input  wire             clk,
    input  wire             rst,
    input  wire             valid_in,
    input  wire [W*16-1:0]  x_re_flat,   // 16 � W-bit real parts
    input  wire [W*16-1:0]  x_im_flat,   // 16 � W-bit imaginary parts
    output reg  [W*16-1:0]  y_re_flat,
    output reg  [W*16-1:0]  y_im_flat,
    output reg              valid_out
);

    // =========================================================================
    // Unpack flat input buses into individual signals
    // Sample i: x_re_flat[W*(15-i) +: W]
    // =========================================================================
    wire signed [W-1:0] xr [0:15];
    wire signed [W-1:0] xi [0:15];

    genvar gu;
    generate
        for (gu = 0; gu < 16; gu = gu + 1) begin : UNPACK
            assign xr[gu] = x_re_flat[W*(15-gu) +: W];
            assign xi[gu] = x_im_flat[W*(15-gu) +: W];
        end
    endgenerate

    // =========================================================================
    // Stage-I: 4 radix-4 butterflies (s1 = 0,1,2,3)
    // s1-th butterfly: x[s1], x[s1+4], x[s1+8], x[s1+12]
    // =========================================================================
    wire signed [W-1:0] b1r [0:3][0:3];  // b1r[s1][l2]
    wire signed [W-1:0] b1i [0:3][0:3];

    // s1=0
    radix4_butterfly #(.W(W)) u_bf1_s0 (
        .x0_re(xr[0]),  .x0_im(xi[0]),
        .x1_re(xr[4]),  .x1_im(xi[4]),
        .x2_re(xr[8]),  .x2_im(xi[8]),
        .x3_re(xr[12]), .x3_im(xi[12]),
        .y0_re(b1r[0][0]), .y0_im(b1i[0][0]),
        .y1_re(b1r[0][1]), .y1_im(b1i[0][1]),
        .y2_re(b1r[0][2]), .y2_im(b1i[0][2]),
        .y3_re(b1r[0][3]), .y3_im(b1i[0][3])
    );
    // s1=1
    radix4_butterfly #(.W(W)) u_bf1_s1 (
        .x0_re(xr[1]),  .x0_im(xi[1]),
        .x1_re(xr[5]),  .x1_im(xi[5]),
        .x2_re(xr[9]),  .x2_im(xi[9]),
        .x3_re(xr[13]), .x3_im(xi[13]),
        .y0_re(b1r[1][0]), .y0_im(b1i[1][0]),
        .y1_re(b1r[1][1]), .y1_im(b1i[1][1]),
        .y2_re(b1r[1][2]), .y2_im(b1i[1][2]),
        .y3_re(b1r[1][3]), .y3_im(b1i[1][3])
    );
    // s1=2
    radix4_butterfly #(.W(W)) u_bf1_s2 (
        .x0_re(xr[2]),  .x0_im(xi[2]),
        .x1_re(xr[6]),  .x1_im(xi[6]),
        .x2_re(xr[10]), .x2_im(xi[10]),
        .x3_re(xr[14]), .x3_im(xi[14]),
        .y0_re(b1r[2][0]), .y0_im(b1i[2][0]),
        .y1_re(b1r[2][1]), .y1_im(b1i[2][1]),
        .y2_re(b1r[2][2]), .y2_im(b1i[2][2]),
        .y3_re(b1r[2][3]), .y3_im(b1i[2][3])
    );
    // s1=3
    radix4_butterfly #(.W(W)) u_bf1_s3 (
        .x0_re(xr[3]),  .x0_im(xi[3]),
        .x1_re(xr[7]),  .x1_im(xi[7]),
        .x2_re(xr[11]), .x2_im(xi[11]),
        .x3_re(xr[15]), .x3_im(xi[15]),
        .y0_re(b1r[3][0]), .y0_im(b1i[3][0]),
        .y1_re(b1r[3][1]), .y1_im(b1i[3][1]),
        .y2_re(b1r[3][2]), .y2_im(b1i[3][2]),
        .y3_re(b1r[3][3]), .y3_im(b1i[3][3])
    );

    // =========================================================================
    // Pipeline Register 1 - latch Stage-I butterfly outputs
    // =========================================================================
    reg signed [W-1:0] r1r [0:3][0:3];
    reg signed [W-1:0] r1i [0:3][0:3];
    reg                v1;
    integer a1, b1_i;

    always @(posedge clk) begin : REG1
        if (rst) begin
            v1 <= 1'b0;
            for (a1=0; a1<4; a1=a1+1)
                for (b1_i=0; b1_i<4; b1_i=b1_i+1) begin
                    r1r[a1][b1_i] <= {W{1'b0}};
                    r1i[a1][b1_i] <= {W{1'b0}};
                end
        end else begin
            v1 <= valid_in;
            for (a1=0; a1<4; a1=a1+1)
                for (b1_i=0; b1_i<4; b1_i=b1_i+1) begin
                    r1r[a1][b1_i] <= b1r[a1][b1_i];
                    r1i[a1][b1_i] <= b1i[a1][b1_i];
                end
        end
    end

    // =========================================================================
    // CSD Intra-Twiddle Multipliers: W16^(s1*l2)
    // All 16 instances with HARDCODED k to avoid Verilog-2001 genvar issues.
    //
    // Twiddle table - k = s1*l2 mod 16:
    //   s1\l2  0  1  2  3
    //      0:  0  0  0  0
    //      1:  0  1  2  3
    //      2:  0  2  4  6
    //      3:  0  3  6  9
    //
    // Instances named: u_tw_s{s1}_l{l2}
    // =========================================================================
    wire signed [W-1:0] twr [0:3][0:3];  // twr[s1][l2]
    wire signed [W-1:0] twi [0:3][0:3];

    // --- s1=0: k = 0*l2 = 0 for all l2 (pass-through, but instantiated) ---
    csd_mult_w16 #(.W(W)) u_tw_s0_l0 (.x_re(r1r[0][0]),.x_im(r1i[0][0]),.k(4'd0),.y_re(twr[0][0]),.y_im(twi[0][0]));
    csd_mult_w16 #(.W(W)) u_tw_s0_l1 (.x_re(r1r[0][1]),.x_im(r1i[0][1]),.k(4'd0),.y_re(twr[0][1]),.y_im(twi[0][1]));
    csd_mult_w16 #(.W(W)) u_tw_s0_l2 (.x_re(r1r[0][2]),.x_im(r1i[0][2]),.k(4'd0),.y_re(twr[0][2]),.y_im(twi[0][2]));
    csd_mult_w16 #(.W(W)) u_tw_s0_l3 (.x_re(r1r[0][3]),.x_im(r1i[0][3]),.k(4'd0),.y_re(twr[0][3]),.y_im(twi[0][3]));

    // --- s1=1: k = 1*l2 = 0,1,2,3 ---
    csd_mult_w16 #(.W(W)) u_tw_s1_l0 (.x_re(r1r[1][0]),.x_im(r1i[1][0]),.k(4'd0),.y_re(twr[1][0]),.y_im(twi[1][0]));
    csd_mult_w16 #(.W(W)) u_tw_s1_l1 (.x_re(r1r[1][1]),.x_im(r1i[1][1]),.k(4'd1),.y_re(twr[1][1]),.y_im(twi[1][1]));
    csd_mult_w16 #(.W(W)) u_tw_s1_l2 (.x_re(r1r[1][2]),.x_im(r1i[1][2]),.k(4'd2),.y_re(twr[1][2]),.y_im(twi[1][2]));
    csd_mult_w16 #(.W(W)) u_tw_s1_l3 (.x_re(r1r[1][3]),.x_im(r1i[1][3]),.k(4'd3),.y_re(twr[1][3]),.y_im(twi[1][3]));

    // --- s1=2: k = 2*l2 = 0,2,4,6 ---
    csd_mult_w16 #(.W(W)) u_tw_s2_l0 (.x_re(r1r[2][0]),.x_im(r1i[2][0]),.k(4'd0),.y_re(twr[2][0]),.y_im(twi[2][0]));
    csd_mult_w16 #(.W(W)) u_tw_s2_l1 (.x_re(r1r[2][1]),.x_im(r1i[2][1]),.k(4'd2),.y_re(twr[2][1]),.y_im(twi[2][1]));
    csd_mult_w16 #(.W(W)) u_tw_s2_l2 (.x_re(r1r[2][2]),.x_im(r1i[2][2]),.k(4'd4),.y_re(twr[2][2]),.y_im(twi[2][2]));
    csd_mult_w16 #(.W(W)) u_tw_s2_l3 (.x_re(r1r[2][3]),.x_im(r1i[2][3]),.k(4'd6),.y_re(twr[2][3]),.y_im(twi[2][3]));

    // --- s1=3: k = 3*l2 = 0,3,6,9 ---
    csd_mult_w16 #(.W(W)) u_tw_s3_l0 (.x_re(r1r[3][0]),.x_im(r1i[3][0]),.k(4'd0),.y_re(twr[3][0]),.y_im(twi[3][0]));
    csd_mult_w16 #(.W(W)) u_tw_s3_l1 (.x_re(r1r[3][1]),.x_im(r1i[3][1]),.k(4'd3),.y_re(twr[3][1]),.y_im(twi[3][1]));
    csd_mult_w16 #(.W(W)) u_tw_s3_l2 (.x_re(r1r[3][2]),.x_im(r1i[3][2]),.k(4'd6),.y_re(twr[3][2]),.y_im(twi[3][2]));
    csd_mult_w16 #(.W(W)) u_tw_s3_l3 (.x_re(r1r[3][3]),.x_im(r1i[3][3]),.k(4'd9),.y_re(twr[3][3]),.y_im(twi[3][3]));

    // =========================================================================
    // Pipeline Register 2 - latch twiddle outputs, TRANSPOSE to [l2][s1]
    // r2[l2][s1] = tw[s1][l2]  (feeds Stage-II: l2-th BF gets all s1 at fixed l2)
    // =========================================================================
    reg signed [W-1:0] r2r [0:3][0:3];  // r2r[l2][s1]
    reg signed [W-1:0] r2i [0:3][0:3];
    reg                v2;
    integer a2, b2;

    always @(posedge clk) begin : REG2
        if (rst) begin
            v2 <= 1'b0;
            for (a2=0; a2<4; a2=a2+1)
                for (b2=0; b2<4; b2=b2+1) begin
                    r2r[a2][b2] <= {W{1'b0}};
                    r2i[a2][b2] <= {W{1'b0}};
                end
        end else begin
            v2 <= v1;
            // Transpose: r2[l2][s1] = twr[s1][l2]
            for (a2=0; a2<4; a2=a2+1)   // a2 = l2
                for (b2=0; b2<4; b2=b2+1) begin  // b2 = s1
                    r2r[a2][b2] <= twr[b2][a2];
                    r2i[a2][b2] <= twi[b2][a2];
                end
        end
    end

    // =========================================================================
    // Stage-II: 4 radix-4 butterflies (l2 = 0,1,2,3)
    // l2-th BF inputs: r2[l2][s1] for s1=0..3
    // Output at bin k1 = 4*l1 + l2
    // =========================================================================
    wire signed [W-1:0] b2r [0:3][0:3];  // b2r[l2][l1]
    wire signed [W-1:0] b2i [0:3][0:3];

    // l2=0
    radix4_butterfly #(.W(W)) u_bf2_l0 (
        .x0_re(r2r[0][0]), .x0_im(r2i[0][0]),
        .x1_re(r2r[0][1]), .x1_im(r2i[0][1]),
        .x2_re(r2r[0][2]), .x2_im(r2i[0][2]),
        .x3_re(r2r[0][3]), .x3_im(r2i[0][3]),
        .y0_re(b2r[0][0]), .y0_im(b2i[0][0]),
        .y1_re(b2r[0][1]), .y1_im(b2i[0][1]),
        .y2_re(b2r[0][2]), .y2_im(b2i[0][2]),
        .y3_re(b2r[0][3]), .y3_im(b2i[0][3])
    );
    // l2=1
    radix4_butterfly #(.W(W)) u_bf2_l1 (
        .x0_re(r2r[1][0]), .x0_im(r2i[1][0]),
        .x1_re(r2r[1][1]), .x1_im(r2i[1][1]),
        .x2_re(r2r[1][2]), .x2_im(r2i[1][2]),
        .x3_re(r2r[1][3]), .x3_im(r2i[1][3]),
        .y0_re(b2r[1][0]), .y0_im(b2i[1][0]),
        .y1_re(b2r[1][1]), .y1_im(b2i[1][1]),
        .y2_re(b2r[1][2]), .y2_im(b2i[1][2]),
        .y3_re(b2r[1][3]), .y3_im(b2i[1][3])
    );
    // l2=2
    radix4_butterfly #(.W(W)) u_bf2_l2 (
        .x0_re(r2r[2][0]), .x0_im(r2i[2][0]),
        .x1_re(r2r[2][1]), .x1_im(r2i[2][1]),
        .x2_re(r2r[2][2]), .x2_im(r2i[2][2]),
        .x3_re(r2r[2][3]), .x3_im(r2i[2][3]),
        .y0_re(b2r[2][0]), .y0_im(b2i[2][0]),
        .y1_re(b2r[2][1]), .y1_im(b2i[2][1]),
        .y2_re(b2r[2][2]), .y2_im(b2i[2][2]),
        .y3_re(b2r[2][3]), .y3_im(b2i[2][3])
    );
    // l2=3
    radix4_butterfly #(.W(W)) u_bf2_l3 (
        .x0_re(r2r[3][0]), .x0_im(r2i[3][0]),
        .x1_re(r2r[3][1]), .x1_im(r2i[3][1]),
        .x2_re(r2r[3][2]), .x2_im(r2i[3][2]),
        .x3_re(r2r[3][3]), .x3_im(r2i[3][3]),
        .y0_re(b2r[3][0]), .y0_im(b2i[3][0]),
        .y1_re(b2r[3][1]), .y1_im(b2i[3][1]),
        .y2_re(b2r[3][2]), .y2_im(b2i[3][2]),
        .y3_re(b2r[3][3]), .y3_im(b2i[3][3])
    );

    // =========================================================================
    // Pipeline Register 3 - output register
    // k1 = 4*l1 + l2  ?  pack into flat output bus
    // =========================================================================
    integer a3, b3;

    always @(posedge clk) begin : REG3
        if (rst) begin
            valid_out <= 1'b0;
            y_re_flat <= {(W*16){1'b0}};
            y_im_flat <= {(W*16){1'b0}};
        end else begin
            valid_out <= v2;
            // a3=l2 (0..3), b3=l1 (0..3)
            // k1 = 4*l1 + l2 = 4*b3 + a3
            // pack: sample at index k1, MSB-first ? bit position W*(15-k1)
            for (a3=0; a3<4; a3=a3+1)
                for (b3=0; b3<4; b3=b3+1) begin
                    y_re_flat[W*(15-(4*b3+a3)) +: W] <= b2r[a3][b3];
                    y_im_flat[W*(15-(4*b3+a3)) +: W] <= b2i[a3][b3];
                end
        end
    end

endmodule

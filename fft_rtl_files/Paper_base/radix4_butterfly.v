`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 24.06.2026 11:29:14
// Design Name: 
// Module Name: radix4_butterfly
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
// Module : radix4_butterfly
// Purpose: 4-point DFT butterfly (BS_4)
//          W4 = exp(-j*pi/2) = -j  (trivial rotation, no multipliers)
//          Signal flow:
//            A = x0 + x2,  B = x0 - x2
//            C = x1 + x3,  D = (x1 - x3) * (-j) = (im(x1-x3), -re(x1-x3))
//            y0=A+C, y1=B+D, y2=A-C, y3=B-D
// Latency : Combinational (0 clock cycles)
// =============================================================================
//module radix4_butterfly #(
//    parameter W = 16   // word width (signed)
//)(
//    // Inputs: 4 complex samples
//    input  signed [W-1:0] x0_re, x0_im,
//    input  signed [W-1:0] x1_re, x1_im,
//    input  signed [W-1:0] x2_re, x2_im,
//    input  signed [W-1:0] x3_re, x3_im,
//    // Outputs: 4 complex results
//    output signed [W-1:0] y0_re, y0_im,
//    output signed [W-1:0] y1_re, y1_im,
//    output signed [W-1:0] y2_re, y2_im,
//    output signed [W-1:0] y3_re, y3_im
//);
//    // Intermediate sums (W+1 bits to prevent overflow before truncation)
//    wire signed [W:0] A_re, A_im;
//    wire signed [W:0] B_re, B_im;
//    wire signed [W:0] C_re, C_im;
//    wire signed [W:0] D_re, D_im;

//    // Stage 1: twiddle-free additions
//    assign A_re = x0_re + x2_re;  assign A_im = x0_im + x2_im;
//    assign B_re = x0_re - x2_re;  assign B_im = x0_im - x2_im;
//    assign C_re = x1_re + x3_re;  assign C_im = x1_im + x3_im;

//    // D = (x1-x3) * (-j): negate imaginary, swap
//    // (-j) * (a+jb) = b - ja  ?  re = im(diff), im = -re(diff)
//    assign D_re =  (x1_im - x3_im);
//    assign D_im = -(x1_re - x3_re);

//    // Stage 2: final combinations (truncate to W bits)
//    assign y0_re = (A_re + C_re) >>> 1;  // arithmetic right shift to keep range
//    assign y0_im = (A_im + C_im) >>> 1;
//    assign y1_re = (B_re + D_re) >>> 1;
//    assign y1_im = (B_im + D_im) >>> 1;
//    assign y2_re = (A_re - C_re) >>> 1;
//    assign y2_im = (A_im - C_im) >>> 1;
//    assign y3_re = (B_re - D_re) >>> 1;
//    assign y3_im = (B_im - D_im) >>> 1;

//endmodule
// =============================================================================
// Module : radix4_butterfly
// Purpose: 4-point DFT butterfly derived from the DFT matrix
//
//   DFT4 twiddle: W4 = exp(-j*2*pi/4) = -j
//
//   Signal flow (Cooley-Tukey DIF, from DFT matrix expansion):
//     A = x0 + x2,  B = x0 - x2
//     C = x1 + x3,  D = x1 - x3
//
//     X[0] = A + C
//     X[1] = B - j*D  =>  re = B_re + D_im,  im = B_im - D_re
//     X[2] = A - C
//     X[3] = B + j*D  =>  re = B_re - D_im,  im = B_im + D_re
//
//   Output scaled by 1/2 (arithmetic right-shift by 1) to prevent overflow
//   across two cascaded stages in BS16. Both stages apply /2, giving /4 total.
//   MATLAB reference uses the same scaling so results match numerically.
//
// Arithmetic strategy (Vivado 2022.2 / Verilog-2001 safe):
//   ALL intermediate nodes declared at W+3 bits to prevent any overflow
//   before the final truncating right-shift. Explicit $signed() casts
//   ensure sign-aware arithmetic at every step. Final outputs are W bits
//   taken as the signed arithmetic right-shift result [W:1] of the W+3 sum.
//
// Latency: Combinational
// Compatible: Verilog-2001, Vivado 2022.2, Cadence Genus/Innovus UMC 65nm
// =============================================================================
module radix4_butterfly #(
    parameter W = 16
)(
    input  wire signed [W-1:0] x0_re, x0_im,
    input  wire signed [W-1:0] x1_re, x1_im,
    input  wire signed [W-1:0] x2_re, x2_im,
    input  wire signed [W-1:0] x3_re, x3_im,
    output wire signed [W-1:0] y0_re, y0_im,
    output wire signed [W-1:0] y1_re, y1_im,
    output wire signed [W-1:0] y2_re, y2_im,
    output wire signed [W-1:0] y3_re, y3_im
);
    // ?? Sign-extend all inputs to W+3 bits ???????????????????????????????
    // W+3 bits: 1 sign + 1 guard for each add level (up to 3 adds deep)
    // This guarantees no overflow for any combination of W-bit inputs.
    wire signed [W+2:0] X0r = {{3{x0_re[W-1]}}, x0_re[W-1:0]};
    wire signed [W+2:0] X0i = {{3{x0_im[W-1]}}, x0_im[W-1:0]};
    wire signed [W+2:0] X1r = {{3{x1_re[W-1]}}, x1_re[W-1:0]};
    wire signed [W+2:0] X1i = {{3{x1_im[W-1]}}, x1_im[W-1:0]};
    wire signed [W+2:0] X2r = {{3{x2_re[W-1]}}, x2_re[W-1:0]};
    wire signed [W+2:0] X2i = {{3{x2_im[W-1]}}, x2_im[W-1:0]};
    wire signed [W+2:0] X3r = {{3{x3_re[W-1]}}, x3_re[W-1:0]};
    wire signed [W+2:0] X3i = {{3{x3_im[W-1]}}, x3_im[W-1:0]};

    // ?? Stage 1: four pairwise sums/differences ???????????????????????????
    wire signed [W+2:0] Ar = X0r + X2r;
    wire signed [W+2:0] Ai = X0i + X2i;
    wire signed [W+2:0] Br = X0r - X2r;
    wire signed [W+2:0] Bi = X0i - X2i;
    wire signed [W+2:0] Cr = X1r + X3r;
    wire signed [W+2:0] Ci = X1i + X3i;
    wire signed [W+2:0] Dr = X1r - X3r;   // D = x1 - x3
    wire signed [W+2:0] Di = X1i - X3i;

    // ?? Stage 2: combine with j-rotation ?????????????????????????????????
    // X[0] = A + C
    wire signed [W+2:0] Y0r_full = Ar + Cr;
    wire signed [W+2:0] Y0i_full = Ai + Ci;
    // X[1] = B - j*D  =>  re = Br + Di,  im = Bi - Dr
    wire signed [W+2:0] Y1r_full = Br + Di;
    wire signed [W+2:0] Y1i_full = Bi - Dr;
    // X[2] = A - C
    wire signed [W+2:0] Y2r_full = Ar - Cr;
    wire signed [W+2:0] Y2i_full = Ai - Ci;
    // X[3] = B + j*D  =>  re = Br - Di,  im = Bi + Dr
    wire signed [W+2:0] Y3r_full = Br - Di;
    wire signed [W+2:0] Y3i_full = Bi + Dr;

    // ?? Arithmetic right-shift by 1: output W bits ???????????????????????
    // Y_full is W+3 bits (W+2:0). After >>1, valid value in bits W+1:0.
    // Take bits [W:1] which gives a W-bit arithmetic right shift result.
    // This is equivalent to signed division by 2 with round-toward-minus-inf.
//    assign y0_re = Y0r_full[W:1];
//    assign y0_im = Y0i_full[W:1];
//    assign y1_re = Y1r_full[W:1];
//    assign y1_im = Y1i_full[W:1];
//    assign y2_re = Y2r_full[W:1];
//    assign y2_im = Y2i_full[W:1];
//    assign y3_re = Y3r_full[W:1];
//    assign y3_im = Y3i_full[W:1];
    assign y0_re = $signed(Y0r_full) >>> 1;
    assign y0_im = $signed(Y0i_full) >>> 1;
    
    assign y1_re = $signed(Y1r_full) >>> 1;
    assign y1_im = $signed(Y1i_full) >>> 1;
    
    assign y2_re = $signed(Y2r_full) >>> 1;
    assign y2_im = $signed(Y2i_full) >>> 1;
    
    assign y3_re = $signed(Y3r_full) >>> 1;
    assign y3_im = $signed(Y3i_full) >>> 1;

endmodule

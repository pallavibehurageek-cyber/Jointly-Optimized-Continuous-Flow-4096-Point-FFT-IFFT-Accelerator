`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 24.06.2026 12:09:13
// Design Name: 
// Module Name: cordic_rotator
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
// Module : cordic_rotator
// Purpose: 10-stage pipelined CORDIC rotation (Eq.15-19 of paper)
//          Rotates complex input (x + jy) by angle theta.
//          Quadrant pre-rotation handles full [0, 2pi) range.
//          Scale factor K = prod(1/sqrt(1+2^{-2i})) ~ 0.6073 applied at output.
//
// Pipeline: 10 CC latency (one register per stage)
// alpha values in Q2.18 (WZ=20): alpha_i = round(atan(2^{-i})*2^18/pi*2)
// =============================================================================
module cordic_rotator #(
    parameter W  = 16,   // data width
    parameter WZ = 20    // phase accumulator width
)(
    input  wire             clk,
    input  wire             rst,
    input  wire             valid_in,
    input  signed [W-1:0]  x_in,
    input  signed [W-1:0]  y_in,
    input  signed [WZ-1:0] theta_in,   // Q2.18 format (pi -> 2^18)
    output reg              valid_out,
    output reg  signed [W-1:0]  x_out,
    output reg  signed [W-1:0]  y_out
);
    // alpha_i = round(atan(2^{-i}) / (pi/2) * 2^(WZ-2))
    // pi/2 maps to 2^(WZ-2) = 2^18 in Q2.18
    localparam signed [WZ-1:0] ALPHA0 = 20'd131072; // atan(1)     = pi/4
    localparam signed [WZ-1:0] ALPHA1 = 20'd77376;  // atan(0.5)
    localparam signed [WZ-1:0] ALPHA2 = 20'd40884;  // atan(0.25)
    localparam signed [WZ-1:0] ALPHA3 = 20'd20588;  // atan(0.125)
    localparam signed [WZ-1:0] ALPHA4 = 20'd10313;  // atan(0.0625)
    localparam signed [WZ-1:0] ALPHA5 = 20'd5157;
    localparam signed [WZ-1:0] ALPHA6 = 20'd2579;
    localparam signed [WZ-1:0] ALPHA7 = 20'd1289;
    localparam signed [WZ-1:0] ALPHA8 = 20'd645;
    localparam signed [WZ-1:0] ALPHA9 = 20'd322;

    // K_inv = 1/K = 1.6467 in Q1.15 = round(1.6467 * 32768) = 53979
    //localparam signed [W-1:0] K_INV = 16'sd26779; // K in Q1.15 = 0.6073*32768
    localparam signed [W-1:0] K_INV = 16'sd19898;

    // ?? Quadrant pre-rotation (combinational, before pipeline) ???????????
    // Reduce theta to [0, pi/2) by rotating input by multiples of pi/2
    // pi/2 in Q2.18 = 2^18 = 262144
    wire signed [WZ-1:0] theta_norm;
    wire signed [W-1:0]  x_pre, y_pre;
    wire [1:0] quad;

    assign quad = theta_in[WZ-1:WZ-2];  // top 2 bits = quadrant

    // Pre-rotate: multiply by j^quad = (re,im) -> (-im,re) q times
    // quad=0: no rotation; quad=1: (*j); quad=2: (*j^2=-1); quad=3: (*j^3=-j)
    reg signed [W-1:0] xp, yp;
    reg signed [WZ-1:0] tn;
    always @(*) begin
        case (quad)
            2'b00: begin xp= x_in; yp= y_in; tn=theta_in; end
            2'b01: begin xp=-y_in; yp= x_in; tn=theta_in-20'sd262144; end
            2'b10: begin xp=-x_in; yp=-y_in; tn=theta_in-20'sd524288; end
            2'b11: begin xp= y_in; yp=-x_in; tn=theta_in-20'sd786432; end
            default: begin xp=x_in; yp=y_in; tn=theta_in; end
        endcase
    end

    // ?? 10 pipeline stages ????????????????????????????????????????????????
    reg signed [W-1:0]  xs [0:10];
    reg signed [W-1:0]  ys [0:10];
    reg signed [WZ-1:0] zs [0:10];
    reg                 vs [0:10];

    wire signed [WZ-1:0] alphas [0:9];
    assign alphas[0]=ALPHA0; assign alphas[1]=ALPHA1;
    assign alphas[2]=ALPHA2; assign alphas[3]=ALPHA3;
    assign alphas[4]=ALPHA4; assign alphas[5]=ALPHA5;
    assign alphas[6]=ALPHA6; assign alphas[7]=ALPHA7;
    assign alphas[8]=ALPHA8; assign alphas[9]=ALPHA9;

    // Stage 0 input
    always @(posedge clk) begin
        if (rst) begin
            xs[0]<=0; ys[0]<=0; zs[0]<=0; vs[0]<=0;
        end else begin
            xs[0]<=xp; ys[0]<=yp; zs[0]<=tn; vs[0]<=valid_in;
        end
    end

    genvar i;
    generate
        for (i = 0; i < 10; i = i + 1) begin : CORDIC_PIPE
            wire signed [W-1:0]  xw, yw;
            wire signed [WZ-1:0] zw;
            wire gam = zs[i][WZ-1];
            // micro-rotation (combinational)
            assign xw = gam ? xs[i]+(ys[i]>>>i) : xs[i]-(ys[i]>>>i);
            assign yw = gam ? ys[i]-(xs[i]>>>i) : ys[i]+(xs[i]>>>i);
            assign zw = gam ? zs[i]+alphas[i]   : zs[i]-alphas[i];
            // pipeline register
            always @(posedge clk) begin
                if (rst) begin
                    xs[i+1]<=0; ys[i+1]<=0; zs[i+1]<=0; vs[i+1]<=0;
                end else begin
                    xs[i+1]<=xw; ys[i+1]<=yw; zs[i+1]<=zw; vs[i+1]<=vs[i];
                end
            end
        end
    endgenerate

    // ?? Apply scale factor K (multiply by K_INV/32768) ???????????????????
    wire signed [2*W-1:0] x_scaled = xs[10] * K_INV;
    wire signed [2*W-1:0] y_scaled = ys[10] * K_INV;

    always @(posedge clk) begin
        if (rst) begin
            x_out <= 0; y_out <= 0; valid_out <= 0;
        end else begin
            x_out     <= x_scaled[2*W-2:W-1];  // Q1.15 -> W bits
            y_out     <= y_scaled[2*W-2:W-1];
            valid_out <= vs[10];
        end
    end
endmodule

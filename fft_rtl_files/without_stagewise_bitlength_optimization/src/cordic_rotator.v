`timescale 1ns / 1ps
// =============================================================================
// Module : cordic_rotator  (REDUCED-CYCLE: 6 coarse micro-rotations + 1 fine)
//   Novelty #3. Rotates (x + jy) by theta (Q2.18, pi/2 <-> 2^18), ROM-free.
//   6 coarse micro-rotations shrink the residual; one small-angle LINEAR
//   correction stage nulls it to 2nd order:
//       theta_q15 = (z6 * C_PI16) >>> 16
//       xf = x6 - (y6*theta_q15) >>> 15 ; yf = y6 + (x6*theta_q15) >>> 15
//   then scale by K = 1/A6. No ROM, no stored twiddles.
//   Latency: 1 + 6 + 1 + 1 = 9 CC (was 12). SQNR over Eq.6/Eq.8 sets ~72 dB.
//   Alphas use exact atan(2^-i); fine stage needs z to track the true residual.
// =============================================================================
module cordic_rotator #(
    parameter W  = 16,
    parameter WZ = 20
)(
    input  wire             clk,
    input  wire             rst,
    input  wire             valid_in,
    input  signed [W-1:0]   x_in,
    input  signed [W-1:0]   y_in,
    input  signed [WZ-1:0]  theta_in,
    output reg              valid_out,
    output reg  signed [W-1:0]  x_out,
    output reg  signed [W-1:0]  y_out
);
    localparam signed [WZ-1:0] ALPHA0 = 20'sd131072;
    localparam signed [WZ-1:0] ALPHA1 = 20'sd77376;
    localparam signed [WZ-1:0] ALPHA2 = 20'sd40884;
    localparam signed [WZ-1:0] ALPHA3 = 20'sd20753;  // corrected
    localparam signed [WZ-1:0] ALPHA4 = 20'sd10417;  // corrected
    localparam signed [WZ-1:0] ALPHA5 = 20'sd5213;   // corrected
    localparam signed [15:0]   C_PI16 = 16'sd12868;  // pi/16
    localparam signed [W-1:0]  K_INV  = 16'sd19902;  // 1/A6, Q1.15

    reg signed [W-1:0]  xp, yp;
    reg signed [WZ-1:0] tn;
    wire [1:0] quad = theta_in[WZ-1:WZ-2];
    always @(*) begin
        case (quad)
            2'b00: begin xp= x_in; yp= y_in; tn=theta_in;              end
            2'b01: begin xp=-y_in; yp= x_in; tn=theta_in-20'sd262144;  end
            2'b10: begin xp=-x_in; yp=-y_in; tn=theta_in-20'sd524288;  end
            2'b11: begin xp= y_in; yp=-x_in; tn=theta_in-20'sd786432;  end
            default:begin xp= x_in; yp= y_in; tn=theta_in;             end
        endcase
    end

    wire signed [WZ-1:0] alphas [0:5];
    assign alphas[0]=ALPHA0; assign alphas[1]=ALPHA1; assign alphas[2]=ALPHA2;
    assign alphas[3]=ALPHA3; assign alphas[4]=ALPHA4; assign alphas[5]=ALPHA5;

    reg signed [W-1:0]  xs [0:6];
    reg signed [W-1:0]  ys [0:6];
    reg signed [WZ-1:0] zs [0:6];
    reg                 vs [0:6];

    always @(posedge clk) begin
        if (rst) begin xs[0]<=0; ys[0]<=0; zs[0]<=0; vs[0]<=0; end
        else     begin xs[0]<=xp; ys[0]<=yp; zs[0]<=tn; vs[0]<=valid_in; end
    end

    genvar i;
    generate
        for (i=0;i<6;i=i+1) begin : CORDIC_COARSE
            wire gam = zs[i][WZ-1];
            wire signed [W-1:0]  xw = gam ? xs[i]+(ys[i]>>>i) : xs[i]-(ys[i]>>>i);
            wire signed [W-1:0]  yw = gam ? ys[i]-(xs[i]>>>i) : ys[i]+(xs[i]>>>i);
            wire signed [WZ-1:0] zw = gam ? zs[i]+alphas[i]   : zs[i]-alphas[i];
            always @(posedge clk) begin
                if (rst) begin xs[i+1]<=0; ys[i+1]<=0; zs[i+1]<=0; vs[i+1]<=0; end
                else begin xs[i+1]<=xw; ys[i+1]<=yw; zs[i+1]<=zw; vs[i+1]<=vs[i]; end
            end
        end
    endgenerate

    // Fine linear correction (1 stage)
    wire signed [35:0] zc   = zs[6] * C_PI16;
    wire signed [15:0] tq15 = zc >>> 16;
    wire signed [31:0] cx   = ys[6] * tq15;
    wire signed [31:0] cy   = xs[6] * tq15;
    reg signed [W-1:0] xf, yf;
    reg                vf;
    always @(posedge clk) begin
        if (rst) begin xf<=0; yf<=0; vf<=0; end
        else begin
            xf <= xs[6] - (cx >>> 15);
            yf <= ys[6] + (cy >>> 15);
            vf <= vs[6];
        end
    end

    // Output scale by K = 1/A6
    wire signed [2*W-1:0] x_scaled = xf * K_INV;
    wire signed [2*W-1:0] y_scaled = yf * K_INV;
    always @(posedge clk) begin
        if (rst) begin x_out<=0; y_out<=0; valid_out<=0; end
        else begin
            x_out     <= x_scaled[2*W-2:W-1];
            y_out     <= y_scaled[2*W-2:W-1];
            valid_out <= vf;
        end
    end
endmodule

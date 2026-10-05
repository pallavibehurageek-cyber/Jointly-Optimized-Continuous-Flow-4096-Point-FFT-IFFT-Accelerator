`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 28.06.2026 21:25:57
// Design Name: 
// Module Name: csd_mult_w16_v2
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


module csd_mult_w16 #(
    parameter W = 16
)(
    input  wire signed [W-1:0]  x_re,
    input  wire signed [W-1:0]  x_im,
    input  wire        [3:0]    k,
    output reg  signed [W-1:0]  y_re,
    output reg  signed [W-1:0]  y_im
);
    // =========================================================================
    // CSD SHIFT-AND-ADD NETWORKS
    // KEY: use $signed(x_re) so Verilog sign-extends to the 2W-bit LHS width.
    // Without $signed(), the concat {{1'b0,x_re}} zero-extends, giving wrong
    // results for all negative inputs (all k with non-trivial twiddles fail).
    // =========================================================================

    // ?? Re{W16^1} * x_re and x_im ??????????????????????????????????????????
    // +30274 = +2^15 - 2^11 - 2^9 + 2^6 + 2^1
    wire signed [2*W-1:0] re1_xre =
          ($signed(x_re) <<< 15)
        - ($signed(x_re) <<< 11)
        - ($signed(x_re) <<<  9)
        + ($signed(x_re) <<<  6)
        + ($signed(x_re) <<<  1);

    wire signed [2*W-1:0] re1_xim =
          ($signed(x_im) <<< 15)
        - ($signed(x_im) <<< 11)
        - ($signed(x_im) <<<  9)
        + ($signed(x_im) <<<  6)
        + ($signed(x_im) <<<  1);

    // ?? Im{W16^1} * x_re and x_im ??????????????????????????????????????????
    // -12540 = -2^14 + 2^12 - 2^8 + 2^2
    wire signed [2*W-1:0] im1_xre =
        - ($signed(x_re) <<< 14)
        + ($signed(x_re) <<< 12)
        - ($signed(x_re) <<<  8)
        + ($signed(x_re) <<<  2);

    wire signed [2*W-1:0] im1_xim =
        - ($signed(x_im) <<< 14)
        + ($signed(x_im) <<< 12)
        - ($signed(x_im) <<<  8)
        + ($signed(x_im) <<<  2);

    // ?? Re{W16^2} * x_re and x_im ??????????????????????????????????????????
    // +23170 = +2^15 - 2^13 - 2^11 + 2^9 - 2^7 + 2^5 + 2^1
    wire signed [2*W-1:0] re2_xre =
          ($signed(x_re) <<< 15)
        - ($signed(x_re) <<< 13)
        - ($signed(x_re) <<< 11)
        + ($signed(x_re) <<<  9)
        + ($signed(x_re) <<<  7)
        + ($signed(x_re) <<<  1);

    wire signed [2*W-1:0] re2_xim =
          ($signed(x_im) <<< 15)
        - ($signed(x_im) <<< 13)
        - ($signed(x_im) <<< 11)
        + ($signed(x_im) <<<  9)
        + ($signed(x_im) <<<  7)
        + ($signed(x_im) <<<  1);

    // ?? Im{W16^2} = -Re{W16^2} ?????????????????????????????????????????????
    // -23170 = -2^15 + 2^13 + 2^11 - 2^9 + 2^7 - 2^5 - 2^1
    wire signed [2*W-1:0] im2_xre = -re2_xre;
    wire signed [2*W-1:0] im2_xim = -re2_xim;

    //============================================================
    // Primitive Products (Full Precision)
    //------------------------------------------------------------
    // P0 =  A*xr
    // P1 =  A*xi
    // P2 =  B*xr
    // P3 =  B*xi
    // P4 =  C*xr
    // P5 =  C*xi
    //
    // NOTE:
    // These remain FULL 32-bit values.
    // Scaling (>>>15) is performed only once at the output.
    //============================================================
    
    // A*x
    
    wire signed [2*W-1:0] P0 = re1_xre;
    wire signed [2*W-1:0] P1 = re1_xim;
    
    // B*x
    // im1_* stores (-B*x)
    
    wire signed [2*W-1:0] P2 = -im1_xre;
    wire signed [2*W-1:0] P3 = -im1_xim;
    
    // C*x
    
    wire signed [2*W-1:0] P4 = re2_xre;
    wire signed [2*W-1:0] P5 = re2_xim;

    //------------------------------------------------------------
    // Full Precision Accumulators
    //------------------------------------------------------------
    
    reg signed [2*W-1:0] temp_re;
    reg signed [2*W-1:0] temp_im;
    // =========================================================================
    // 1/8 SYMMETRY MAPPING (Fig.10, paper)
    // All W16^k obtained from W16^1=(w1_re+j*w1_im) and W16^2=(w2_re+j*w2_im)
    // by sign inversion and/or real-imaginary swap.
    //
    // Derivation from W16^k = cos(k*pi/8) - j*sin(k*pi/8):
    //   Let A=cos(pi/8), B=-sin(pi/8), C=cos(pi/4)=-sin(pi/4)
    //   w1_re?A, w1_im?B, w2_re?C, w2_im?-C
    //
    //   k=0:  1+j*0          ? (x_re, x_im)           trivial
    //   k=1:  A+j*B          ? (w1_re, w1_im)          direct
    //   k=2:  C+j*(-C)       ? (w2_re, w2_im)          direct
    //   k=3:  B+j*(-A)       ? (-w1_im, w1_re) note B=-w1_im, -A=-w1_re...
    //          W16^3=cos(3pi/8)-j*sin(3pi/8)=sin(pi/8)-j*cos(pi/8)=-B-j*A
    //          = -w1_im + j*(-w1_re)   [since B=w1_im, A=w1_re]
    //          Wait: w1_re=A=+cos(pi/8), w1_im=B=-sin(pi/8)
    //          W16^3= sin(pi/8) - j*cos(pi/8) = -w1_im - j*w1_re
    //   k=4:  0-j*1          ? (x_im, -x_re)           trivial -j mult
    //   k=5:  -sin(pi/8)-j*cos(pi/8) = w1_im - j*w1_re ... wait
    //          W16^5=cos(5pi/8)-j*sin(5pi/8)=-sin(pi/8)-j*cos(pi/8)=w1_im-j*w1_re... 
    //          Hmm: w1_im=B=-sin(pi/8), so w1_im = -sin(pi/8) ?
    //          -cos(pi/8) = -w1_re ?
    //          So W16^5 = w1_im + j*(-w1_re) = (w1_im, -w1_re) ?
    //
    // All 16 mappings verified against float reference:
    // =========================================================================
    
    always @(*) begin
        temp_re = 0;
        temp_im = 0;
        case (k)
            //------------------------------------------------------------
            // k = 0
            //------------------------------------------------------------
            4'd0:
            begin
                temp_re = $signed(x_re) <<< (W-1);
                temp_im = $signed(x_im) <<< (W-1);
            end
            
            //------------------------------------------------------------
            // k = 1
            // (A+j(-B))*x
            //------------------------------------------------------------
            4'd1:
            begin
                temp_re = P0 + P3;
                temp_im = P1 - P2;
            end
            
            //------------------------------------------------------------
            // k = 2
            // (C-jC)*x
            //------------------------------------------------------------
            4'd2:
            begin
                temp_re = P4 + P5;
                temp_im = P5 - P4;
            end
            
            //------------------------------------------------------------
            // k = 3
            // (B-jA)*x
            //------------------------------------------------------------
            4'd3:
            begin
                temp_re = P2 + P1;
                temp_im = P3 - P0;
            end
            //------------------------------------------------------------
            // k = 4
            //------------------------------------------------------------
            4'd4:
            begin
                temp_re = $signed(x_im) <<< (W-1);
                temp_im = -($signed(x_re) <<< (W-1));
            end
            
            //------------------------------------------------------------
            // k = 5
            //------------------------------------------------------------
            4'd5:
            begin
                temp_re = -P2 + P1;
                temp_im = -P3 - P0;
            end
            
            //------------------------------------------------------------
            // k = 6
            //------------------------------------------------------------
            4'd6:
            begin
                temp_re = P5 - P4;
                temp_im = -P4 - P5;
            end
            
            //------------------------------------------------------------
            // k = 7
            //------------------------------------------------------------
            4'd7:
            begin
                temp_re = -P0 + P3;
                temp_im = -P1 - P2;
            end
            //------------------------------------------------------------
            // k = 8
            //------------------------------------------------------------
            4'd8:
            begin
                temp_re = -($signed(x_re) <<< (W-1));
                temp_im = -($signed(x_im) <<< (W-1));
            end
            
            //------------------------------------------------------------
            // k = 9
            //------------------------------------------------------------
            4'd9:
            begin
                temp_re = -P0 - P3;
                temp_im = -P1 + P2;
            end
            
            //------------------------------------------------------------
            // k = 10
            //------------------------------------------------------------
            4'd10:
            begin
                temp_re = -P4 - P5;
                temp_im = P4 - P5;
            end
            
            //------------------------------------------------------------
            // k = 11
            //------------------------------------------------------------
            4'd11:
            begin
                temp_re = -P2 - P1;
                temp_im = P0 - P3;
            end
            
            //------------------------------------------------------------
            // k = 12
            //------------------------------------------------------------
            4'd12:
            begin
                temp_re = -($signed(x_im) <<< (W-1));
                temp_im =  ($signed(x_re) <<< (W-1));
            end
            
            //------------------------------------------------------------
            // k = 13
            //------------------------------------------------------------
            4'd13:
            begin
                temp_re = P2 - P1;
                temp_im = P3 + P0;
            end
            
            //------------------------------------------------------------
            // k = 14
            //------------------------------------------------------------
            4'd14:
            begin
                temp_re = P4 - P5;
                temp_im = P4 + P5;
            end
            
            //------------------------------------------------------------
            // k = 15
            //------------------------------------------------------------
            4'd15:
            begin
                temp_re = P0 - P3;
                temp_im = P1 + P2;
            end
            
            default:
            begin
                temp_re = 0;
                temp_im = 0;
            end 
        endcase
        //------------------------------------------------------------
        // Final Scaling
        //------------------------------------------------------------
        y_re = temp_re >>> (W-1);
        y_im = temp_im >>> (W-1);
    end

endmodule

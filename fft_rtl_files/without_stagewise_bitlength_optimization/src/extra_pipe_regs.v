`timescale 1ns / 1ps
// =============================================================================
// Module : extra_pipe_regs
// Purpose: Pipeline padding register bank (paper Eq.21 latency alignment).
//
//          The Master Control Logic schedule is derived exclusively from the
//          paper's pipeline constants (Sec.V, Eq.21):
//              Cpipelined_stgI   = 32
//              Cpipelined_stgII  = 32
//              Cpipelined_stgIII = 13
//          The PE latencies actually implemented by the sub-modules are:
//              Stage-I  : 1 (mem reg rd) + 3 (bs16_unit) + 12 (cordic_rotator) = 16
//              Stage-II : 1 (mem reg rd) + 3 (bs16_unit) + 12 (cordic_rotator) = 16
//              Stage-III: 1 (mem reg rd) + 3 (bs16_unit)                       =  4
//          This module inserts DEPTH transparent register stages so the total
//          path latency equals the paper's constant:
//              Stage-I  pad : 32 - 16 = 16
//              Stage-II pad : 32 - 16 = 16
//              Stage-III pad: 13 -  4 =  9
//          (cordic_rotator = 12 CC: 1 input reg + 10 micro-rotation regs +
//           1 output scaling reg.)
//
//          Pure delay: no reordering, no arithmetic. 16 complex lanes plus a
//          valid flag are shifted through DEPTH register stages.
//
// Compatible: Verilog-2001, Vivado 2022.2
// =============================================================================
module extra_pipe_regs #(
    parameter W     = 16,   // complex word width (signed)
    parameter DEPTH = 16    // number of padding register stages (>= 1)
)(
    input  wire             clk,
    input  wire             rst,

    input  wire             valid_in,
    input  wire [W*16-1:0]  x_re_flat,
    input  wire [W*16-1:0]  x_im_flat,

    output wire             valid_out,
    output wire [W*16-1:0]  y_re_flat,
    output wire [W*16-1:0]  y_im_flat
);

    reg [W*16-1:0] pipe_re [0:DEPTH-1];
    reg [W*16-1:0] pipe_im [0:DEPTH-1];
    reg [DEPTH-1:0] pipe_v;

    integer k;
    always @(posedge clk) begin
        if (rst) begin
            pipe_v <= {DEPTH{1'b0}};
            for (k = 0; k < DEPTH; k = k + 1) begin
                pipe_re[k] <= {W*16{1'b0}};
                pipe_im[k] <= {W*16{1'b0}};
            end
        end else begin
            pipe_re[0] <= x_re_flat;
            pipe_im[0] <= x_im_flat;
            pipe_v[0]  <= valid_in;
            for (k = 1; k < DEPTH; k = k + 1) begin
                pipe_re[k] <= pipe_re[k-1];
                pipe_im[k] <= pipe_im[k-1];
                pipe_v[k]  <= pipe_v[k-1];
            end
        end
    end

    assign y_re_flat = pipe_re[DEPTH-1];
    assign y_im_flat = pipe_im[DEPTH-1];
    assign valid_out = pipe_v[DEPTH-1];

endmodule

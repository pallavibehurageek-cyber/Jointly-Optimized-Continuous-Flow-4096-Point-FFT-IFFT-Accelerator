`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 24.06.2026 23:55:56
// Design Name: 
// Module Name: lcs_unit
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
// Module : lcs_unit
// Purpose: Left Circular Shift of 16 parallel complex words (Fig.5)
//          Output port p gets input from port (p + shift) mod 16
//          Inverse of RCS. Combinational.
// =============================================================================
`timescale 1ns / 1ps

// =============================================================================
// Module : lcs_unit
// Purpose: Left Circular Shift of 16 parallel complex words
//          Output port p gets input from port (p + shift) mod 16
//          Verilog-2001 compatible (flattened buses)
// =============================================================================

module lcs_unit #(
    parameter W = 16,
    parameter P = 16
)(
    input  [W*P-1:0] x_re_flat,
    input  [W*P-1:0] x_im_flat,

    input  [3:0] shift,

    output reg [W*P-1:0] y_re_flat,
    output reg [W*P-1:0] y_im_flat
);

integer p;
integer src;

always @(*) begin

    y_re_flat = 0;
    y_im_flat = 0;

    for(p=0; p<P; p=p+1) begin

        // Left circular shift
        src = (p + shift) & 4'hF;

        y_re_flat[W*(P-1-p)+:W]
            = x_re_flat[W*(P-1-src)+:W];

        y_im_flat[W*(P-1-p)+:W]
            = x_im_flat[W*(P-1-src)+:W];

    end

end

endmodule

// =============================================================================
// Module : rcs_unit
// Purpose: Right Circular Shift of 16 parallel complex words (Fig.5)
//          Output port p gets input from port (p - shift) mod 16
//          Implemented as 16 wide MUXes driven by shift amount.
//          Combinational.
// =============================================================================
// =============================================================================
// Module : rcs_unit
// Purpose: Right Circular Shift of 16 parallel complex words (Fig.5)
//          Output port p gets input from port (p - shift) mod 16
//          Implemented as 16 wide MUXes driven by shift amount.
//          Combinational.
// =============================================================================
module rcs_unit #(
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

        src = (p - shift) & 4'hF;

        y_re_flat[W*(P-1-p)+:W]
            = x_re_flat[W*(P-1-src)+:W];

        y_im_flat[W*(P-1-p)+:W]
            = x_im_flat[W*(P-1-src)+:W];

    end

end

endmodule

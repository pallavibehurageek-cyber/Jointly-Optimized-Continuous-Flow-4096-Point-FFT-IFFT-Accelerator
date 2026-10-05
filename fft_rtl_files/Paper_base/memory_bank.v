// =============================================================================
// Module : memory_bank
// Purpose: Single dual-port SRAM bank (256 � 32-bit = 16-bit re + 16-bit im)
//          Write port: synchronous on posedge clk when wr_en=1
//          Read port : synchronous on posedge clk (registered output)
//          Vivado infers this as RAMB36 Block RAM automatically.
// =============================================================================
module memory_bank #(
    parameter W    = 16,    // word width per component
    parameter DEPTH= 256    // number of addresses
)(
    input  wire             clk,
    // Write port
    input  wire             wr_en,
    input  wire [$clog2(DEPTH)-1:0] wr_addr,
    input  signed [W-1:0]  wr_re,
    input  signed [W-1:0]  wr_im,
    // Read port
    input  wire [$clog2(DEPTH)-1:0] rd_addr,
    output reg  signed [W-1:0]  rd_re,
    output reg  signed [W-1:0]  rd_im
);
    reg signed [2*W-1:0] mem [0:DEPTH-1];

    // Write
    always @(posedge clk) begin
        if (wr_en)
            mem[wr_addr] <= {wr_re, wr_im};
    end

    // Read (registered: 1 CC latency)
    always @(posedge clk) begin
        rd_re <= mem[rd_addr][2*W-1:W];
        rd_im <= mem[rd_addr][W-1:0];
    end

    // Initialise to zero (for simulation)
    integer i;
    initial begin
        for (i=0; i<DEPTH; i=i+1) mem[i] = 0;
    end
endmodule

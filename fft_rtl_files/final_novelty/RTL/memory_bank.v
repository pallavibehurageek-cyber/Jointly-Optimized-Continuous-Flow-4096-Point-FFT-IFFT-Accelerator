// =============================================================================
// Module : memory_bank
// Purpose: Single dual-port SRAM bank (256 ? 32-bit = 16-bit re + 16-bit im)
//          Write port: synchronous on posedge clk when wr_en=1
//          Read port : synchronous on posedge clk (registered output)
//          Vivado infers this as RAMB36 Block RAM automatically.
// =============================================================================
module memory_bank #(
    parameter W     = 16,   // datapath word width per component (unchanged)
    parameter W_MEM = 16,   // NOVELTY #4: STORED word length (W_MEM <= W).
                            // Only the top W_MEM bits are stored; the low
                            // (W-W_MEM) bits are truncated on write and
                            // zero-filled on read. Datapath stays W-bit.
    parameter DEPTH = 256   // number of addresses
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
    reg signed [2*W_MEM-1:0] mem [0:DEPTH-1];

    // Write
    always @(posedge clk) begin
        if (wr_en)
            mem[wr_addr] <= {wr_re[W-1 -: W_MEM], wr_im[W-1 -: W_MEM]};
    end

    // Read (registered: 1 CC latency)
    always @(posedge clk) begin
        // Zero-fill the truncated LSBs. A left shift is used instead of a
        // concatenation with {(W-W_MEM){1'b0}}, because that becomes a
        // ZERO-WIDTH replication when W_MEM == W, which some synthesis tools
        // reject. The shift is portable for all W_MEM <= W.
        rd_re <= mem[rd_addr][2*W_MEM-1 -: W_MEM] << (W - W_MEM);
        rd_im <= mem[rd_addr][W_MEM-1   -: W_MEM] << (W - W_MEM);
    end

    // Initialise to zero (for simulation)
    integer i;
    initial begin
        for (i=0; i<DEPTH; i=i+1) mem[i] = 0;
    end
endmodule

`timescale 1ns / 1ps
// =============================================================================
// Module : stage1_datapath
// Purpose: Stage-I memory + processing-element datapath (Fig.4, Fig.5, Fig.6).
//          Combines:
//            aagu_stage1 (write side, set_sel)      -> write addr/shift
//            aagu_stage1 (read  side, ~set_sel)      -> read  addr/shift
//            rcs_unit  : routes 16 incoming lanes into the correct banks
//            16 x memory_bank (256 x 16b each)       : the Stage-I memory
//            lcs_unit  : undoes the routing, restores n1 (BS16 lane) order
//            bs16_unit : Stage-I radix-16 butterfly + CSD intra-twiddles
//
//          Sequencing (cycle_cnt, set_sel) is supplied externally for now;
//          the Master Control Logic that free-runs these will be added when
//          the full top-level controller is built. wr_en is also external,
//          so this module can be exercised as a pure combinational/pipelined
//          datapath in isolation.
//
// Latency note: memory_bank read output is registered (1 CC). pe_in_*_flat
// (the BS16/CORDIC processing-element input, exposed here for verification)
// is therefore valid 1 cycle after the corresponding read cycle_cnt/set_sel
// were presented. valid_in is delayed internally by 1 CC to line up with
// this before being passed into bs16_unit (which adds its own 3 CC).
//
// pe_in_re_flat / pe_in_im_flat are exposed purely for verification against
// Fig.2/Fig.6 (so the addressing+routing logic can be checked independently
// of BS16 math, which was already verified standalone).
//
// Compatible: Verilog-2001, Vivado 2022.2
// =============================================================================
module stage1_datapath #(
    parameter W = 16
)(
    input  wire             clk,
    // NOVELTY #1: separate compute-domain clock (gated independently of memory)
    input  wire             clk_c,
    input  wire             rst,

    // --- externally-driven sequencing (Master Control Logic stands in) ---
    input  wire [7:0]       cycle_cnt,   // shared write/read step counter
    input  wire             set_sel,     // which set is currently being WRITTEN
                                          // (read side automatically uses ~set_sel)
    input  wire             wr_en,
    input  wire             valid_in,    // qualifies this cycle's read request

    // --- incoming 16 lanes (true sample order, lane p = sample's n3 nibble) ---
    input  wire [W*16-1:0]  x_re_flat,
    input  wire [W*16-1:0]  x_im_flat,

    // --- processing-element input, post RCS/mem/LCS (for verification) ---
    output wire [W*16-1:0]  pe_in_re_flat,
    output wire [W*16-1:0]  pe_in_im_flat,
    output wire             pe_in_valid,

    // --- BS16 output ---
    output wire [W*16-1:0]  y_re_flat,
    output wire [W*16-1:0]  y_im_flat,
    output wire             valid_out
);

    // =========================================================================
    // Address generation: write side uses set_sel, read side uses ~set_sel,
    // both driven by the same shared cycle_cnt (see aagu_stage1.v header for
    // why this reproduces "read address of one set = write address of other").
    // =========================================================================
    wire [3:0]      shift_w;
    wire [16*8-1:0] addr_w_flat;
    aagu_stage1 u_aagu_w (
        .cycle_cnt (cycle_cnt),
        .set_sel   (set_sel),
        .rcs_shift (shift_w),
        .addr_flat (addr_w_flat)
    );

    wire [3:0]      shift_r;
    wire [16*8-1:0] addr_r_flat;
    aagu_stage1 u_aagu_r (
        .cycle_cnt (cycle_cnt),
        .set_sel   (set_sel),   // SAME set as the write side (paper Sec.III-A:
                                 // the read addresses "also serve as write
                                 // addresses for the next set" - in-place
                                 // alternation. Previous ~set_sel wiring made
                                 // reads use the SAME sequence the data was
                                 // written with (pass-through, no reordering).
        .rcs_shift (shift_r),
        .addr_flat (addr_r_flat)
    );

    // =========================================================================
    // Write-side RCS: route the 16 incoming lanes into bank order
    // =========================================================================
    wire [W*16-1:0] routed_re_flat, routed_im_flat;
    rcs_unit #(.W(W), .P(16)) u_rcs (
        .x_re_flat (x_re_flat),
        .x_im_flat (x_im_flat),
        .shift     (shift_w),
        .y_re_flat (routed_re_flat),
        .y_im_flat (routed_im_flat)
    );

    // =========================================================================
    // 16 x memory_bank (256 deep each). Bank b uses:
    //   write addr = addr_w_flat[8*(15-b) +: 8], write data = routed_*_flat[b]
    //   read  addr = addr_r_flat[8*(15-b) +: 8], read  data -> mem_rd_*_flat[b]
    // =========================================================================
    wire [W*16-1:0] mem_rd_re_flat, mem_rd_im_flat;

    genvar bk;
    generate
        for (bk = 0; bk < 16; bk = bk + 1) begin : BANKS
            wire [7:0] bank_addr_w = addr_w_flat[8*(15-bk) +: 8];
            wire [7:0] bank_addr_r = addr_r_flat[8*(15-bk) +: 8];

            memory_bank #(.W(W), .DEPTH(256)) u_mem (
                .clk     (clk),
                .wr_en   (wr_en),
                .wr_addr (bank_addr_w),
                .wr_re   (routed_re_flat[W*(15-bk) +: W]),
                .wr_im   (routed_im_flat[W*(15-bk) +: W]),
                .rd_addr (bank_addr_r),
                .rd_re   (mem_rd_re_flat[W*(15-bk) +: W]),
                .rd_im   (mem_rd_im_flat[W*(15-bk) +: W])
            );
        end
    endgenerate

    // =========================================================================
    // Read-side LCS: undo the routing, restore n1 (BS16 lane) order.
    // shift_r must be registered by 1 CC to stay aligned with mem_rd_*_flat,
    // which is itself memory_bank's registered (1 CC) read output.
    // =========================================================================
    reg [3:0] shift_r_d1;
    always @(posedge clk) begin
        if (rst) shift_r_d1 <= 4'd0;
        else     shift_r_d1 <= shift_r;
    end

    lcs_unit #(.W(W), .P(16)) u_lcs (
        .x_re_flat (mem_rd_re_flat),
        .x_im_flat (mem_rd_im_flat),
        .shift     (shift_r_d1),
        .y_re_flat (pe_in_re_flat),
        .y_im_flat (pe_in_im_flat)
    );

    reg valid_d1;
    always @(posedge clk) begin
        if (rst) valid_d1 <= 1'b0;
        else     valid_d1 <= valid_in;
    end
    assign pe_in_valid = valid_d1;

    // =========================================================================
    // Stage-I processing element: BS16 (CORDIC inter-stage twiddle is applied
    // externally downstream, after this PE, per Fig.4 -- not included here).
    // =========================================================================
    bs16_unit #(.W(W)) u_bs16 (
        .clk        (clk_c),
        .rst        (rst),
        .valid_in   (pe_in_valid),
        .x_re_flat  (pe_in_re_flat),
        .x_im_flat  (pe_in_im_flat),
        .y_re_flat  (y_re_flat),
        .y_im_flat  (y_im_flat),
        .valid_out  (valid_out)
    );

endmodule

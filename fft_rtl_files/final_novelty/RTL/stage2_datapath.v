`timescale 1ns / 1ps
// =============================================================================
// Module : stage2_datapath
// Purpose: Stage-II memory + processing-element datapath (Fig.4, Fig.5, Fig.7).
//          Mirrors stage1_datapath.v in structure; differences from Stage-I:
//            - cycle_cnt is 4-bit (0..15) - one inner counter per 16-set group
//            - Uses aagu_stage2 (4-bit addr, per-bank Addr_set2)
//            - memory_bank DEPTH=16 (4-bit address) - "16 banks, each 16?16"
//            - CORDIC inter-stage twiddle is applied downstream; pe_in_* exposed
//              for stage-isolated verification (same convention as stage1_datapath)
//
// Architecture (Sec. III-A.2, Fig.4, Fig.5):
//   aagu_stage2 (write, set_sel)       ? write addr + rcs_shift
//   aagu_stage2 (read, ~set_sel)       ? read addr + shift_r
//   rcs_unit     (shift_w)             ? routes 16 incoming lanes to banks
//   16 ? memory_bank(DEPTH=16)        ? Stage-II memory (256 locations total)
//   lcs_unit     (shift_r_d1)          ? restores lane order for BS16
//   bs16_unit                          ? Stage-II radix-16 butterfly + CSD twiddles
//
// Latency: 1 CC (memory registered read) + 3 CC (bs16_unit) = 4 CC from pe_in
//          to y_re/y_im_flat.  pe_in_* is valid 1 CC after the read cycle_cnt.
//
// Compatible: Verilog-2001, Vivado 2022.2
// =============================================================================
module stage2_datapath #(
    parameter W = 16,
    parameter W_MEM = 16   // NOVELTY #4: stored memory word length
)(
    input  wire             clk,
    // NOVELTY #1: separate compute-domain clock (gated independently of memory)
    input  wire             clk_c,
    input  wire             rst,

    // --- externally-driven sequencing (Master Control Logic placeholder) ---
    input  wire [3:0]       cycle_cnt,   // inner counter 0..15 (= n2 within group)
    input  wire             set_sel,     // which set is currently being WRITTEN
    input  wire             wr_en,
    input  wire             valid_in,

    // --- 16 incoming lanes (Stage-I outputs, one per k1) ---
    input  wire [W*16-1:0]  x_re_flat,
    input  wire [W*16-1:0]  x_im_flat,

    // --- PE input (post RCS/mem/LCS), exposed for verification ---
    output wire [W*16-1:0]  pe_in_re_flat,
    output wire [W*16-1:0]  pe_in_im_flat,
    output wire             pe_in_valid,

    // --- BS16 output ---
    output wire [W*16-1:0]  y_re_flat,
    output wire [W*16-1:0]  y_im_flat,
    output wire             valid_out
);
    // =========================================================================
    // Address generation
    // Stage-II addr_flat is 16 ? 4-bit (DEPTH=16 ? 4-bit address)
    // =========================================================================
    wire [3:0]      shift_w;
    wire [16*4-1:0] addr_w_flat;
    aagu_stage2 u_aagu_w (
        .cycle_cnt (cycle_cnt),
        .set_sel   (set_sel),
        .rcs_shift (shift_w),
        .addr_flat (addr_w_flat)
    );

    wire [3:0]      shift_r;
    wire [16*4-1:0] addr_r_flat;
    aagu_stage2 u_aagu_r (
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
    // Write-side RCS
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
    // 16 ? memory_bank, DEPTH=16 (4-bit address per bank)
    // =========================================================================
    wire [W*16-1:0] mem_rd_re_flat, mem_rd_im_flat;

    genvar bk;
    generate
        for (bk = 0; bk < 16; bk = bk + 1) begin : BANKS
            wire [3:0] bank_addr_w = addr_w_flat[4*(15-bk) +: 4];
            wire [3:0] bank_addr_r = addr_r_flat[4*(15-bk) +: 4];

            memory_bank #(.W(W), .W_MEM(W_MEM), .DEPTH(16)) u_mem (
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
    // Read-side LCS: register shift_r by 1 CC to align with memory output
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
    // Stage-II processing element: BS16
    // CORDIC inter-stage twiddle (Stage-II?Stage-III) applied downstream.
    // =========================================================================
    bs16_unit #(.W(W)) u_bs16 (
        .clk       (clk_c),
        .rst       (rst),
        .valid_in  (pe_in_valid),
        .x_re_flat (pe_in_re_flat),
        .x_im_flat (pe_in_im_flat),
        .y_re_flat (y_re_flat),
        .y_im_flat (y_im_flat),
        .valid_out (valid_out)
    );

endmodule

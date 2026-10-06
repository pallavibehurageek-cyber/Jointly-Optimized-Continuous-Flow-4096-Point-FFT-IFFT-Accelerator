`timescale 1ns / 1ps
// =============================================================================
// Module : aagu_stage1
// Purpose: Alternating Address Generating Unit (AAGU) for Stage-I memory
//          (Sec. III-A.1, Fig.6 of paper). Combinational address-generation
//          core implementing both:
//            Addr_set1 (natural up-counter order)  -- Fig.6(a)
//            Addr_set2 (ROM / permuted order)       -- Fig.6(b)
//          selected by set_sel. Drives the shared RCS/LCS shift amount and,
//          per physical bank, the row address to use this cycle.
//
// -----------------------------------------------------------------------------
// Derivation (verified against worked examples quoted in paper Sec. III-A.1):
//   For an arriving/departing sample with true index x (0..4095), write
//   x = {n1,n2,n3} as three 4-bit nibbles: n1=x[11:8], n2=x[7:4], n3=x[3:0].
//
//     Bank(x)      = (n1 + n3) mod 16                       (Eq.14)
//     Addr_set1(x) = {n1, n2}        = x[11:4]
//     Addr_set2(x) = {n3, n2}
//
//   In hardware, 16 samples x = 16*cycle_cnt + lane (lane = 0..15) arrive or
//   depart together every cycle, so n3 = lane, n2 = cycle_cnt[3:0],
//   n1 = cycle_cnt[7:4]. For a FIXED physical bank b, the lane currently
//   routed into it is
//       lane_for_bank(b) = (b - cycle_cnt[7:4]) mod 16
//   giving:
//       rcs_shift        = cycle_cnt[7:4]                 (same for both sets)
//       addr_set1[bank]  = cycle_cnt                       (one shared row)
//       addr_set2[bank]  = {lane_for_bank(bank), cycle_cnt[3:0]}
//
//   Hand-verified against the paper's text examples:
//     idx=1    set1 -> Bank1[0]                 idx=256  set2 -> Bank1[0]
//     idx=286  set1 -> Bank15[17]               idx=3601 set2 -> Bank15[17]
//     idx=271  set1 -> Bank0[16]                idx=3841 set2 -> Bank0[16]
//   All four are exhaustively re-checked (along with all 4096x2 index/set
//   combinations) in tb_aagu_stage1.v.
//
// Port convention: flat bus, bank b occupies slice [8*(15-b) +: 8], matching
// the MSB-first convention used in lcs_unit.v / rcs_unit.v.
// Compatible: Verilog-2001, Vivado 2022.2
// =============================================================================
module aagu_stage1 (
    input  wire [7:0]      cycle_cnt,  // 0..255: write group / read step counter
    input  wire            set_sel,    // 0 = Addr_set1 (Fig.6a), 1 = Addr_set2 (Fig.6b)
    output wire [3:0]      rcs_shift,  // shared RCS/LCS shift amount this cycle
    output wire [16*8-1:0] addr_flat   // 16 x 8-bit row addresses,
                                        // bank b at addr_flat[8*(15-b) +: 8]
);

    wire [3:0] shift = cycle_cnt[7:4];
    assign rcs_shift = shift;

    genvar b;
    generate
        for (b = 0; b < 16; b = b + 1) begin : ADDR_GEN
            wire [3:0] lane_for_bank = (b[3:0] - shift) & 4'hF;
            wire [7:0] addr_set1_b   = cycle_cnt;
            wire [7:0] addr_set2_b   = {lane_for_bank, cycle_cnt[3:0]};
            assign addr_flat[8*(15-b) +: 8] = set_sel ? addr_set2_b : addr_set1_b;
        end
    endgenerate

endmodule

`timescale 1ns / 1ps
// =============================================================================
// Module : aagu_stage3
// Purpose: Alternating Address Generating Unit (AAGU) for Stage-III memory.
//          (Sec. III-A.3, Fig.8 of paper)
//
// -----------------------------------------------------------------------------
// Derivation from paper (Sec. III-A.3):
//
//   Stage-III concurrent inputs differ by 1 -> n3 is the varying stage
//   variable within each BS16. The Stage-III input index of a Stage-II
//   output is z = 256*k1 + 16*k2 + n3.
//
//   Stage-II emits sample (k1,k2,n3) at stream cycle t = 16*n3 + k1 on
//   lane k2 (one k1 read-column per cycle within each n3 set - forced by
//   the Stage-II 16x16 in-place transpose). Hence at cycle t:
//       n3 = t[7:4]   (slow digit / set index)
//       k1 = t[3:0]   (fast digit / column within set)
//
//   "The required number of circular shifts for the stage-III data incoming
//    is determined by the bits b3b2b1b0 of the input index" -> shift = n3:
//       rcs_shift = cycle_cnt[7:4]
//       Bank(z)   = (k2 + n3) mod 16
//   conflict-free for the 16 concurrent writes (k2 = 0..15) AND for the
//   read groups (n3 = 0..15 with k1,k2 fixed).
//
//   Address sets (paper: 2nd-set index rearranged b11..b8 b3..b0 b7..b4,
//   i.e. mid and low nibbles swapped; frame-2 sample (k1',k2',n3')
//   occupies the frame-1 location of (k1', n3', k2') = same bank
//   (n3'+k2'), row {k2', k1'}):
//     Addr_set1[b] = cycle_cnt                        = {n3 , k1}
//     Addr_set2[b] = {(b - cycle_cnt[7:4]) mod 16, cycle_cnt[3:0]}
//                                                     = {k2 , k1}
//
//   READ schedule (both parities): at read cycle r the banks deliver the
//   group (k1, k2) = (r[3:0], r[7:4]) with n3 = (b - r[7:4]) varying
//   across banks; the LCS alignment shift is therefore ALSO cycle_cnt[7:4]
//   - one shift output serves both roles, exactly as in Stages I/II.
//   Resulting output permutation: bin k = k1 + 16*k2 + 256*k3 appears at
//   output cycle k[7:0] (= 16*k2 + k1), lane k3 = k[11:8].
//
//   In-place alternation closes with period 2 (set1 rows {n3,k1} <->
//   set2 rows {k2,k1}), enabling continuous flow: while frame F is read
//   with one set, frame F+1 is written at the same addresses.
//
// Verified against all Fig.8 paper examples (arrival cycle t = 16*n3 + k1):
//   Bank1[0]   1st-set: z=16  (k1=0,k2=1, n3=0)  t=0   Addr_set1 = 0      OK
//   Bank15[17] 1st-set: z=481 (k1=1,k2=14,n3=1)  t=17  Addr_set1 = 17     OK
//   Bank1[0]   2nd-set: z=1   (k1=0,k2=0, n3=1)  t=16  Addr_set2[1]  = 0  OK
//   Bank15[17] 2nd-set: z=286 (k1=1,k2=1, n3=14) t=225 Addr_set2[15] = 17 OK
//
// Port convention: bank b at addr_flat[8*(15-b) +: 8]  (MSB-first, 8-bit)
// Compatible: Verilog-2001, Vivado 2022.2
// =============================================================================
module aagu_stage3 (
    input  wire [7:0]      cycle_cnt,  // 0..255: write/read step counter
    input  wire            set_sel,    // 0 = Addr_set1, 1 = Addr_set2
    output wire [3:0]      rcs_shift,  // = cycle_cnt[7:4] = n3 (RCS and LCS)
    output wire [16*8-1:0] addr_flat   // 16 x 8-bit row addresses,
                                        // bank b at addr_flat[8*(15-b) +: 8]
);
    assign rcs_shift = cycle_cnt[7:4];

    genvar b;
    generate
        for (b = 0; b < 16; b = b + 1) begin : ADDR_GEN
            wire [3:0] lane_for_bank  = (b[3:0] - cycle_cnt[7:4]) & 4'hF;
            wire [7:0] addr_set1_b    = cycle_cnt;
            wire [7:0] addr_set2_b    = {lane_for_bank, cycle_cnt[3:0]};
            assign addr_flat[8*(15-b) +: 8] = set_sel ? addr_set2_b : addr_set1_b;
        end
    endgenerate

endmodule

`timescale 1ns / 1ps
// =============================================================================
// Module : aagu_stage2
// Purpose: Alternating Address Generating Unit (AAGU) for Stage-II memory.
//          (Sec. III-A.2, Fig.7 of paper)
//
// -----------------------------------------------------------------------------
// Derivation from paper (Sec. III-A.2):
//
//   Stage-II concurrent inputs differ by multiples of 16 ? n2 is the varying
//   stage variable within each BS16. The Stage-II "input index" of the s1
//   output is x = 256*k1 + �2 where k1 = Stage-I output bin (0..15) and
//   �2 = 16*n2 + n3.
//
//   RCS shift = b7b6b5b4 of input index = n2 = �2>>4.
//
//   Memory: "16 banks, each with a size of 16�16" ? DEPTH = 16 per bank
//   (4-bit address, 0..15). One 16-set group = 16 write cycles.
//
//   Within each 16-set group (fixed n3, varying n2 = cycle_cnt):
//     cycle_cnt = n2 = 0..15 (4-bit inner counter)
//     rcs_shift = cycle_cnt = n2
//
//   Address formulas (Sec. III-A.2, bit-rearrangement description):
//     Addr_set1[b] = cycle_cnt             (binary up-counter, same all banks)
//     Addr_set2[b] = (b - cycle_cnt) mod 16  (ROM, per-bank)
//
//   set_sel=0 ? write uses Addr_set1, read uses Addr_set2 (~set_sel)
//   set_sel=1 ? write uses Addr_set2, read uses Addr_set1 (~set_sel)
//
// Verified against all Fig.7 paper examples:
//   Bank0[0] 1st-set: idx=0   k1=0 �2=0  ? Addr_set1(cnt=0)=0          ?
//   Bank1[0] 1st-set: idx=256 k1=1 �2=0  ? Addr_set1(cnt=0)=0, bank=1  ?
//   Bank0[0] 2nd-set: idx=1   k1=0 �2=1  ? Addr_set2[0](cnt=0)=0       ?
//   Bank1[0] 2nd-set: idx=17  k1=0 �2=17 ? Addr_set2[1](cnt=1)=0       ?
//   Bank0[0] 3rd-set: idx=2   k1=0 �2=2  ? Addr_set1(cnt=0)=0          ?
//   Bank1[0] 3rd-set: idx=258 k1=1 �2=2  ? Addr_set1(cnt=0)=0, bank=1  ?
//
// Port convention: bank b at addr_flat[4*(15-b) +: 4]  (MSB-first, 4-bit)
// Compatible: Verilog-2001, Vivado 2022.2
// =============================================================================
module aagu_stage2 (
    input  wire [3:0]      cycle_cnt,  // inner counter 0..15 (= n2 of the set)
    input  wire            set_sel,    // 0 = write Addr_set1 / read Addr_set2
                                        // 1 = write Addr_set2 / read Addr_set1
    output wire [3:0]      rcs_shift,  // = cycle_cnt (n2 for Stage-II)
    output wire [16*4-1:0] addr_flat   // 16 � 4-bit row addresses,
                                        // bank b at addr_flat[4*(15-b) +: 4]
);
    assign rcs_shift = cycle_cnt;

    genvar b;
    generate
        for (b = 0; b < 16; b = b + 1) begin : ADDR_GEN
            wire [3:0] addr_set1_b = cycle_cnt;
            wire [3:0] addr_set2_b = (b[3:0] - cycle_cnt) & 4'hF;
            assign addr_flat[4*(15-b) +: 4] = set_sel ? addr_set2_b : addr_set1_b;
        end
    endgenerate

endmodule

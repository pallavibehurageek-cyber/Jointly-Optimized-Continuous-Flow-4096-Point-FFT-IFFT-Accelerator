`timescale 1ns / 1ps
// =============================================================================
// Module : clock_gate  (integrated clock-gating cell, latch-based)
//   Novelty #1 primitive. gclk pulses only when `en` is asserted.
//   Enable is captured on the LOW phase of clk (transparent-low latch) so the
//   gated clock is glitch-free. Maps to a UMC 65 nm ICG cell in synthesis;
//   in simulation it behaves identically. When en=0 the driven registers hold.
// =============================================================================
module clock_gate (
    input  wire clk,
    input  wire en,
    output wire gclk
);
    reg en_lat;
    always @(*) if (!clk) en_lat = en;   // transparent-low latch
    assign gclk = clk & en_lat;
endmodule

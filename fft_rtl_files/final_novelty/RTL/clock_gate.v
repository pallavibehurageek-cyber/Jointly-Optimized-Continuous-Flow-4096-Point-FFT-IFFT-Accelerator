//`timescale 1ns / 1ps
//// =============================================================================
//// Module : clock_gate  (integrated clock-gating cell, latch-based)
////   Novelty #1 primitive. gclk pulses only when `en` is asserted.
////   Enable is captured on the LOW phase of clk (transparent-low latch) so the
////   gated clock is glitch-free. Maps to a UMC 65 nm ICG cell in synthesis;
////   in simulation it behaves identically. When en=0 the driven registers hold.
//// =============================================================================
//module clock_gate (
//    input  wire clk,
//    input  wire en,
//    output wire gclk
//);
//    reg en_lat;
//    always @(*) if (!clk) en_lat = en;   // transparent-low latch
//    assign gclk = clk & en_lat;
//endmodule
`timescale 1ns / 1ps
// =============================================================================
// Module : clock_gate  (integrated clock-gating cell)
//
//   Novelty #1 primitive. gclk pulses only when `en` is asserted.
//   Enable is captured on the LOW phase of clk (transparent-low latch) so the
//   gated clock is glitch-free. When en=0 the driven registers hold.
//
//   TWO VIEWS, FUNCTIONALLY IDENTICAL:
//
//   1. SYNTHESIS (`define SYN_ICG) -- instantiates the gsclib045 integrated
//      clock-gating cell TLATNCAX8 directly. Required so that:
//        - report_clock_gating COUNTS these gates (the discrete latch+AND
//          version reported 0 gating instances / "Enable not found" for all
//          289,158 flops, because Genus only recognises cells carrying the
//          clock_gating_integrated_cell attribute);
//        - Genus's power engine models the reduced clock-pin power on the
//          gated flops -- without this, the whole benefit of novelty #1 is
//          invisible in report_power;
//        - the gate is guaranteed glitch-free by construction rather than by
//          hoping the tool preserves a hand-built latch+AND;
//        - clock-gating setup/hold checks populate the CG path group.
//
//      Library attribute : clock_gating_integrated_cell : "latch_posedge"
//        -> enable latched while CK is LOW, gates clock for POSEDGE flops.
//        -> matches the behavioural model below exactly.
//      Pins : CK (clock in), E (enable in), ECK (gated clock out).
//
//   2. SIMULATION (default) -- behavioural transparent-low latch + AND.
//      Keeps simulation independent of the technology library, so the
//      existing continuous-flow testbench runs unchanged.
//
//   DRIVE STRENGTH NOTE: X8 chosen as a mid-range starting point. These gates
//   sit at the root of large clock domains; Innovus CTS will buffer downstream
//   of the ICG, so the ICG's own strength is not the limiting factor. If the
//   post-CTS clock tree shows excessive insertion delay at a gate, step up to
//   TLATNCAX12/16/20. Available: X2 X3 X4 X6 X8 X12 X16 X20.
//
//   SCAN NOTE: TLATNCAX* has NO test-enable pin. If scan insertion is added
//   later, switch to TLATNTSCAX8, which provides a TE input that must be tied
//   so the clock is forced on during scan shift.
// =============================================================================
module clock_gate (
    input  wire clk,
    input  wire en,
    output wire gclk
);

`ifdef SYN_ICG
    // ---- Technology view: real integrated clock-gating cell ----------------
    TLATNCAX8 u_icg (
        .CK  (clk),
        .E   (en),
        .ECK (gclk)
    );
`else
    // ---- Behavioural view: transparent-low latch + AND ---------------------
    reg en_lat;
    always @(*) if (!clk) en_lat = en;   // transparent-low latch
    assign gclk = clk & en_lat;
`endif

endmodule

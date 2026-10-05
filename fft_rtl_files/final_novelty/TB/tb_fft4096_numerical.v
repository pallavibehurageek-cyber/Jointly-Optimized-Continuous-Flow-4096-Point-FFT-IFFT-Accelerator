`timescale 1ns / 1ps
// =============================================================================
// Testbench : tb_fft4096_numerical
// Purpose   : Strict numerical verification of fft4096_top (single-frame flow)
//
// ---------------------------------------------------------------------------
// INPUT SCALING REQUIREMENT (paper Sec.V):
//   The paper converts input samples to Block-Floating-Point (BFP) format
//   before hardware processing; the fixed-point core assumes conditioned
//   inputs. The architecture's internal scaling is 1/64 total (>>1 in each
//   of the two radix-4 layers of each of the three BS16 stages), so a
//   full-scale spectral peak is X = N*A/64 = 64*A. For 16-bit outputs this
//   requires A <= 511 for a single complex tone (A <= 32767 for an impulse,
//   whose spectrum is flat at A/64). Amplitudes above this wrap inside the
//   Stage-III BS16 and smear energy across k3 in steps of 4 - the exact
//   signature previously observed with an unconditioned tone stimulus.
//
// OUTPUT ORDERING (fixed by the AAGU read schedules, Sec.III-A):
//   Writing the bin index as k = k1 + 16*k2 + 256*k3
//   (k1 = k[3:0], k2 = k[7:4], k3 = k[11:8]), bin k appears at
//       output cycle = k[7:0] = 16*k2 + k1   (cycle 0..255 within the
//                                             256-cycle output burst)
//       output lane  = k3 = k[11:8]          (0..15)
//   i.e. capture index 16*cycle + lane holds bin (256*lane + cycle).
//
// FRAME PROTOCOL (single-frame controller):
//   - Assert SOP for one cycle; frame data cycle 0 must be presented the
//     cycle AFTER SOP is sampled (= mc 0). 256 data cycles follow.
//   - First output at mc=605 (Eq.21), 256 output cycles, idle after mc=860.
//   - Next SOP only after the current frame fully drains (>= 861 CC frame
//     period) until the continuous-flow controller revision.
//
// TESTS (each an independent frame; frames also regression-test frame
// independence of the in-place memory alternation):
//   1  DC          x[n] = 500              -> bin 0    : re ~ 32000
//   2  Impulse     x[0] = 32767            -> ALL bins : re ~ 511, im ~ 0
//   3  Real cosine A=400, f0 = bin 256     -> bins 256 & 3840 : re ~ 12800
//   4  Complex exp A=400, f0 = bin 256     -> bin 256  : re ~ 25600
//   5  Complex exp A=400, f0 = bin 805     -> bin 805  : re ~ 25600
//      (bin 805 = (k1,k2,k3) = (5,2,3): all three digits nonzero,
//       exercises both CORDIC twiddle sets and a k3 != 0 output)
//
// Compatible: Verilog-2001, Vivado 2022.2 XSim (uses #1 after posedge),
//             Icarus Verilog (cross-checked)
// =============================================================================
module tb_fft4096_numerical;
    localparam W = 16;
    // Expected first-output latency. Fork-B (CORDIC=12, pads removed) = 564.
    // Baseline (Eq.21 padded) = 605; after reduced-cycle CORDIC (9) = 558.
    localparam EXP_LAT = 558;

    reg clk = 0, rst = 1, sop = 0;
    reg fft_ifft = 0;
    reg  [W*16-1:0] x_re, x_im;
    wire eop, valid_out;
    wire [W*16-1:0] y_re, y_im;

    fft4096_top #(.W(W)) dut (
        .clk(clk), .rst(rst), .sop(sop), .fft_ifft(fft_ifft),
        .x_re_flat(x_re), .x_im_flat(x_im),
        .eop(eop), .valid_out(valid_out),
        .y_re_flat(y_re), .y_im_flat(y_im)
    );

    always #5 clk = ~clk;

    // ------------------------------------------------------------------------
    // Test bookkeeping
    // ------------------------------------------------------------------------
    integer cyc, lane, vcnt, errs, first_v_time, t0_time;
    integer pass_cnt, fail_cnt;
    real    ph;

    // stimulus parameters for the current frame
    integer stim_mode;   // 0=DC 1=impulse 2=real cosine 3=complex exp
    integer stim_amp;
    integer stim_bin;

    // expected hot bins (up to 2): location and value
    integer nhot;
    integer hot_cyc [0:1];
    integer hot_lane[0:1];
    integer hot_re  [0:1];
    integer hot_im  [0:1];

    // captured output frame
    reg signed [W-1:0] out_re [0:4095];
    reg signed [W-1:0] out_im [0:4095];

    integer i, k, idx, exp_r, exp_i, tol, is_hot;
    integer leak_thresh;

    // ------------------------------------------------------------------------
    // Drive one frame: SOP, then 256 data cycles starting at mc=0
    // ------------------------------------------------------------------------
    task set_input(input integer md, input integer amp, input integer fb,
                   input integer cc);
        begin
                for (lane = 0; lane < 16; lane = lane + 1) begin
                    // sample index n = 16*cc + lane (natural input order)
                    case (md)
                    0: begin // DC
                        x_re[W*(15-lane) +: W] = amp[15:0];
                        x_im[W*(15-lane) +: W] = 16'sd0;
                    end
                    1: begin // impulse at n = 0
                        x_re[W*(15-lane) +: W] =
                            (cyc==0 && lane==0) ? amp[15:0] : 16'sd0;
                        x_im[W*(15-lane) +: W] = 16'sd0;
                    end
                    2: begin // real cosine at bin stim_bin
                        ph = 2.0*3.14159265358979*fb*(16.0*cc+lane)/4096.0;
                        x_re[W*(15-lane) +: W] = $rtoi(1.0*amp*$cos(ph));
                        x_im[W*(15-lane) +: W] = 16'sd0;
                    end
                    3: begin // complex exponential at bin stim_bin
                        ph = 2.0*3.14159265358979*fb*(16.0*cc+lane)/4096.0;
                        x_re[W*(15-lane) +: W] = $rtoi(1.0*amp*$cos(ph));
                        x_im[W*(15-lane) +: W] = $rtoi(1.0*amp*$sin(ph));
                    end
                    4: begin // IFFT input: single spectral line stim_amp*e^{j*pi/4} at bin stim_bin
                        if (16*cc+lane == fb) begin
                            x_re[W*(15-lane) +: W] = $rtoi(1.0*amp*$cos(0.78539816));
                            x_im[W*(15-lane) +: W] = $rtoi(1.0*amp*$sin(0.78539816));
                        end else begin
                            x_re[W*(15-lane) +: W] = 16'sd0;
                            x_im[W*(15-lane) +: W] = 16'sd0;
                        end
                    end
                    default: begin // IFFT input: conj-symmetric pair at bins stim_bin, 4096-stim_bin
                        if (16*cc+lane == fb || 16*cc+lane == 4096-fb) begin
                            x_re[W*(15-lane) +: W] = amp[15:0];
                            x_im[W*(15-lane) +: W] = 16'sd0;
                        end else begin
                            x_re[W*(15-lane) +: W] = 16'sd0;
                            x_im[W*(15-lane) +: W] = 16'sd0;
                        end
                    end
                    endcase
                end
        end
    endtask

    task send_frame;
        begin
            @(posedge clk); #1;
            sop = 1;
            @(posedge clk); #1;      // sop sampled at this edge
            sop = 0;
            t0_time = $time;         // start of mc = 0
            for (cyc = 0; cyc < 256; cyc = cyc + 1) begin
                set_input(stim_mode, stim_amp, stim_bin, cyc);
                @(posedge clk); #1;  // data for cyc written at this edge
            end
            x_re = 0; x_im = 0;
        end
    endtask

    // ------------------------------------------------------------------------
    // Capture 256 output cycles into out_re/out_im (index = 16*cycle + lane)
    // ------------------------------------------------------------------------
    task capture_frame;
        begin
            vcnt = 0; first_v_time = -1;
            while (vcnt < 256) begin
                @(posedge clk); #1;
                if (valid_out) begin
                    if (first_v_time < 0) first_v_time = $time;
                    for (lane = 0; lane < 16; lane = lane + 1) begin
                        out_re[16*vcnt + lane] = y_re[W*(15-lane) +: W];
                        out_im[16*vcnt + lane] = y_im[W*(15-lane) +: W];
                    end
                    vcnt = vcnt + 1;
                end
            end
            $display("  first-output latency = %0d CC (expected %0d)",
                     (first_v_time - t0_time)/10, EXP_LAT);
            repeat (30) @(posedge clk);  // drain before the next frame
        end
    endtask

    // ------------------------------------------------------------------------
    // Check: expected hot bins (nhot entries) within tolerance, all other
    // positions below the leakage threshold. Impulse uses the flat check.
    // ------------------------------------------------------------------------
    task check_frame(input integer flat, input integer flat_re);
        begin
            errs = 0;
            if (flat) begin
                for (i = 0; i < 4096; i = i + 1) begin
                    if (out_re[i] < flat_re - 40 || out_re[i] > flat_re + 40 ||
                        out_im[i] >  40          || out_im[i] < -40) begin
                        errs = errs + 1;
                        if (errs <= 5)
                            $display("  FLAT mismatch idx=%0d re=%0d im=%0d",
                                     i, out_re[i], out_im[i]);
                    end
                end
            end else begin
                for (i = 0; i < 4096; i = i + 1) begin
                    is_hot = 0;
                    for (k = 0; k < nhot; k = k + 1) begin
                        idx = 16*hot_cyc[k] + hot_lane[k];
                        if (i == idx) begin
                            is_hot = 1;
                            exp_r = hot_re[k];  exp_i = hot_im[k];
                            tol = ((exp_r<0?-exp_r:exp_r)
                                 + (exp_i<0?-exp_i:exp_i)) / 20 + 64; // 5% + 64
                            if (out_re[i] < exp_r - tol || out_re[i] > exp_r + tol ||
                                out_im[i] < exp_i - tol || out_im[i] > exp_i + tol) begin
                                errs = errs + 1;
                                $display("  HOT-BIN mismatch cyc=%0d lane=%0d: got (%0d,%0d) expected (%0d,%0d)+-%0d",
                                         hot_cyc[k], hot_lane[k],
                                         out_re[i], out_im[i], exp_r, exp_i, tol);
                            end else
                                $display("  hot bin OK  cyc=%0d lane=%0d re=%0d im=%0d (expected %0d,%0d)",
                                         hot_cyc[k], hot_lane[k],
                                         out_re[i], out_im[i], exp_r, exp_i);
                        end
                    end
                    if (!is_hot) begin
                        if ((out_re[i]<0?-out_re[i]:out_re[i]) +
                            (out_im[i]<0?-out_im[i]:out_im[i]) > leak_thresh) begin
                            errs = errs + 1;
                            if (errs <= 5)
                                $display("  LEAKAGE idx=%0d (cyc=%0d lane=%0d) re=%0d im=%0d",
                                         i, i/16, i%16, out_re[i], out_im[i]);
                        end
                    end
                end
            end
            if (errs == 0) begin
                pass_cnt = pass_cnt + 1;
                $display("  >>> PASS");
            end else begin
                fail_cnt = fail_cnt + 1;
                $display("  >>> FAIL (%0d errors)", errs);
            end
        end
    endtask

    // ------------------------------------------------------------------------
    // Pointwise IFFT check: y[m] must equal a complex exponential (mode=0:
    // amp*e^{j(2*pi*f*m/4096 + pi/4)}) or a real cosine (mode=1:
    // amp*cos(2*pi*f*m/4096)). Output sample m sits at capture index
    // 16*cyc+lane with m = 256*lane + cyc (same permutation as the FFT
    // bins - it is the same read schedule).
    // ------------------------------------------------------------------------
    integer m, ptol;
    task check_ifft_pointwise(input integer cosmode, input integer amp, input integer fbin);
        begin
            errs = 0;
            ptol = amp/10 + 24;   // 10% + 24 LSB (CORDIC/truncation noise floor)
            for (i = 0; i < 4096; i = i + 1) begin
                m = 256*(i%16) + (i/16);
                if (cosmode) begin
                    ph = 2.0*3.14159265358979*fbin*m/4096.0;
                    exp_r = $rtoi(1.0*amp*$cos(ph));
                    exp_i = 0;
                end else begin
                    ph = 2.0*3.14159265358979*fbin*m/4096.0 + 0.78539816;
                    exp_r = $rtoi(1.0*amp*$cos(ph));
                    exp_i = $rtoi(1.0*amp*$sin(ph));
                end
                if (out_re[i] < exp_r-ptol || out_re[i] > exp_r+ptol ||
                    out_im[i] < exp_i-ptol || out_im[i] > exp_i+ptol) begin
                    errs = errs + 1;
                    if (errs <= 5)
                        $display("  IFFT mismatch m=%0d (cyc=%0d lane=%0d): got (%0d,%0d) expected (%0d,%0d)+-%0d",
                                 m, i/16, i%16, out_re[i], out_im[i], exp_r, exp_i, ptol);
                end
            end
            if (errs == 0) begin pass_cnt = pass_cnt + 1; $display("  >>> PASS (4096 samples pointwise)"); end
            else           begin fail_cnt = fail_cnt + 1; $display("  >>> FAIL (%0d errors)", errs); end
        end
    endtask

    // ------------------------------------------------------------------------
    // Background capture (multi-frame tests): records up to 4 frames of
    // outputs, per-frame EOP times, and output-burst continuity.
    // ------------------------------------------------------------------------
    reg  cap_en = 0;
    integer cap_ptr = 0, cap_target = 0, cap_started = 0, cap_gap = 0;
    integer eop_n = 0;
    integer eop_time [0:7];
    integer clane, f;
    reg signed [W-1:0] cap_re [0:16383];
    reg signed [W-1:0] cap_im [0:16383];

    always @(posedge clk) begin
        #1;
        if (cap_en) begin
            if (eop && eop_n < 8) begin
                eop_time[eop_n] = $time;
                eop_n = eop_n + 1;
            end
            if (valid_out && cap_ptr < cap_target) begin
                cap_started = 1;
                for (clane = 0; clane < 16; clane = clane + 1) begin
                    cap_re[16*cap_ptr + clane] = y_re[W*(15-clane) +: W];
                    cap_im[16*cap_ptr + clane] = y_im[W*(15-clane) +: W];
                end
                cap_ptr = cap_ptr + 1;
            end else if (cap_started && cap_ptr < cap_target)
                cap_gap = cap_gap + 1;   // idle cycle inside the burst
        end
    end

    task cap_reset(input integer target);
        begin
            cap_ptr = 0; cap_started = 0; cap_gap = 0; eop_n = 0;
            cap_target = target;
            cap_en = 1;
        end
    endtask

    // copy captured frame f into out_re/out_im for the existing checkers
    task load_cap_frame(input integer fr);
        begin
            for (i = 0; i < 4096; i = i + 1) begin
                out_re[i] = cap_re[4096*fr + i];
                out_im[i] = cap_im[4096*fr + i];
            end
        end
    endtask

    // helper: expected location of bin k (see header)
    // cycle = 16*k[3:0] + k[7:4]; lane = k[11:8]
    function integer bin_cycle(input integer b); bin_cycle = b % 256;    endfunction
    function integer bin_lane (input integer b); bin_lane  = (b/256)%16; endfunction

    initial begin
        x_re = 0; x_im = 0;
        pass_cnt = 0; fail_cnt = 0;
        leak_thresh = 600;
        repeat (4) @(posedge clk); #1;
        rst = 0;
        repeat (2) @(posedge clk);

        $display("====================================================");
        $display("  4096-pt FFT Numerical Verification (strict)");
        $display("  Amplitude rule: |X_peak| = 64*A <= 32767 -> A <= 511");
        $display("====================================================");

        // ---- TEST 1: DC, amplitude 500 -> bin 0 = 32000 ----
        $display("TEST 1: DC x[n]=500  (expect bin 0 -> cycle 0 lane 0, re~32000)");
        stim_mode = 0; stim_amp = 500;
        nhot = 1;
        hot_cyc[0]=bin_cycle(0); hot_lane[0]=bin_lane(0); hot_re[0]=32000; hot_im[0]=0;
        send_frame; capture_frame; check_frame(0, 0);

        // ---- TEST 2: impulse 32767 -> flat 511 ----
        $display("TEST 2: Impulse x[0]=32767  (expect ALL bins re~511, im~0)");
        stim_mode = 1; stim_amp = 32767;
        send_frame; capture_frame; check_frame(1, 511);

        // ---- TEST 3: real cosine bin 256, A=400 -> bins 256 & 3840 = 12800 ----
        $display("TEST 3: cos, bin 256, A=400  (expect bins 256 & 3840 -> cycle 0 lanes 1 & 15, re~12800)");
        stim_mode = 2; stim_amp = 400; stim_bin = 256;
        nhot = 2;
        hot_cyc[0]=bin_cycle(256);  hot_lane[0]=bin_lane(256);  hot_re[0]=12800; hot_im[0]=0;
        hot_cyc[1]=bin_cycle(3840); hot_lane[1]=bin_lane(3840); hot_re[1]=12800; hot_im[1]=0;
        send_frame; capture_frame; check_frame(0, 0);

        // ---- TEST 4: complex exp bin 256, A=400 -> bin 256 = 25600 ----
        $display("TEST 4: exp, bin 256, A=400  (expect bin 256 -> cycle 0 lane 1, re~25600)");
        stim_mode = 3; stim_amp = 400; stim_bin = 256;
        nhot = 1;
        hot_cyc[0]=bin_cycle(256); hot_lane[0]=bin_lane(256); hot_re[0]=25600; hot_im[0]=0;
        send_frame; capture_frame; check_frame(0, 0);

        // ---- TEST 5: complex exp bin 805 (k1=5,k2=2,k3=3), A=400 ----
        $display("TEST 5: exp, bin 805, A=400  (expect cycle 37 lane 3, re~25600)");
        stim_mode = 3; stim_amp = 400; stim_bin = 805;
        nhot = 1;
        hot_cyc[0]=bin_cycle(805); hot_lane[0]=bin_lane(805); hot_re[0]=25600; hot_im[0]=0;
        send_frame; capture_frame; check_frame(0, 0);

        // ---- TEST 6: IFFT, single spectral line 32000*e^{j*pi/4} at bin 37 ----
        // Eq.2: y = conj(FFT(conj(X)))/64 -> y[m] = 500*e^{j(2*pi*37*m/4096+pi/4)}
        // Complex amplitude exercises the INPUT conjugate MUX; bin 37 makes
        // even output cycle 0 complex, exercising the (en|eop) OUTPUT
        // conjugate MUX timing on the very first output cycle.
        $display("TEST 6: IFFT, X[37]=32000*e^{j*pi/4}  (expect y[m]=500*e^{j(2*pi*37*m/4096+pi/4)}, all m)");
        fft_ifft = 1;
        stim_mode = 4; stim_amp = 32000; stim_bin = 37;
        send_frame; capture_frame;
        check_ifft_pointwise(0, 500, 37);

        // ---- TEST 7: IFFT, conjugate-symmetric pair -> pure real cosine ----
        // X[37] = X[4059] = 16000 -> y[m] = 500*cos(2*pi*37*m/4096), im ~ 0
        $display("TEST 7: IFFT, X[37]=X[4059]=16000  (expect y[m]=500*cos(2*pi*37*m/4096), im~0)");
        fft_ifft = 1;
        stim_mode = 5; stim_amp = 16000; stim_bin = 37;
        send_frame; capture_frame;
        check_ifft_pointwise(1, 500, 37);
        fft_ifft = 0;

        // =================================================================
        // TEST 8: CONTINUOUS FLOW - 4 back-to-back frames (SOP every 256)
        //   F0 exp bin 256 A=400 | F1 exp bin 805 A=400 | F2 DC 500 |
        //   F3 impulse 32767. Expect: 4 EOPs spaced exactly 256 CC (first
        //   at 605), one gapless 1024-cycle output burst, and each frame
        //   numerically identical to its single-frame result.
        // =================================================================
        $display("TEST 8: CONTINUOUS FLOW - 4 back-to-back frames");
        cap_reset(1024);
        @(posedge clk); #1;
        sop = 1;
        @(posedge clk); #1;
        sop = 0;
        t0_time = $time;                 // start of frame 0 (mc = 0)
        for (f = 0; f < 4; f = f + 1) begin
            for (cyc = 0; cyc < 256; cyc = cyc + 1) begin
                case (f)
                0: set_input(3, 400, 256, cyc);
                1: set_input(3, 400, 805, cyc);
                2: set_input(0, 500, 0,   cyc);
                default: set_input(1, 32767, 0, cyc);
                  end

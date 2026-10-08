`timescale 1ns/1ps
// Occupancy sweep: drives frames at a given duty cycle and counts posedges on
// each gated clock domain vs the ungated reference. Clock-domain activity
// factor = gated_edges / ungated_edges  ->  dynamic clock/register power ratio.
module tb_occ;
  localparam W=16;
  reg clk=0, rst=1, sop=0, fft_ifft=0;
  reg [W*16-1:0] x_re, x_im;
  wire eop, valid_out; wire [W*16-1:0] y_re, y_im;
  fft4096_top #(.W(W)) dut(.clk(clk),.rst(rst),.sop(sop),.fft_ifft(fft_ifft),
    .x_re_flat(x_re),.x_im_flat(x_im),.eop(eop),.valid_out(valid_out),
    .y_re_flat(y_re),.y_im_flat(y_im));
  always #5 clk=~clk;

  integer n_m1,n_b1,n_c1,n_m2,n_b2,n_c2,n_m3,n_b3,n_ref;
  integer w,cyc,i,duty,period,nwin;
  real f_m1,f_b1,f_c1,f_m2,f_b2,f_c2,f_m3,f_b3;

  // count posedges on each gated clock
  always @(posedge dut.gclk_m1) n_m1=n_m1+1;
  always @(posedge dut.gclk_b1) n_b1=n_b1+1;
  always @(posedge dut.gclk_c1) n_c1=n_c1+1;
  always @(posedge dut.gclk_m2) n_m2=n_m2+1;
  always @(posedge dut.gclk_b2) n_b2=n_b2+1;
  always @(posedge dut.gclk_c2) n_c2=n_c2+1;
  always @(posedge dut.gclk_m3) n_m3=n_m3+1;
  always @(posedge dut.gclk_b3) n_b3=n_b3+1;
  always @(posedge clk)          n_ref=n_ref+1;

  task run_occ(input integer one_in_n, input integer nframes);
    begin
      n_m1=0;n_b1=0;n_c1=0;n_m2=0;n_b2=0;n_c2=0;n_m3=0;n_b3=0;n_ref=0;
      rst=1; @(posedge clk); #1; @(posedge clk); #1; rst=0;
      n_m1=0;n_b1=0;n_c1=0;n_m2=0;n_b2=0;n_c2=0;n_m3=0;n_b3=0;n_ref=0;
      nwin = nframes*one_in_n;
      for (w=0; w<nwin; w=w+1) begin
        // assert SOP only on every one_in_n-th window
        for (cyc=0; cyc<256; cyc=cyc+1) begin
          sop = (cyc==0) && ((w % one_in_n)==0);
          if ((w % one_in_n)==0) x_re = {16{16'sd500}}; else x_re = 0;
          @(posedge clk); #1;
        end
      end
      sop=0; x_re=0;
      // drain the pipeline
      for (i=0;i<900;i=i+1) begin @(posedge clk); #1; end
      f_m1=100.0*n_m1/n_ref; f_b1=100.0*n_b1/n_ref; f_c1=100.0*n_c1/n_ref;
      f_m2=100.0*n_m2/n_ref; f_b2=100.0*n_b2/n_ref; f_c2=100.0*n_c2/n_ref;
      f_m3=100.0*n_m3/n_ref; f_b3=100.0*n_b3/n_ref;
      $display("  1-in-%0d (%0.1f%%): M1=%0.1f B1=%0.1f C1=%0.1f | M2=%0.1f B2=%0.1f C2=%0.1f | M3=%0.1f B3=%0.1f",
               one_in_n, 100.0/one_in_n, f_m1,f_b1,f_c1,f_m2,f_b2,f_c2,f_m3,f_b3);
    end
  endtask

  initial begin
    x_re=0; x_im=0; fft_ifft=0;
    $display("=== Gated-clock activity factor per domain (%% of ungated edges) ===");
    run_occ(1, 12);
    run_occ(2, 8);
    run_occ(4, 6);
    run_occ(8, 4);
    run_occ(16, 3);
    $finish;
  end
endmodule

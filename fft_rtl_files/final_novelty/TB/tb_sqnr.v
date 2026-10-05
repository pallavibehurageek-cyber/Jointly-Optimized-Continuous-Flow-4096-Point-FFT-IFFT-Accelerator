`timescale 1ns/1ps
// System-level SQNR harness + per-stage signal-occupancy instrumentation.
// Feeds OFDM-like random frames; dumps input/output for Python reference FFT;
// records peak |value| at each stage boundary to expose unused MSB headroom.
module tb_sqnr;
  localparam W=16, NF=2;
  parameter AMP=511;
  parameter WM=16;   // W_MEM under test              // frames
  reg clk=0, rst=1, sop=0, fft_ifft=0;
  reg [W*16-1:0] x_re, x_im;
  wire eop, valid_out; wire [W*16-1:0] y_re, y_im;
  fft4096_top #(.W(W), .W_MEM1(WM), .W_MEM2(WM), .W_MEM3(WM)) dut(.clk(clk),.rst(rst),.sop(sop),.fft_ifft(fft_ifft),
    .x_re_flat(x_re),.x_im_flat(x_im),.eop(eop),.valid_out(valid_out),
    .y_re_flat(y_re),.y_im_flat(y_im));
  always #5 clk=~clk;

  integer f,c,l,vc,i,seed;
  reg signed [W-1:0] ire[0:NF*4096-1], iim[0:NF*4096-1];
  reg signed [W-1:0] ore[0:NF*4096-1], oim[0:NF*4096-1];

  // per-stage peak magnitude (absolute value) trackers
  integer pk_s1, pk_s2, pk_s3, pk_in;
  task upd(inout integer pk, input signed [W-1:0] v);
    begin if (v>0 && v>pk) pk=v; else if (v<0 && -v>pk) pk=-v; end
  endtask
  integer k;
  always @(posedge clk) if(!rst) begin
    for (k=0;k<16;k=k+1) begin
      upd(pk_in, x_re[W*(15-k)+:W]);
      upd(pk_s1, dut.s1c_re[W*(15-k)+:W]);   // Stage-I  CORDIC out
      upd(pk_s2, dut.s2c_re[W*(15-k)+:W]);   // Stage-II CORDIC out
      upd(pk_s3, dut.s3_y_re[W*(15-k)+:W]);  // Stage-III BS16 out
    end
  end

  // simple LCG for repeatable pseudo-random input
  function [31:0] rnd; input dummy; begin seed=(seed*1103515245+12345)&32'h7FFFFFFF; rnd=seed; end endfunction

  initial begin
    pk_in=0; pk_s1=0; pk_s2=0; pk_s3=0; seed=12345;
    x_re=0; x_im=0;
    repeat(4)@(posedge clk); #1; rst=0; repeat(2)@(posedge clk);
    for (f=0; f<NF; f=f+1) begin
      @(posedge clk); #1; sop=1; @(posedge clk); #1; sop=0;
      for (c=0;c<256;c=c+1) begin
        for (l=0;l<16;l=l+1) begin
          // OFDM-like: near-uniform random, amplitude within the A<=511 rule
          ire[f*4096+16*c+l] = (rnd(0)%(2*AMP+1))-AMP;
          iim[f*4096+16*c+l] = (rnd(0)%(2*AMP+1))-AMP;
          x_re[W*(15-l)+:W] = ire[f*4096+16*c+l];
          x_im[W*(15-l)+:W] = iim[f*4096+16*c+l];
        end
        @(posedge clk); #1;
      end
      x_re=0; x_im=0;
      vc=0;
      while (vc<256) begin
        @(posedge clk); #1;
        if (valid_out) begin
          for (l=0;l<16;l=l+1) begin
            ore[f*4096+16*vc+l]=y_re[W*(15-l)+:W];
            oim[f*4096+16*vc+l]=y_im[W*(15-l)+:W];
          end
          vc=vc+1;
        end
      end
      repeat(20)@(posedge clk);
    end
    $writememh("sq_ire.hex",ire); $writememh("sq_iim.hex",iim);
    $writememh("sq_ore.hex",ore); $writememh("sq_oim.hex",oim);
    $display("PEAK |value| per stage boundary (16-bit range = 32767):");
    $display("  input=%0d  stage1_out=%0d  stage2_out=%0d  stage3_out=%0d",
             pk_in, pk_s1, pk_s2, pk_s3);
    $display("SQNR_DUMP_COMPLETE");
    $finish;
  end
endmodule

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% derive_w16_mapping.m
%
% Purpose:
%   Derive the symmetry mapping of W16^k
%   used in csd_mult_w16.v
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
clear;

fprintf("=====================================================\n");
fprintf("      W16 TWIDDLE SYMMETRY DERIVATION\n");
fprintf("=====================================================\n\n");

fprintf("Definitions\n");
fprintf("-----------------------------------------------\n");
fprintf("w1 = cos(pi/8) - j*sin(pi/8)\n");
fprintf("w2 = cos(pi/4) - j*sin(pi/4)\n\n");

A = cos(pi/8);
B = sin(pi/8);
C = cos(pi/4);

fprintf("A = cos(pi/8) = %.12f\n",A);
fprintf("B = sin(pi/8) = %.12f\n",B);
fprintf("C = cos(pi/4) = %.12f\n\n",C);

fprintf("RTL variables are\n");
fprintf("w1_re = +A\n");
fprintf("w1_im = -B\n");
fprintf("w2_re = +C\n");
fprintf("w2_im = -C\n\n");

fprintf("=============================================================\n");
fprintf(" k        W16^k\n");
fprintf("=============================================================\n");

for k=0:15

    W = exp(-1j*2*pi*k/16);

    fprintf("%2d   %10.6f %+10.6fj\n",...
        k,real(W),imag(W));

end

fprintf("\n");
fprintf("=============================================================\n");
fprintf("EXPECTED RTL CASE TABLE\n");
fprintf("=============================================================\n\n");

fprintf("k=0  : ( x_re , x_im )\n");
fprintf("k=1  : ( w1_re ,  w1_im )\n");
fprintf("k=2  : ( w2_re ,  w2_im )\n");
fprintf("k=3  : ???\n");
fprintf("k=4  : ( x_im , -x_re )\n");
fprintf("k=5  : ???\n");
fprintf("k=6  : ???\n");
fprintf("k=7  : ???\n");
fprintf("k=8  : (-x_re , -x_im )\n");
fprintf("k=9  : ???\n");
fprintf("k=10 : ???\n");
fprintf("k=11 : ???\n");
fprintf("k=12 : (-x_im , x_re )\n");
fprintf("k=13 : ???\n");
fprintf("k=14 : ???\n");
fprintf("k=15 : ???\n");

fprintf("\n");

fprintf("=============================================================\n");
fprintf("REFERENCE VALUES\n");
fprintf("=============================================================\n");

fprintf("w1_re = %.6f\n", A);
fprintf("w1_im = %.6f\n",-B);

fprintf("w2_re = %.6f\n", C);
fprintf("w2_im = %.6f\n",-C);

fprintf("\n");

fprintf("Notice:\n");
fprintf("Only k = {1,2} are primitive.\n");
fprintf("Every other exponent must be generated\n");
fprintf("using sign changes and/or swapping.\n");

fprintf("\nDone.\n");

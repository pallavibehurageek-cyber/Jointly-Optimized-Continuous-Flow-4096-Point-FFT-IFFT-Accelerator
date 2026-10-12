%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% verify_table3_csd.m
%
% Purpose:
%   Verify the CSD coefficients given in Table III of the paper.
%
%   The paper gives:
%
%       Re(W16^0) = 32768
%       Re(W16^1) = 30274
%       Im(W16^1) = -12540
%       Re(W16^2) = 23170
%
%   We compare them with the RTL implementation.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
clear;

fprintf("============================================\n");
fprintf(" VERIFY TABLE III CSD COEFFICIENTS\n");
fprintf("============================================\n\n");

%% ------------------------------------------------------------------------
%% RTL IMPLEMENTATION
%% ------------------------------------------------------------------------

rtl_re0 = ...
      2^15;

rtl_re1 = ...
      2^15 ...
    - 2^11 ...
    - 2^9 ...
    + 2^6 ...
    + 2^1;

rtl_im1 = ...
    - 2^14 ...
    + 2^12 ...
    - 2^8 ...
    + 2^2;

rtl_re2 = ...
      2^15 ...
    - 2^13 ...
    - 2^11 ...
    + 2^9 ...
    - 2^7 ...
    + 2^5 ...
    + 2^1;

%% ------------------------------------------------------------------------
%% PAPER VALUES
%% ------------------------------------------------------------------------

paper_re0 = 32768;
paper_re1 = 30274;
paper_im1 = -12540;
paper_re2 = 23170;

%% ------------------------------------------------------------------------
%% PRINT RESULTS
%% ------------------------------------------------------------------------

fprintf("--------------------------------------------\n");
fprintf("Re(W16^0)\n");
fprintf("--------------------------------------------\n");
fprintf("RTL      : %6d\n",rtl_re0);
fprintf("Paper    : %6d\n",paper_re0);
fprintf("Difference: %6d\n\n",paper_re0-rtl_re0);

fprintf("--------------------------------------------\n");
fprintf("Re(W16^1)\n");
fprintf("--------------------------------------------\n");
fprintf("RTL      : %6d\n",rtl_re1);
fprintf("Paper    : %6d\n",paper_re1);
fprintf("Difference: %6d\n\n",paper_re1-rtl_re1);

fprintf("--------------------------------------------\n");
fprintf("Im(W16^1)\n");
fprintf("--------------------------------------------\n");
fprintf("RTL      : %6d\n",rtl_im1);
fprintf("Paper    : %6d\n",paper_im1);
fprintf("Difference: %6d\n\n",paper_im1-rtl_im1);

fprintf("--------------------------------------------\n");
fprintf("Re(W16^2)\n");
fprintf("--------------------------------------------\n");
fprintf("RTL      : %6d\n",rtl_re2);
fprintf("Paper    : %6d\n",paper_re2);
fprintf("Difference: %6d\n\n",paper_re2-rtl_re2);

%% ------------------------------------------------------------------------
%% SUMMARY
%% ------------------------------------------------------------------------

fprintf("============================================\n");

if rtl_re0 == paper_re0
    fprintf("Re(W16^0) : PASS\n");
else
    fprintf("Re(W16^0) : FAIL\n");
end

if rtl_re1 == paper_re1
    fprintf("Re(W16^1) : PASS\n");
else
    fprintf("Re(W16^1) : FAIL\n");
end

if rtl_im1 == paper_im1
    fprintf("Im(W16^1) : PASS\n");
else
    fprintf("Im(W16^1) : FAIL\n");
end

if rtl_re2 == paper_re2
    fprintf("Re(W16^2) : PASS\n");
else
    fprintf("Re(W16^2) : FAIL\n");
end

fprintf("============================================\n");

%% ------------------------------------------------------------------------
%% EXTRA INFORMATION
%% ------------------------------------------------------------------------

fprintf("\n");
fprintf("RTL Re(W16^2) constant is implemented as:\n");
fprintf("\n");
fprintf("2^15 - 2^13 - 2^11 + 2^9 - 2^7 + 2^5 + 2^1\n");
fprintf("\n");
fprintf("Decimal value = %d\n",rtl_re2);

fprintf("\n");
fprintf("Paper claims decimal value = %d\n",paper_re2);

fprintf("\n");
fprintf("Difference = %d\n",paper_re2-rtl_re2);

fprintf("\nDone.\n");

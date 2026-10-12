%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% derive_mapping.m
%
% Purpose
% -------
% Determine whether every W16^k multiplication can be generated from
% primitive multipliers W1 and W2 using only:
%
%   + sign changes
%   + swapping real/imaginary parts
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
clc;

fprintf("=============================================================\n");
fprintf("      VERIFYING PAPER'S TWIDDLE SYMMETRY MAPPING\n");
fprintf("=============================================================\n\n");

%% ------------------------------------------------------------
%% Arbitrary complex input
%% ------------------------------------------------------------

xr = 1000;
xi = 500;

x = xr + 1j*xi;

%% ------------------------------------------------------------
%% Primitive multipliers
%% ------------------------------------------------------------

W1 = exp(-1j*pi/8);
W2 = exp(-1j*pi/4);

Y1 = x*W1;
Y2 = x*W2;

w1_re = real(Y1);
w1_im = imag(Y1);

w2_re = real(Y2);
w2_im = imag(Y2);

fprintf("Primitive Outputs\n");
fprintf("---------------------------------------------\n");
fprintf("W1 = (%10.4f , %10.4f)\n",w1_re,w1_im);
fprintf("W2 = (%10.4f , %10.4f)\n\n",w2_re,w2_im);

%% ------------------------------------------------------------
%% Candidate mappings
%% ------------------------------------------------------------

cand = {};

% identity
cand{end+1} = @(a,b)[ a  b];
cand{end+1} = @(a,b)[ a -b];
cand{end+1} = @(a,b)[-a  b];
cand{end+1} = @(a,b)[-a -b];

% swapped
cand{end+1} = @(a,b)[ b  a];
cand{end+1} = @(a,b)[ b -a];
cand{end+1} = @(a,b)[-b  a];
cand{end+1} = @(a,b)[-b -a];

names = {
"( a ,  b )"
"( a , -b )"
"(-a ,  b )"
"(-a , -b )"
"( b ,  a )"
"( b , -a )"
"(-b ,  a )"
"(-b , -a )"
};

%% ------------------------------------------------------------
%% Check every exponent
%% ------------------------------------------------------------

tol = 1e-6;

fprintf("=============================================================\n");

for k=0:15

    W = exp(-1j*2*pi*k/16);

    Y = x*W;

    Yr = real(Y);
    Yi = imag(Y);

    fprintf("\n");
    fprintf("-------------------------------------------------\n");
    fprintf("k = %2d\n",k);
    fprintf("Expected = (%10.4f , %10.4f)\n",Yr,Yi);

    found = false;

    %----------------------------------------------------------
    % W1 candidates
    %----------------------------------------------------------

    for n=1:length(cand)

        tmp = cand{n}(w1_re,w1_im);

        if abs(tmp(1)-Yr)<tol && abs(tmp(2)-Yi)<tol

            fprintf("Matches W1 mapping : %s\n",names{n});
            found = true;

        end

    end

    %----------------------------------------------------------
    % W2 candidates
    %----------------------------------------------------------

    for n=1:length(cand)

        tmp = cand{n}(w2_re,w2_im);

        if abs(tmp(1)-Yr)<tol && abs(tmp(2)-Yi)<tol

            fprintf("Matches W2 mapping : %s\n",names{n});
            found = true;

        end

    end

    if ~found
        fprintf("NO SIGN/SWAP MAPPING EXISTS\n");
    end

end

fprintf("\n");
fprintf("=============================================================\n");
fprintf("Finished.\n");
fprintf("=============================================================\n");

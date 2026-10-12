%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% verify_new_architecture.m
%
% Verifies the reconstructed architecture using
% primitive products P0-P5.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
clear;

fprintf('==============================================\n');
fprintf(' VERIFY NEW CSD MULTIPLIER ARCHITECTURE\n');
fprintf('==============================================\n\n');

xr = 1000;
xi = 500;

x = xr + 1j*xi;

A = cos(pi/8);
B = sin(pi/8);
C = cos(pi/4);

% Primitive products
P0 = A*xr;
P1 = A*xi;
P2 = B*xr;
P3 = B*xi;
P4 = C*xr;
P5 = C*xi;

for k = 0:15

    W = exp(-1j*2*pi*k/16);
    y_ref = x*W;

    switch k

        case 0
            yr = xr;
            yi = xi;

        case 1
            yr = P0 + P3;
            yi = P1 - P2;

        case 2
            yr = P4 + P5;
            yi = P5 - P4;

        case 3
            yr = P2 + P1;
            yi = P3 - P0;

        case 4
            yr = xi;
            yi = -xr;

        case 5
            yr = -P2 + P1;
            yi = -P3 - P0;

        case 6
            yr = P5 - P4;
            yi = -P4 - P5;

        case 7
            yr = -P0 + P3;
            yi = -P1 - P2;

        case 8
            yr = -xr;
            yi = -xi;

        case 9
            yr = -P0 - P3;
            yi = -P1 + P2;

        case 10
            yr = -P4 - P5;
            yi = P4 - P5;

        case 11
            yr = -P2 - P1;
            yi = P0 - P3;

        case 12
            yr = -xi;
            yi = xr;

        case 13
            yr = P2 - P1;
            yi = P3 + P0;

        case 14
            yr = P4 - P5;
            yi = P4 + P5;

        case 15
            yr = P0 - P3;
            yi = P1 + P2;

    end

    fprintf('k=%2d\n',k);
    fprintf('Reference : (%10.4f,%10.4f)\n',real(y_ref),imag(y_ref));
    fprintf('Derived   : (%10.4f,%10.4f)\n',yr,yi);

    err = abs(real(y_ref)-yr)+abs(imag(y_ref)-yi);

    if err < 1e-9
        fprintf('PASS\n\n');
    else
        fprintf('FAIL\n\n');
    end

end

fprintf('Done.\n');

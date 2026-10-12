%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% csd_mult_reference.m
%
% Reference model for csd_mult_w16 RTL
%
% Computes:
%
%   y = x * W16^k
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function y = csd_mult_reference(x,k)

% 16-point FFT twiddle

W16 = exp(-1j*2*pi/16);

% Exact twiddle multiplication

y = x * W16^k;

% RTL uses integer arithmetic

y = round(real(y)) + 1j*round(imag(y));

end

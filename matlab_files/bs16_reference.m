function y = bs16_reference(x)

%-------------------------------------------------------------
% BS16 Golden Reference
%
% Version 1:
% Mathematical reference for RTL verification.
%
% Input:
%   x : 16x1 complex vector
%
% Output:
%   y : 16x1 complex vector
%
% RTL scaling:
%   Radix-4 Stage-I : /2
%   Radix-4 Stage-II: /2
%
% Overall scaling = /4
%-------------------------------------------------------------

x = x(:);

y = fft(x,16);

%y = round(y/4);
y = fix(y/4);
end

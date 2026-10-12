%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% integer_to_csd.m
%
% General Integer -> Canonical Signed Digit (CSD) Converter
%
% Usage:
%   coeff = 23170;
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
clear;

%% ==============================================================
%% CHANGE THIS NUMBER
%% ==============================================================

coeff = 23170;

%% ==============================================================

fprintf("============================================\n");
fprintf(" INTEGER TO CSD CONVERTER\n");
fprintf("============================================\n\n");

fprintf("Target coefficient : %d\n\n",coeff);

%% ----------------------------------------------------------------
% Binary representation
%% ----------------------------------------------------------------

N = 16;

b = zeros(1,N+1);

temp = coeff;

for i = N:-1:1
    b(i) = mod(temp,2);
    temp = floor(temp/2);
end

fprintf("Binary:\n");

for i=1:N
    fprintf("%d",b(i));
end

fprintf("\n\n");

%% ----------------------------------------------------------------
% Canonical Signed Digit Conversion
%% ----------------------------------------------------------------

csd = zeros(1,N+1);

carry = 0;

for i = N:-1:1

    sum = b(i) + carry;

    if sum==0

        csd(i)=0;
        carry=0;

    elseif sum==1

        if i>1

            if b(i-1)==1

                csd(i)=-1;
                carry=1;

            else

                csd(i)=1;
                carry=0;

            end

        else

            csd(i)=1;
            carry=0;

        end

    elseif sum==2

        csd(i)=0;
        carry=1;

    end

end

csd(1)=csd(1)+carry;

%% ----------------------------------------------------------------
% Print CSD digits
%% ----------------------------------------------------------------

fprintf("CSD Digits\n");
fprintf("--------------------------------------------\n");

for i=1:N

    if csd(i)==1

        fprintf("+1 ");

    elseif csd(i)==-1

        fprintf("-1 ");

    else

        fprintf(" 0 ");

    end

end

fprintf("\n\n");

%% ----------------------------------------------------------------
% Print Shift Add Network
%% ----------------------------------------------------------------

fprintf("Shift-Add Network\n");
fprintf("--------------------------------------------\n\n");

value = 0;

for i=1:N

    power = N-i;

    if csd(i)==1

        fprintf("+2^%2d\n",power);

        value = value + 2^power;

    elseif csd(i)==-1

        fprintf("-2^%2d\n",power);

        value = value - 2^power;

    end

end

fprintf("\n");

fprintf("Reconstructed Value = %d\n",value);

fprintf("Target Value        = %d\n",coeff);

fprintf("Difference          = %d\n",coeff-value);

fprintf("\n");

%% ----------------------------------------------------------------
% Generate Verilog
%% ----------------------------------------------------------------

fprintf("--------------------------------------------\n");
fprintf("Generated Verilog\n");
fprintf("--------------------------------------------\n\n");

fprintf("wire signed [31:0] coeff =\n");

for i=1:N

    power = N-i;

    if csd(i)==1

        fprintf("      ($signed(x) <<< %2d)\n",power);

    elseif csd(i)==-1

        fprintf("    - ($signed(x) <<< %2d)\n",power);

    end

end

fprintf(";\n");

fprintf("\nDone.\n");

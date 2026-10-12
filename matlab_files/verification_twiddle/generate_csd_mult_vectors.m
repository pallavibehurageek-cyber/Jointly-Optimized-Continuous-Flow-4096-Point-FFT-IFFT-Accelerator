%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% generate_csd_mult_vectors.m
%
% Generates verification vectors for csd_mult_w16
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
clear;

fprintf("=========================================\n");
fprintf(" CSD Multiplier Verification Generator\n");
fprintf("=========================================\n\n");

%%-------------------------------------------------------------
%% Create folders if needed
%%-------------------------------------------------------------

if ~exist("vectors_twiddle","dir")
    mkdir("vectors_twiddle");
end

if ~exist("expected_twiddle","dir")
    mkdir("expected_twiddle");
end

%%-------------------------------------------------------------
%% Test Inputs
%%-------------------------------------------------------------

tests = {

16384 + 0j
0 + 16384j
1000 + 500j

};

names = {

"real"
"imag"
"complex"

};

%%-------------------------------------------------------------
%% Generate vectors
%%-------------------------------------------------------------

for t = 1:length(tests)

    x = tests{t};

    fprintf("Generating %s vectors...\n",names{t});

    for k = 0:15

        y = csd_mult_reference(x,k);

        input_file = sprintf(...
            "vectors_twiddle/%s_k%02d_input.txt",...
            names{t},k);

        output_file = sprintf(...
            "expected_twiddle/%s_k%02d_output.txt",...
            names{t},k);

        %%-----------------------
        %% Save Input
        %%-----------------------

        fid = fopen(input_file,'w');

        fprintf(fid,"%d %d\n",...
            round(real(x)),...
            round(imag(x)));

        fclose(fid);

        %%-----------------------
        %% Save Expected
        %%-----------------------

        fid = fopen(output_file,'w');

        fprintf(fid,"%d %d\n",...
            round(real(y)),...
            round(imag(y)));

        fclose(fid);

    end

end

fprintf("\n");
fprintf("=========================================\n");
fprintf("48 Twiddle Tests Generated Successfully\n");
fprintf("=========================================\n");
